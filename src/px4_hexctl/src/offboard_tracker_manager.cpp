/**
 * @file offboard_tracker_manager.cpp
 * @brief 无人机飞控管理节点 - Offboard 模式自主起飞与追踪控制
 * 
 * 功能描述:
 * --------
 * 本节点是 AprilTag 追踪系统的飞控中枢，负责:
 * 1. 处理无人机的启动序列 (安全解锁、起飞至指定高度)
 * 2. 监听视觉追踪节点的相对位置目标或速度命令
 * 3. 根据 EKF 状态自适应选择控制模式 (POSITION 或 ATTITUDE)
 * 4. 保持 OFFBOARD 信号连续性 (防止飞控切回 Manual 模式)
 * 5. 在 Ctrl+C 中断时安全降落
 * 
 * 工作流程:
 * --------
 * Phase 1 - 启动序列 (startup_sequence):
 *   1. 检测 EKF 状态 (XY 位置有效性)
 *   2. 若 EKF 无效: 切到 ATTITUDE 模式，降低健康检查门槛
 *   3. 若 EKF 有效: 使用 POSITION 模式进行精准控制
 *   4. 预热控制信号 (1 秒)
 *   5. 循环申请 OFFBOARD 和 ARM，直到两者同时激活
 *   6. 执行起飞 (attitude 模式: 推力 0.68 → 0.58)
 *   7. 等待 EKF 收敛 (最多 5 秒)
 *   8. 若 EKF 有效: 切到 POSITION，悬停在目标高度
 *   9. 若 EKF 无效且 allow_xy_invalid=true: 保持 ATTITUDE 模式
 * 
 * Phase 2 - 主控制循环 (main):
 *   1. 订阅视觉追踪输入 (相对位置或速度命令)
 *   2. 定时检查 OFFBOARD/ARM 状态，丢失时恢复
 *   3. 根据 EKF 状态选择控制模式:
 *      - XY 有效: 优先使用相对位置目标 → 转换为位置指令
 *      - XY 有效: 备选使用速度命令
 *      - XY 有效: 无输入时保持悬停位置
 *      - XY 无效: 强制 ATTITUDE 模式悬停 (推力 0.58)
 *   4. 检查命令新鲜度 (超时>0.5s 视为失效)
 *   5. 发送控制指令至 PX4 固件
 *   6. 维持 20Hz 主循环频率
 * 
 * 关键参数:
 * ---------
 * - takeoff_alt: 起飞高度 (1.2m)
 * - max_altitude: 最大允许高度 (2.0m) - 高度围栏上限
 * - min_altitude: 最小允许高度 (0.3m) - 防止撞地
 * - z_hold_kp: XY 无效时的高度保持比例系数
 * - z_hold_max_vz: XY 无效时的最大竖直速度
 * - z_hold_deadband: 高度保持死区
 * - z_cmd_max_vz: 外部 z 速度限幅
 * - takeoff_ramp_time: 起飞推力平滑时间 (s)
 * - takeoff_slow_zone: 接近目标高度的减速区间 (m)
 * - takeoff_thrust_min: 起飞推力最低值 (接近目标高度时)
 * - command_timeout: 命令超时阈值 (0.5s)
 * - allow_xy_invalid: 是否允许 GPS 缺失情况下起飞 (true)
 * - hover_thrust: 悬停推力 (0.58)
 * 
 * 信号处理:
 * --------
 * - 当用户按下 Ctrl+C 时，设置 g_signal_triggered 标志
 * - 主循环检测到标志后发布降落命令
 * - 所有资源正确释放
 * 
 * @author px4-ros2-uavctl team
 * @date 2026-02
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>

#include "px4_hexctl/offboard_control.hpp"
#include "px4_hexctl/vehicle.hpp"

#include <chrono>
#include <thread>
#include <iostream>
#include <csignal>
#include <atomic>
#include <mutex>
#include <cmath>

using namespace std::chrono_literals;

/// 全局信号标志 - 由 Ctrl+C 处理器设置
std::atomic<bool> g_signal_triggered(false);

/**
 * @brief POSIX 信号处理器 - 捕获 Ctrl+C
 * 
 * 当用户按下 Ctrl+C 时，设置全局标志以触发优雅退出
 */
void signal_handler(int signum) {
    (void)signum;
    g_signal_triggered = true;
}

/**
 * @struct CmdState
 * @brief 速度命令状态容器 (线程安全)
 * 
 * 保存从 /qr_tracker/cmd_vel_body 接收的最新速度命令
 * 通过互斥锁保护防止数据竞争
 */
struct CmdState {
    geometry_msgs::msg::Twist last_cmd{};      ///< 最后接收的速度命令
    rclcpp::Time last_time{0, 0, RCL_ROS_TIME}; ///< 命令时间戳
    bool have_cmd{false};                       ///< 是否收到过命令
    std::mutex mutex;                           ///< 访问保护
    rclcpp::Time last_log_time{0, 0, RCL_ROS_TIME}; ///< 上次日志输出时间
};

/**
 * @struct TargetState
 * @brief 相对位置目标状态容器 (线程安全)
 * 
 * 保存从 /qr_tracker/relative_position 接收的最新相对位置目标
 * 优先级高于速度命令
 */
struct TargetState {
    geometry_msgs::msg::PointStamped last_target{};  ///< 最后接收的相对位置目标
    rclcpp::Time last_time{0, 0, RCL_ROS_TIME};       ///< 目标时间戳
    bool have_target{false};                          ///< 是否收到过目标
    std::mutex mutex;                                 ///< 访问保护
};

/**
 * @struct StartupResult
 * @brief 启动序列返回结果
 */
struct StartupResult {
    bool success{false};     ///< 启动是否成功
    double home_z{0.0};      ///< 起飞时的高度基准 (m)
    bool xy_valid{false};    ///< 启动后 EKF XY 是否有效
};

/**
 * @brief 启动序列函数 - 安全的自主起飞和模式切换
 * 
 * 职责:
 * 1. 检测 EKF 状态并选择初始控制模式
 * 2. 预热控制信号 (防止冷启动失败)
 * 3. 循环申请 OFFBOARD + ARM 直到成功
 * 4. 执行起飞 (推力从 0.68 → 0.58)
 * 5. 等待 EKF XY 收敛 (GPS 或 VIO 初始化)
 * 6. 根据 EKF 状态切换到 POSITION 或保持 ATTITUDE
 * 
 * 参数:
 * @param drone 飞控对象指针
 * @param takeoff_alt 目标起飞高度 (m)
 * @param takeoff_thrust 起飞推力 (0-1，通常 0.6-0.7)
 * @param takeoff_ramp_time 起飞推力平滑时间 (s)
 * @param takeoff_slow_zone 接近目标高度的减速区间 (m)
 * @param takeoff_thrust_min 起飞推力最低值 (接近目标高度时)
 * @param allow_xy_invalid 是否允许 XY 无效情况下继续 (true: ATTITUDE 模式)
 * @param hover_thrust 悬停推力 (通常 0.55-0.60)
 * 
 * 返回:
 * @return StartupResult 包含成功标志、起飞高度、EKF 有效性
 */
StartupResult startup_sequence(
    const std::shared_ptr<OffboardControl> &drone,
    double takeoff_alt,
    double takeoff_thrust,
    double takeoff_ramp_time,
    double takeoff_slow_zone,
    double takeoff_thrust_min,
    bool allow_xy_invalid,
    double hover_thrust) {

    StartupResult result;

    // ===== Phase 1: 检测 EKF 状态并选择模式 =====
    if (!drone->is_position_valid()) {
        std::cout << "⚠️  EKF XY position is INVALID. Using ATTITUDE mode to bypass health checks for arming..." << std::endl;
        drone->set_control_mode("attitude");
        drone->update_attitude_setpoint(0.0, 0.0, 0.0, 0.0);
    } else {
        std::cout << "📍 EKF Position is VALID. Using standard POSITION mode..." << std::endl;
        drone->set_control_mode("position");
        drone->update_position_setpoint(0.0, 0.0, 0.0, 0.0);
    }

    // 记录起飞基准高度（若 Z 有效）
    if (drone->is_z_valid()) {
        auto pos = drone->get_local_position();
        result.home_z = pos.z;
    }

    // ===== Phase 2: 预热控制信号 =====
    std::cout << "📡 Pre-warming control signals (1 second)..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // ===== Phase 3: 循环申请 OFFBOARD + ARM =====
    auto last_request = std::chrono::steady_clock::now();
    std::cout << "⏳ Starting OFFBOARD & ARM sequence..." << std::endl;

    while (rclcpp::ok() && !g_signal_triggered) {
        auto now = std::chrono::steady_clock::now();
        auto status = drone->get_vehicle_status();

        // 检查飞控当前状态
        bool is_offboard = (status.nav_state == 14);  // nav_state 14 = OFFBOARD
        bool is_armed = (status.arming_state == 2);   // arming_state 2 = ARMED

        // 定期输出调试信息 (1.5s)
        static auto last_print = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_print).count() >= 1500) {
            if (status.timestamp == 0) {
                std::cout << "⚠️  WARNING: No VehicleStatus message received yet! Check topic names." << std::endl;
            } else {
                std::cout << "DEBUG: [nav_state=" << static_cast<int>(status.nav_state)
                          << ", arming_state=" << static_cast<int>(status.arming_state)
                          << "] Offboard=" << (is_offboard ? "Y" : "N")
                          << ", Armed=" << (is_armed ? "Y" : "N") << std::endl;
            }
            last_print = now;
        }

        // ===== 检查 OFFBOARD + ARM 同时激活 =====
        if (is_offboard && is_armed) {
            std::cout << "✅ System Ready & Armed! EKF Valid: "
                      << (drone->is_position_valid() ? "YES" : "NO") << std::endl;

            // ===== Phase 4: 执行起飞 =====
            if (!drone->is_position_valid()) {
                std::cout << "🚀 [ADAPTIVE] EKF not ready. Performing smooth attitude liftoff..." << std::endl;
                // 平滑起飞推力曲线：先逐步增加推力，接近目标高度时提前降速
                int ramp_steps = std::max(10, static_cast<int>(takeoff_ramp_time / 0.1));
                for (int i = 0; i < ramp_steps; ++i) {
                    double t = static_cast<double>(i) / static_cast<double>(ramp_steps - 1);
                    double thrust = hover_thrust + (takeoff_thrust - hover_thrust) * t;

                    if (drone->is_z_valid()) {
                        auto pos = drone->get_local_position();
                        double target_z = result.home_z + takeoff_alt;
                        double z_error = target_z - pos.z;
                        if (z_error < takeoff_slow_zone) {
                            double slow_ratio = std::max(0.0, z_error / std::max(0.1, takeoff_slow_zone));
                            double desired = takeoff_thrust_min + (takeoff_thrust - takeoff_thrust_min) * slow_ratio;
                            thrust = std::min(thrust, desired);
                        }
                    }

                    thrust = std::max(takeoff_thrust_min, std::min(takeoff_thrust, thrust));
                    drone->update_attitude_setpoint(0.0, 0.0, 0.0, thrust);
                    std::this_thread::sleep_for(100ms);
                    if (drone->is_position_valid()) break;  // 若 EKF 突然有效则跳出
                }

                // ===== Phase 5: 等待 EKF 收敛 =====
                std::cout << "⏳ Waiting for EKF XY to stabilize while airborne..." << std::endl;
                auto liftoff_start = std::chrono::steady_clock::now();
                while (rclcpp::ok() && !drone->is_position_valid() &&
                       std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - liftoff_start).count() < 5) {
                    // 持续悬停 (hover_thrust) 并等待 EKF
                    drone->update_attitude_setpoint(0.0, 0.0, 0.0, hover_thrust);
                    std::this_thread::sleep_for(200ms);
                }
            }

            // ===== Phase 6: 根据 EKF 状态确定最终模式 =====
            if (drone->is_position_valid()) {
                std::cout << "📍 EKF is now VALID. Switching to POSITION mode for stabilizing..." << std::endl;
                drone->set_control_mode("position");

                auto pos = drone->get_local_position();
                result.home_z = pos.z;
                double target_z = result.home_z + takeoff_alt;
                drone->update_position_setpoint(0.0, 0.0, target_z, 0.0);
                std::this_thread::sleep_for(500ms);
                result.success = true;
                result.xy_valid = true;
            } else {
                // EKF 仍然无效
                if (allow_xy_invalid) {
                    std::cout << "⚠️ EKF XY still invalid. Continue in ATTITUDE hover mode." << std::endl;
                    drone->set_control_mode("attitude");
                    drone->update_attitude_setpoint(0.0, 0.0, 0.0, hover_thrust);
                    result.success = true;
                    result.xy_valid = false;
                } else {
                    // 不允许 GPS 缺失情况下起飞，执行安全降落
                    std::cout << "❌ EKF failed to normalize after liftoff. Safety Landing..." << std::endl;
                    drone->land();
                }
            }
            break;  // 启动完成，退出循环
        }

        // ===== 定时重新申请 OFFBOARD + ARM (3s) =====
        if (std::chrono::duration_cast<std::chrono::seconds>(now - last_request).count() >= 3) {
            last_request = now;

            if (!is_offboard) {
                std::cout << "🔄 Requesting OFFBOARD mode (Current nav_state=" << static_cast<int>(status.nav_state) << ")..." << std::endl;
                // PX4 命令: VEHICLE_CMD_DO_SET_MODE = 176, 参数: mode_main=1 (OFFBOARD), mode_sub=6
                drone->publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0);
            } else if (!is_armed) {
                std::cout << "🔓 Requesting ARM (Current arming_state=" << static_cast<int>(status.arming_state) << ")..." << std::endl;
                drone->arm();
            }
        }

        std::this_thread::sleep_for(100ms);
    }

    return result;
}

/**
 * @brief 主函数 - 初始化 ROS2、启动飞控和主控制循环
 * 
 * 执行步骤:
 * 1. 初始化 ROS2 (不使用自动 Ctrl+C 处理)
 * 2. 注册 POSIX SIGINT 处理器捕获 Ctrl+C
 * 3. 创建 ROS2 节点并声明所有参数
 * 4. 初始化飞控系统 (Vehicle + OffboardControl)
 * 5. 订阅视觉追踪输入
 * 6. 执行启动序列 (安全起飞)
 * 7. 运行主控制循环 (20Hz)
 * 8. 处理 Ctrl+C 中断和优雅关闭
 */
int main(int argc, char* argv[]) {
    // ===== 初始化 ROS2 =====
    // 禁用自动 Ctrl+C 处理，改用 POSIX 信号处理器
    if (!rclcpp::ok()) {
        auto options = rclcpp::InitOptions();
        options.shutdown_on_signal = false;
        rclcpp::init(argc, argv, options);
    }
    std::signal(SIGINT, signal_handler);

    // ===== 创建节点和参数 =====
    auto node = std::make_shared<rclcpp::Node>("offboard_tracker_manager");
    
    // 声明参数及其默认值
    node->declare_parameter("cmd_vel_topic", "/qr_tracker/cmd_vel_body");         ///< 速度命令话题
    node->declare_parameter("target_pos_topic", "/qr_tracker/relative_position");  ///< 相对位置目标话题
    node->declare_parameter("takeoff_alt", 1.2);                                    ///< 起飞高度 (m)
    node->declare_parameter("max_altitude", 2.0);                                  ///< 最大允许高度 (m) - 高度围栏
    node->declare_parameter("min_altitude", 0.3);                                  ///< 最小允许高度 (m) - 防撞地
    node->declare_parameter("z_hold_kp", 0.6);                                      ///< Z 轴保持比例系数
    node->declare_parameter("z_hold_max_vz", 0.25);                                ///< Z 轴最大速度 (m/s)
    node->declare_parameter("z_hold_deadband", 0.08);                              ///< Z 轴保持死区 (m)
    node->declare_parameter("z_cmd_max_vz", 0.15);                                 ///< 外部 z 速度限幅 (m/s)
    node->declare_parameter("command_timeout", 0.5);                               ///< 命令超时 (s)
    node->declare_parameter("takeoff_thrust", 0.68);                               ///< 起飞推力
    node->declare_parameter("takeoff_ramp_time", 2.5);                             ///< 起飞推力平滑时间 (s)
    node->declare_parameter("takeoff_slow_zone", 0.6);                             ///< 接近目标高度的减速区间 (m)
    node->declare_parameter("takeoff_thrust_min", 0.55);                           ///< 起飞推力最低值 (接近目标高度时)
    node->declare_parameter("allow_xy_invalid", true);                             ///< 允许 GPS 缺失起飞
    node->declare_parameter("hover_thrust", 0.58);                                 ///< 悬停推力

    // 读取参数值
    const auto cmd_vel_topic = node->get_parameter("cmd_vel_topic").as_string();
    const auto target_pos_topic = node->get_parameter("target_pos_topic").as_string();
    const auto takeoff_alt = node->get_parameter("takeoff_alt").as_double();
    const auto max_altitude = node->get_parameter("max_altitude").as_double();
    const auto min_altitude = node->get_parameter("min_altitude").as_double();
    const auto z_hold_kp = node->get_parameter("z_hold_kp").as_double();
    const auto z_hold_max_vz = node->get_parameter("z_hold_max_vz").as_double();
    const auto z_hold_deadband = node->get_parameter("z_hold_deadband").as_double();
    const auto z_cmd_max_vz = node->get_parameter("z_cmd_max_vz").as_double();
    const auto command_timeout = node->get_parameter("command_timeout").as_double();
    const auto takeoff_thrust = node->get_parameter("takeoff_thrust").as_double();
    const auto takeoff_ramp_time = node->get_parameter("takeoff_ramp_time").as_double();
    const auto takeoff_slow_zone = node->get_parameter("takeoff_slow_zone").as_double();
    const auto takeoff_thrust_min = node->get_parameter("takeoff_thrust_min").as_double();
    const auto allow_xy_invalid = node->get_parameter("allow_xy_invalid").as_bool();
    const auto hover_thrust = node->get_parameter("hover_thrust").as_double();

    std::cout << "════════════════════════════════════════════════════════" << std::endl;
    std::cout << "🚀 PX4 Offboard Tracker Manager" << std::endl;
    std::cout << "════════════════════════════════════════════════════════" << std::endl;

    // ===== 初始化飞控系统 =====
    auto vehicle = std::make_shared<Vehicle>();  // 创建飞控封装对象 (管理 ROS2 上下文生命周期)
    auto drone = vehicle->drone();                 // 获取 OffboardControl 实例

    // ===== 初始化命令状态容器 =====
    CmdState cmd_state;    ///< 保存速度命令状态
    TargetState target_state;  ///< 保存相对位置目标状态

    // ===== 订阅速度命令话题 =====
    auto cmd_sub = node->create_subscription<geometry_msgs::msg::Twist>(
        cmd_vel_topic, 10,
        [&cmd_state, &node](const geometry_msgs::msg::Twist::SharedPtr msg) {
            // 线程安全地保存最新命令
            std::lock_guard<std::mutex> guard(cmd_state.mutex);
            cmd_state.last_cmd = *msg;
            cmd_state.last_time = rclcpp::Clock(RCL_ROS_TIME).now();
            cmd_state.have_cmd = true;
            
            // 0.8 秒节流日志
            auto now_time = node->now();
            if ((now_time - cmd_state.last_log_time).seconds() >= 0.8) {
                RCLCPP_INFO(node->get_logger(), "✅ move command received: vx=%.3f vy=%.3f vz=%.3f", 
                    msg->linear.x, msg->linear.y, msg->linear.z);
                cmd_state.last_log_time = now_time;
            }
        });

    // ===== 订阅相对位置目标话题 =====
    auto target_sub = node->create_subscription<geometry_msgs::msg::PointStamped>(
        target_pos_topic, 10,
        [&target_state](const geometry_msgs::msg::PointStamped::SharedPtr msg) {
            // 线程安全地保存最新目标
            std::lock_guard<std::mutex> guard(target_state.mutex);
            target_state.last_target = *msg;
            target_state.last_time = rclcpp::Clock(RCL_ROS_TIME).now();
            target_state.have_target = true;
        });

    (void)cmd_sub;  // 避免未使用警告

    // ===== 执行启动序列 =====
    // 安全起飞到指定高度，并根据 EKF 状态自适应选择控制模式
    auto startup = startup_sequence(drone, takeoff_alt, takeoff_thrust, takeoff_ramp_time, takeoff_slow_zone, takeoff_thrust_min, allow_xy_invalid, hover_thrust);
    if (!startup.success || g_signal_triggered) {
        // 启动失败或被中断，优雅关闭
        vehicle->close();
        if (rclcpp::ok()) rclcpp::shutdown();
        return 0;
    }

    std::cout << "✅ Hovering, waiting for /qr_tracker/cmd_vel_body" << std::endl;

    // ===== 启动主控制循环 =====
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node);
    rclcpp::WallRate rate(20.0);  ///< 20Hz 主循环频率
    rclcpp::Time last_recover_request{0, 0, RCL_ROS_TIME};  ///< OFFBOARD 恢复请求计时

    while (rclcpp::ok() && !g_signal_triggered) {
        // 处理 ROS2 事件 (订阅回调等)
        exec.spin_some();

        // ===== 检查 OFFBOARD/ARM 状态并恢复 =====
        auto status = drone->get_vehicle_status();
        bool is_offboard = (status.nav_state == 14);  // OFFBOARD 模式
        bool is_armed = (status.arming_state == 2);   // ARMED 状态

        auto now_time = node->now();
        // 定时恢复丢失的 OFFBOARD/ARM (3 秒)
        if ((!is_offboard || !is_armed) && (now_time - last_recover_request).seconds() >= 3.0) {
            last_recover_request = now_time;
            if (!is_offboard) {
                RCLCPP_WARN(node->get_logger(), "⚠️ OFFBOARD lost, requesting OFFBOARD...");
                drone->publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0);
            }
            if (!is_armed) {
                RCLCPP_WARN(node->get_logger(), "⚠️ ARM lost, requesting ARM...");
                drone->arm();
            }
        }

        // ===== 读取最新的视觉追踪输入 (线程安全) =====
        geometry_msgs::msg::Twist cmd;
        bool use_cmd = false;
        rclcpp::Time last_time;
        
        geometry_msgs::msg::PointStamped target;
        bool use_target = false;
        rclcpp::Time target_time;

        // 原子地读取速度命令状态
        {
            std::lock_guard<std::mutex> guard(cmd_state.mutex);
            cmd = cmd_state.last_cmd;
            use_cmd = cmd_state.have_cmd;
            last_time = cmd_state.last_time;
        }

        // 原子地读取相对位置目标状态
        {
            std::lock_guard<std::mutex> guard(target_state.mutex);
            target = target_state.last_target;
            use_target = target_state.have_target;
            target_time = target_state.last_time;
        }

        // ===== 检查命令新鲜度 (超时判定) =====
        // 若命令超过 0.5 秒未更新，视为失效
        if (use_cmd) {
            double dt = (node->now() - last_time).seconds();
            if (dt > command_timeout) {
                use_cmd = false;  // 命令过期
            }
        }

        if (use_target) {
            double dt = (node->now() - target_time).seconds();
            if (dt > command_timeout) {
                use_target = false;  // 目标过期
            }
        }

        // ===== 控制律分支 =====
        if (startup.xy_valid) {
            // ===== 分支 1: EKF XY 有效 → 使用位置/速度控制 =====
            auto pos = drone->get_local_position();
            double target_z = startup.home_z + takeoff_alt;  // 目标 Z 轴 (维持起飞高度)

            if (use_target) {
                // 优先: 相对位置目标 → 转换为绝对位置指令
                // 相对位置 (Δx, Δy, Δz) + 当前位置 = 目标位置
                double tx = pos.x + target.point.x;  // 前向 X
                double ty = pos.y + target.point.y;  // 横向 Y
                // 注意：Z 轴使用固定目标高度，忽略视觉追踪的 Z 偏差
                // 这样可以防止无人机因视觉误差而无限上升
                double tz = target_z;  // 保持在起飞高度
                
                // ===== 高度围栏检查 =====
                double abs_altitude = tz - startup.home_z;  // 相对地面高度
                if (abs_altitude > max_altitude) {
                    tz = startup.home_z + max_altitude;
                    RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
                        "🚨 HEIGHT FENCE: Clamped altitude %.2f m → %.2f m (max)", abs_altitude, max_altitude);
                } else if (abs_altitude < min_altitude) {
                    tz = startup.home_z + min_altitude;
                    RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
                        "🚨 HEIGHT FENCE: Clamped altitude %.2f m → %.2f m (min)", abs_altitude, min_altitude);
                }
                
                drone->update_position_setpoint(tx, ty, tz, 0.0);
            } else if (use_cmd) {
                // 备选: 速度命令
                double vx = cmd.linear.x;
                double vy = cmd.linear.y;
                double vz = cmd.linear.z;

                // ===== 高度保持优先（避免持续上升） =====
                double z_error = target_z - pos.z;
                if (std::abs(z_error) > z_hold_deadband) {
                    vz = z_hold_kp * z_error;
                    vz = std::max(-z_hold_max_vz, std::min(z_hold_max_vz, vz));
                } else {
                    // 死区内仅限幅外部 z 速度
                    vz = std::max(-z_cmd_max_vz, std::min(z_cmd_max_vz, vz));
                }
                
                // ===== 高度围栏检查（速度模式）=====
                double current_altitude = pos.z - startup.home_z;
                if (current_altitude >= max_altitude && vz > 0) {
                    vz = 0.0;  // 到达最大高度时禁止向上
                    RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
                        "🚨 HEIGHT FENCE: Blocking upward velocity at %.2f m (max)", current_altitude);
                } else if (current_altitude <= min_altitude && vz < 0) {
                    vz = 0.0;  // 到达最小高度时禁止向下
                    RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
                        "🚨 HEIGHT FENCE: Blocking downward velocity at %.2f m (min)", current_altitude);
                }
                
                drone->update_velocity_setpoint(vx, vy, vz, 0.0);
            } else {
                // 无输入: 保持当前位置悬停（带围栏检查）
                double safe_z = pos.z;
                double current_altitude = pos.z - startup.home_z;
                if (current_altitude > max_altitude) {
                    safe_z = startup.home_z + max_altitude;
                } else if (current_altitude < min_altitude) {
                    safe_z = startup.home_z + min_altitude;
                }
                drone->update_position_setpoint(pos.x, pos.y, safe_z, 0.0);
            }
        } else {
            // ===== 分支 2: EKF XY 无效 =====
            // 若 Z 有效，则使用速度模式进行高度保持；否则退化为姿态悬停
            bool has_xy_cmd = std::abs(cmd.linear.x) > 1e-3 || std::abs(cmd.linear.y) > 1e-3;
            if (use_cmd && has_xy_cmd) {
                RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 2000,
                    "⚠️ XY invalid, ignoring XY velocity command.");
            }

            if (drone->is_z_valid()) {
                auto pos = drone->get_local_position();
                double target_z = startup.home_z + takeoff_alt;

                // 高度围栏钳位
                double abs_altitude = target_z - startup.home_z;
                if (abs_altitude > max_altitude) {
                    target_z = startup.home_z + max_altitude;
                } else if (abs_altitude < min_altitude) {
                    target_z = startup.home_z + min_altitude;
                }

                double z_error = target_z - pos.z;
                double vz = 0.0;
                if (std::abs(z_error) > z_hold_deadband) {
                    vz = z_hold_kp * z_error;
                    vz = std::max(-z_hold_max_vz, std::min(z_hold_max_vz, vz));
                }

                drone->update_velocity_setpoint(0.0, 0.0, vz, 0.0);
            } else {
                drone->update_attitude_setpoint(0.0, 0.0, 0.0, hover_thrust);
            }
        }

        rate.sleep();  // 维持 20Hz 循环
    }

    // ===== Ctrl+C 中断处理 - 安全降落 =====
    // 当用户按下 Ctrl+C 时，如果无人机仍在空中，发布降落命令
    if (g_signal_triggered) {
        auto status = drone->get_vehicle_status();
        if (status.arming_state == 2) {  // 仍为 ARMED 状态
            // 发送降落命令 (VEHICLE_CMD_NAV_LAND = 21)
            drone->publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
            std::cout << "🛑 Landing command issued..." << std::endl;
        }
    }

    // ===== 清理资源 =====
    vehicle->close();  // 关闭飞控并释放 ROS2 上下文
    if (rclcpp::ok()) {
        rclcpp::shutdown();  // 关闭 ROS2
    }
    
    std::cout << "✅ offboard_tracker_manager shut down gracefully" << std::endl;
    return 0;
}// Note: duplicate content removed.
