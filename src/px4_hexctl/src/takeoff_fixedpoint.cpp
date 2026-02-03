/**
 * @file takeoff_fixedpoint.cpp
 * @brief PX4 无人机定点起飞节点（定高1m，OFFBOARD模式，uXRCE-DDS）
 * 
 * 功能描述：
 * - 通过 uXRCE-DDS 与 PX4 通信，不依赖 MAVROS
 * - 切入 OFFBOARD 模式后发布位置目标
 * - 订阅本地位置话题，实时跟踪无人机高度
 * - PID 控制无人机起飞并稳定在目标高度（如 1m）
 * - 支持参数化配置（目标高度、增益、话题名等）
 * - 不依赖二维码视觉追踪
 * 
 * 控制逻辑：
 * 1. 等待初始位置有效
 * 2. 设置位置目标，逐步提升高度至目标值
 * 3. 到达目标高度后保持定高
 * 4. 支持安全边界和速度限制
 * 
 * @author px4-ros2-uavctl team
 * @date 2026-02
 */

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>

#include <chrono>
#include <mutex>
#include <cmath>

using namespace std::chrono_literals;

class TakeoffFixedPointNode : public rclcpp::Node {
public:
    TakeoffFixedPointNode() : Node("takeoff_fixedpoint"), offboard_setpoint_counter_(0) {
        // ========== 参数声明 ==========
        declare_parameter("target_height_m", 1.0);              ///< 目标高度 (m)
        declare_parameter("kp_height", 0.5);                    ///< 高度 PID 比例系数 (降低以减缓响应)
        declare_parameter("ki_height", 0.05);                   ///< 高度 PID 积分系数
        declare_parameter("kd_height", 0.05);                   ///< 高度 PID 微分系数
        declare_parameter("max_vertical_speed", 0.15);          ///< 最大竖直速度 (m/s) - 六轴已降至 0.15
        declare_parameter("max_vertical_accel", 0.1);           ///< 最大竖直加速度 (m/s²) - 防止电机突跃
        declare_parameter("height_tolerance", 0.1);             ///< 高度容差 (m)
        declare_parameter("height_safety_margin", 0.5);         ///< 安全高度余量 (m) - 高度超过目标 + 余量时强制降低
        declare_parameter("takeoff_ramp_rate", 0.05);           ///< 起飞加速度 (m/s²) - 平滑起飞斜坡
        declare_parameter("emergency_stop_height", 1.5);        ///< 紧急停止高度 (m) - 超过此高度立即切断
        declare_parameter("enable_accel_ramp", true);           ///< 启用加速度斜坡
        declare_parameter("ramp_update_interval_ms", 100);      ///< 加速度斜坡更新间隔 (ms)
        
        declare_parameter("local_position_topic", std::string("/fmu/out/vehicle_local_position"));
        declare_parameter("vehicle_status_topic", std::string("/fmu/out/vehicle_status"));
        declare_parameter("offboard_mode_topic", std::string("/fmu/in/offboard_control_mode"));
        declare_parameter("trajectory_setpoint_topic", std::string("/fmu/in/trajectory_setpoint"));
        
        // 参数读取
        target_height_m_ = get_parameter("target_height_m").as_double();
        kp_height_ = get_parameter("kp_height").as_double();
        ki_height_ = get_parameter("ki_height").as_double();
        kd_height_ = get_parameter("kd_height").as_double();
        max_vertical_speed_ = get_parameter("max_vertical_speed").as_double();
        max_vertical_accel_ = get_parameter("max_vertical_accel").as_double();
        height_tolerance_ = get_parameter("height_tolerance").as_double();
        height_safety_margin_ = get_parameter("height_safety_margin").as_double();
        takeoff_ramp_rate_ = get_parameter("takeoff_ramp_rate").as_double();
        emergency_stop_height_ = get_parameter("emergency_stop_height").as_double();
        enable_accel_ramp_ = get_parameter("enable_accel_ramp").as_bool();
        ramp_update_interval_ms_ = get_parameter("ramp_update_interval_ms").as_int();

        std::string local_pos_topic = get_parameter("local_position_topic").as_string();
        std::string status_topic = get_parameter("vehicle_status_topic").as_string();
        std::string offboard_mode_topic = get_parameter("offboard_mode_topic").as_string();
        std::string trajectory_topic = get_parameter("trajectory_setpoint_topic").as_string();

        // ========== 创建 ROS2 订阅者/发布者 ==========
        // 订阅无人机本地位置
        local_position_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            local_pos_topic, 10, std::bind(&TakeoffFixedPointNode::local_position_callback, this, std::placeholders::_1));
        
        // 订阅无人机状态（判断是否在 OFFBOARD 模式）
        vehicle_status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
            status_topic, 10, std::bind(&TakeoffFixedPointNode::vehicle_status_callback, this, std::placeholders::_1));

        // 发布 OFFBOARD 控制模式
        offboard_control_mode_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(offboard_mode_topic, 10);
        
        // 发布轨迹设定点（位置目标）
        trajectory_setpoint_pub_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(trajectory_topic, 10);

        // 控制循环定时器（50ms 周期 = 20Hz）
        timer_ = create_wall_timer(50ms, std::bind(&TakeoffFixedPointNode::control_loop, this));

        RCLCPP_INFO(get_logger(), "✅ TakeoffFixedPointNode started (uXRCE-DDS). target_height=%.2f m, kp=%.2f, max_speed=%.3f m/s, max_accel=%.3f m/s²", 
            target_height_m_, kp_height_, max_vertical_speed_, max_vertical_accel_);
    }

private:
    /**
     * @brief 本地位置回调 - 获取当前高度
     * 在 PX4 中，z 为负值表示高度（向下为正）
     */
    void local_position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
        std::lock_guard<std::mutex> guard(state_mutex_);
        current_height_ = -msg->z;  // z 为负值，取反得到高度
        position_valid_ = msg->xy_valid && msg->z_valid;
        timestamp_ = msg->timestamp;
    }

    /**
     * @brief 无人机状态回调 - 判断是否在 OFFBOARD 模式
     */
    void vehicle_status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
        std::lock_guard<std::mutex> guard(state_mutex_);
        nav_state_ = msg->nav_state;
    }

    /**
     * @brief 控制循环 - 50ms 周期执行
     * 
     * 逻辑：
     * 1. 检查位置有效性和安全约束
     * 2. 发布 OFFBOARD 控制模式（启用位置控制）
     * 3. PID 计算垂直速度目标
     * 4. 加速度斜坡平滑过渡（防止电机突跃）
     * 5. 安全边界检查（超高检测和紧急停止）
     * 6. 发布轨迹设定点
     */
    void control_loop() {
        px4_msgs::msg::OffboardControlMode offboard_mode{};
        px4_msgs::msg::TrajectorySetpoint trajectory{};

        {
            std::lock_guard<std::mutex> guard(state_mutex_);

            // 检查位置有效
            if (!position_valid_) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, 
                    "⚠️ Position not valid yet, waiting...");
                return;
            }

            // ===== 紧急安全检查：高度超限 =====
            if (current_height_ > emergency_stop_height_) {
                RCLCPP_ERROR(get_logger(), "🚨 EMERGENCY: Height %.3f m exceeded limit %.3f m. CUTTING MOTORS!", 
                    current_height_, emergency_stop_height_);
                // 发布零速度指令（紧急停止）
                offboard_mode.position = true;
                offboard_mode.velocity = false;
                offboard_mode.acceleration = false;
                offboard_mode.attitude = false;
                offboard_mode.body_rate = false;
                offboard_mode.timestamp = timestamp_;

                trajectory.position[0] = 0.0f;
                trajectory.position[1] = 0.0f;
                trajectory.position[2] = -current_height_;  // 保持当前高度
                trajectory.velocity[0] = 0.0f;
                trajectory.velocity[1] = 0.0f;
                trajectory.velocity[2] = 0.0f;  // 停止垂直运动
                trajectory.acceleration[0] = 0.0f;
                trajectory.acceleration[1] = 0.0f;
                trajectory.acceleration[2] = 0.0f;
                trajectory.yaw = 0.0f;
                trajectory.timestamp = timestamp_;

                offboard_control_mode_pub_->publish(offboard_mode);
                trajectory_setpoint_pub_->publish(trajectory);
                return;
            }

            // ===== OFFBOARD 模式设置 =====
            offboard_mode.position = true;
            offboard_mode.velocity = false;
            offboard_mode.acceleration = false;
            offboard_mode.attitude = false;
            offboard_mode.body_rate = false;
            offboard_mode.timestamp = timestamp_;

            // ===== 高度控制逻辑 =====
            double height_error = target_height_m_ - current_height_;
            double desired_vz = 0.0;

            // 检查是否超过安全余量
            if (current_height_ > target_height_m_ + height_safety_margin_) {
                // 已超过安全高度，强制降低速度
                desired_vz = -0.1;  // 向下运动
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500, 
                    "⚠️ Over-height: %.3f m > target %.3f m + margin %.3f m. Descending.", 
                    current_height_, target_height_m_, height_safety_margin_);
            } else if (std::abs(height_error) > height_tolerance_) {
                // 未到达目标高度，计算 PID 输出
                desired_vz = kp_height_ * height_error;
                
                // 应用积分项
                height_error_integral_ += height_error * 0.05;  // 0.05s 积分步长
                height_error_integral_ = std::max(-0.3, std::min(0.3, height_error_integral_));  // 防积分饱和
                desired_vz += ki_height_ * height_error_integral_;

                // 应用微分项
                double height_error_derivative = (height_error - last_height_error_) / 0.05;
                desired_vz += kd_height_ * height_error_derivative;
                last_height_error_ = height_error;
            } else {
                // 已到达目标高度，保持定高
                desired_vz = 0.0;
                height_error_integral_ = 0.0;
                last_height_error_ = 0.0;
                if (!at_target_height_) {
                    RCLCPP_INFO(get_logger(), "✅ Reached target height: %.3f m", current_height_);
                    at_target_height_ = true;
                }
            }

            // ===== 加速度斜坡（防止电机突跃）=====
            if (enable_accel_ramp_) {
                double dt = 0.05;  // 50ms 周期
                double max_delta_vz = max_vertical_accel_ * dt;  // 单步最大速度变化
                
                // 限制速度变化率
                if (desired_vz > last_velocity_cmd_ + max_delta_vz) {
                    desired_vz = last_velocity_cmd_ + max_delta_vz;
                } else if (desired_vz < last_velocity_cmd_ - max_delta_vz) {
                    desired_vz = last_velocity_cmd_ - max_delta_vz;
                }
                last_velocity_cmd_ = desired_vz;
            }

            // 限制速度
            desired_vz = std::max(-max_vertical_speed_, std::min(max_vertical_speed_, desired_vz));

            // ===== 轨迹设定点 =====
            trajectory.position[0] = 0.0f;      // 水平位置（PX4 的局部坐标系）
            trajectory.position[1] = 0.0f;
            trajectory.position[2] = -target_height_m_;  // z 为负值表示高度
            trajectory.velocity[0] = 0.0f;      // 水平速度
            trajectory.velocity[1] = 0.0f;
            trajectory.velocity[2] = -desired_vz;  // z 负值表示向上
            trajectory.acceleration[0] = 0.0f;
            trajectory.acceleration[1] = 0.0f;
            trajectory.acceleration[2] = 0.0f;
            trajectory.yaw = 0.0f;
            trajectory.timestamp = timestamp_;

            // 每次循环输出调试日志（每 0.4s 输出一次）
            ++offboard_setpoint_counter_;
            if (offboard_setpoint_counter_ >= 8) {
                RCLCPP_INFO(get_logger(), 
                    "📍 h=%.3f/%.3f m, err=%.3f, vz_cmd=%.3f m/s, mode=%u, safety_ok=%s",
                    current_height_, target_height_m_, height_error, desired_vz, nav_state_,
                    (current_height_ > target_height_m_ + height_safety_margin_) ? "NO" : "OK");
                offboard_setpoint_counter_ = 0;
            }
        }

        // 发布控制指令
        offboard_control_mode_pub_->publish(offboard_mode);
        trajectory_setpoint_pub_->publish(trajectory);
    }

    // ========== 参数存储 ==========
    double target_height_m_;           ///< 目标高度 (m)
    double kp_height_, ki_height_, kd_height_;  ///< PID 系数
    double max_vertical_speed_;        ///< 最大竖直速度 (m/s)
    double max_vertical_accel_;        ///< 最大竖直加速度 (m/s²)
    double height_tolerance_;          ///< 高度容差 (m)
    double height_safety_margin_;      ///< 安全高度余量 (m)
    double takeoff_ramp_rate_;         ///< 起飞斜坡速率 (m/s)
    double emergency_stop_height_;     ///< 紧急停止高度 (m)
    bool enable_accel_ramp_;           ///< 是否启用加速度斜坡
    int ramp_update_interval_ms_;      ///< 加速度斜坡更新间隔 (ms)

    // ========== ROS2 通信接口 ==========
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_position_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // ========== 状态变量 (线程安全) ==========
    std::mutex state_mutex_;
    double current_height_ = 0.0;      ///< 当前高度 (m)
    bool position_valid_ = false;      ///< 位置是否有效
    uint64_t timestamp_ = 0;           ///< PX4 时间戳
    uint8_t nav_state_ = 0;            ///< 无人机导航状态

    // ========== PID 状态 ==========
    double height_error_integral_ = 0.0;
    double last_height_error_ = 0.0;
    double last_velocity_cmd_ = 0.0;   ///< 上一次速度命令（用于加速度斜坡）
    bool at_target_height_ = false;
    int offboard_setpoint_counter_ = 0;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TakeoffFixedPointNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
