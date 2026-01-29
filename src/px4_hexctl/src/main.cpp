#include <rclcpp/rclcpp.hpp>
#include "px4_hexctl/offboard_control.hpp"
#include "px4_hexctl/vehicle.hpp"
#include <chrono>
#include <thread>


int main(int argc, char* argv[]) {
    std::cout << "════════════════════════════════════════════════════════" << std::endl;
    std::cout << "🚀 PX4 Offboard Control - Position Mode State Machine" << std::endl;
    std::cout << "════════════════════════════════════════════════════════" << std::endl;

    // 创建 Vehicle 实例（初始化 ROS2 并启动心跳）
    auto vehicle = std::make_shared<Vehicle>();

    try {
        // 1. 设置控制模式
        // 在室内无GPS环境下，必须使用 attitude 模式。
        // target_ 格式: [roll, pitch, yaw, thrust] (前三个弧度，最后一个 0.0~1.0)
        std::string mode = "attitude"; 
        std::cout << "📍 Setting up [" << mode << "] control mode..." << std::endl;
        vehicle->drone()->set_control_mode(mode);
        // 初始化一个安全的姿态：平飞，不给油门（直到解锁后才给）
        vehicle->drone()->update_attitude_setpoint(0.0, 0.0, 0.0, 0.0);
        
        // 2. 状态机：循环检查并请求 OFFBOARD 模式和解锁
        auto start_time = std::chrono::steady_clock::now();
        auto last_request = std::chrono::steady_clock::now();
        std::cout << "⏳ Waiting for Offboard and Arming (State Machine)..." << std::endl;

        while (rclcpp::ok()) {
            auto now = std::chrono::steady_clock::now();
            auto status = vehicle->drone()->get_vehicle_status();
            
            // PX4 常量: NAVIGATION_STATE_OFFBOARD = 14, ARMING_STATE_ARMED = 2
            bool is_offboard = (status.nav_state == 14);
            bool is_armed = (status.arming_state == 2);

            if (is_offboard && is_armed) {
                std::cout << "✅ System Ready: Armed and in Offboard mode." << std::endl;
                break;
            }

            // 检查位置数据状态
            bool has_pos = vehicle->drone()->is_position_received();
            
            // 每 2 秒发送一次请求
            if (std::chrono::duration_cast<std::chrono::seconds>(now - last_request).count() >= 2) {
                last_request = now;
                
                if (!is_offboard) {
                    std::cout << "🔄 Requesting OFFBOARD... " 
                              << (mode == "position" ? (has_pos ? "(Pos-Ready)" : "(WAITING-POS)") : "(Attitude-Ready)") 
                              << std::endl;
                    // 发送切换模式指令
                    vehicle->drone()->publish_vehicle_command(
                        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0);
                } else if (!is_armed) {
                    std::cout << "🔓 Requesting ARM..." << std::endl;
                    // 在解锁前，心跳线程已经在持续发送 setpoint (在 Vehicle 构造中已启动)
                    vehicle->drone()->publish_vehicle_command(
                        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
                }
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        // 3. 执行飞行任务
        std::cout << "🚀 Proceeding with mission..." << std::endl;

        std::cout << "🛸 Taking off to 2.0m..." << std::endl;
        if (!vehicle->drone()->takeoff(2.0)) {
            std::cerr << "❌ Takeoff failed!" << std::endl;
        }
        
        std::cout << "⏳ Hovering for 5 seconds..." << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(5));

        std::cout << "✅ Flying to the target 1!" << std::endl;
        vehicle->drone()->fly_to_trajectory_setpoint(5.0, 0.0, 2.0, 0.0, 10.0);

        std::cout << "🛬 Landing..." << std::endl;
        if (!vehicle->drone()->land()) {
            std::cerr << "❌ Land command failed or timed out!" << std::endl;
        }
        
        std::cout << "🔒 Disarming..." << std::endl;
        vehicle->drone()->disarm();
        std::cout << "✅ Disarm command sent." << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "❌ Exception caught: " << e.what() << std::endl;
    }

    std::cout << "🛑 Shutting down..." << std::endl;
    vehicle->close();
    std::cout << "✅ Application terminated successfully." << std::endl;
    return 0;
}
