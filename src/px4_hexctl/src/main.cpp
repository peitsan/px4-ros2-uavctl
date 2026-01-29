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
        // 1. 自动根据定位状态选择模式
        // 注意：is_position_valid() 检查 EKF 是否真正对位置有信心
        bool pos_ok = vehicle->drone()->is_position_valid();
        std::string mode = pos_ok ? "position" : "attitude"; 
        
        std::cout << "📍 Position status: " << (pos_ok ? "VALID (Ready for Position mode)" : "INVALID (Using Attitude mode)") << std::endl;
        if (!pos_ok && vehicle->drone()->get_local_position().timestamp > 0) {
            std::cout << "⚠️ Warning: Received position data but EKF flags it as UNRELIABLE (xy_valid=0)." << std::endl;
        }
        
        std::cout << "📍 Final Mission Mode: [" << mode << "]" << std::endl;
        
        vehicle->drone()->set_control_mode(mode);
        
        if (mode == "attitude") {
            vehicle->drone()->update_attitude_setpoint(0.0, 0.0, 0.0, 0.0);
        } else {
            auto pos = vehicle->drone()->get_local_position();
            vehicle->drone()->update_position_setpoint(pos.x, pos.y, pos.z, pos.heading);
        }
        
        // 2. 状态机：循环检查并请求 OFFBOARD 模式和解锁
        auto last_request = std::chrono::steady_clock::now();
        std::cout << "⏳ Waiting for Offboard and Arming..." << std::endl;

        while (rclcpp::ok()) {
            auto now = std::chrono::steady_clock::now();
            auto status = vehicle->drone()->get_vehicle_status();
            
            bool is_offboard = (status.nav_state == 14);
            bool is_armed = (status.arming_state == 2);

            if (is_offboard && is_armed) {
                std::cout << "✅ System Ready: Armed and in Offboard mode." << std::endl;
                break;
            }

            // 每 2 秒发送一次请求
            if (std::chrono::duration_cast<std::chrono::seconds>(now - last_request).count() >= 2) {
                last_request = now;
                
                if (!is_offboard) {
                    std::cout << "🔄 Requesting OFFBOARD (" << mode << " mode)..." << std::endl;
                    // PX4 v1.14+ 推荐的模式切换参数
                    vehicle->drone()->publish_vehicle_command(
                        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0);
                } else if (!is_armed) {
                    std::cout << "🔓 Requesting ARM..." << std::endl;
                    vehicle->drone()->arm(); 
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        // 3. 执行飞行任务
        if (rclcpp::ok()) {
            if (mode == "position") {
                std::cout << "🚀 Mission Start [POSITION MODE]" << std::endl;
                std::cout << "🛸 Taking off to 2.0m..." << std::endl;
                if (vehicle->drone()->takeoff(2.0, 15.0)) {
                    std::cout << "⏳ Hovering for 5 seconds..." << std::endl;
                    auto hover_start = std::chrono::steady_clock::now();
                    while (rclcpp::ok() && std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - hover_start).count() < 5) {
                        std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    }
                    
                    if (rclcpp::ok()) {
                        std::cout << "✅ Flying to target..." << std::endl;
                        vehicle->drone()->fly_to_trajectory_setpoint(5.0, 0.0, 2.0, 0.0, 10.0);
                    }
                }
            } else {
                std::cout << "🚀 Mission Start [ATTITUDE MODE]" << std::endl;
                std::cout << "📶 Ramping up thrust..." << std::endl;
                for (int i = 0; rclcpp::ok() && i < 20; i++) {
                    double thrust = 0.1 + (i * 0.015);
                    vehicle->drone()->update_attitude_setpoint(0.0, 0.0, 0.0, thrust);
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
                if (rclcpp::ok()) {
                    auto start = std::chrono::steady_clock::now();
                    while (rclcpp::ok() && std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start).count() < 3) {
                        std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    }
                }
            }
        }

        // 4. 程序结束或 Ctrl+C 后的清理
        if (!rclcpp::ok()) {
            std::cout << "\n🛑 Interrupted! Performing emergency landing..." << std::endl;
        } else {
            std::cout << "🛬 Mission complete. Landing..." << std::endl;
        }

        if (mode == "position") {
            vehicle->drone()->land();
        } else {
            vehicle->drone()->update_attitude_setpoint(0.0, 0.0, 0.0, 0.1); // 降油门
            std::this_thread::sleep_for(std::chrono::seconds(1));
            vehicle->drone()->disarm();
        }
        
    } catch (const std::exception& e) {
        std::cerr << "❌ Exception caught: " << e.what() << std::endl;
    }

    std::cout << "🛑 Shutting down..." << std::endl;
    vehicle->close();
    std::cout << "✅ Application terminated successfully." << std::endl;
    return 0;
}
