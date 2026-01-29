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
        bool has_pos = vehicle->drone()->is_position_received();
        std::string mode = has_pos ? "position" : "attitude"; 
        
        std::cout << "📍 Position status: " << (has_pos ? "VALID" : "INVALID (Indoor/No GPS)") << std::endl;
        std::cout << "📍 Auto-selecting [" << mode << "] mode..." << std::endl;
        
        vehicle->drone()->set_control_mode(mode);
        
        if (mode == "attitude") {
            // 姿态模式初始化：平飞，零油门
            vehicle->drone()->update_attitude_setpoint(0.0, 0.0, 0.0, 0.0);
        } else {
            // 定点模式初始化：当前位置悬停
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
                    vehicle->drone()->publish_vehicle_command(
                        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0);
                } else if (!is_armed) {
                    std::cout << "🔓 Requesting ARM..." << std::endl;
                    vehicle->drone()->arm(); // 使用库提供的 arm() 方法
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        // 3. 执行飞行任务 (根据模式自动判断)
        if (rclcpp::ok() && mode == "position") {
            std::cout << "🚀 Mission Start [POSITION MODE]" << std::endl;
            std::cout << "🛸 Taking off to 2.0m..." << std::endl;
            if (vehicle->drone()->takeoff(2.0, 15.0)) { // 缩短超时时间
                std::cout << "⏳ Hovering for 5 seconds..." << std::endl;
                
                auto hover_start = std::chrono::steady_clock::now();
                while (rclcpp::ok() && std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - hover_start).count() < 5) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
                
                if (rclcpp::ok()) {
                    std::cout << "✅ Flying to target (5.0, 0.0, 2.0)..." << std::endl;
                    vehicle->drone()->fly_to_trajectory_setpoint(5.0, 0.0, 2.0, 0.0, 10.0);
                }
                
                if (rclcpp::ok()) {
                    std::cout << "🛬 Landing..." << std::endl;
                    vehicle->drone()->land();
                }
            }
        } else if (rclcpp::ok()) {
            std::cout << "🚀 Mission Start [ATTITUDE MODE]" << std::endl;
            std::cout << "⚠️ Running indoor attitude sequence..." << std::endl;
            
            // 态模式下的安全测试
            for (int i = 0; rclcpp::ok() && i < 20; i++) {
                double thrust = 0.1 + (i * 0.015);
                vehicle->drone()->update_attitude_setpoint(0.0, 0.0, 0.0, thrust);
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }

            if (rclcpp::ok()) {
                auto ramp_end = std::chrono::steady_clock::now();
                while(rclcpp::ok() && std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - ramp_end).count() < 2) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
                vehicle->drone()->update_attitude_setpoint(0.0, 0.0, 0.0, 0.1); 
            }
            
            std::cout << "🔒 Disarming..." << std::endl;
            vehicle->drone()->disarm();
        }
        
        if (!rclcpp::ok()) {
            std::cout << "🛑 Mission interrupted by user (Ctrl+C)." << std::endl;
        }
        
    } catch (const std::exception& e) {
        std::cerr << "❌ Exception caught: " << e.what() << std::endl;
    }

    std::cout << "🛑 Shutting down..." << std::endl;
    vehicle->close();
    std::cout << "✅ Application terminated successfully." << std::endl;
    return 0;
}
