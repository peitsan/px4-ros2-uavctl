#include <rclcpp/rclcpp.hpp>
#include "px4_hexctl/offboard_control.hpp"
#include "px4_hexctl/vehicle.hpp"
#include <chrono>
#include <thread>
#include <iostream>

using namespace std::chrono_literals;

int main(int argc, char* argv[]) {
    std::cout << "════════════════════════════════════════════════════════" << std::endl;
    std::cout << "🚀 PX4 Offboard Control - Professional Armed Sequence" << std::endl;
    std::cout << "════════════════════════════════════════════════════════" << std::endl;

    // 1. 初始化 Vehicle (这会启动 ROS2, 节点, 以及 20Hz 的控制心跳线程)
    auto vehicle = std::make_shared<Vehicle>();
    auto drone = vehicle->drone();

    try {
        // 2. 模式检测与自适应初始化
        // 如果 EKF XY 无效 (VALID: XY:0)，直接使用 Position 模式解锁会被拒绝。
        // 我们根据传感器状态自动选择解锁时使用的控制流。
        if (!drone->is_position_valid()) {
            std::cout << "⚠️  EKF XY position is INVALID. Using ATTITUDE mode to bypass health checks for arming..." << std::endl;
            drone->set_control_mode("attitude");
            drone->update_attitude_setpoint(0.0, 0.0, 0.0, 0.0); // 水平，零推力
        } else {
            std::cout << "📍 EKF Position is VALID. Using standard POSITION mode..." << std::endl;
            drone->set_control_mode("position");
            drone->update_position_setpoint(0.0, 0.0, 0.0, 0.0);
        }

        // 3. 预热阶段 (Pre-warm)
        // 在切换 Offboard 模式前，后台心跳已经在持续发送 Setpoint 数据
        std::cout << "📡 Pre-warming control signals (2 seconds)..." << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(2));

        // 4. 执行模式切换和解锁的状态机
        auto last_request = std::chrono::steady_clock::now();
        std::cout << "⏳ Starting OFFBOARD & ARM sequence..." << std::endl;

        while (rclcpp::ok()) {
            auto now = std::chrono::steady_clock::now();
            auto status = drone->get_vehicle_status();
            
            bool is_offboard = (status.nav_state == 14); // NAVIGATION_STATE_OFFBOARD
            bool is_armed = (status.arming_state == 2);   // ARMING_STATE_ARMED

            // 每 1.5 秒打印一次状态，帮助诊断
            static auto last_print = std::chrono::steady_clock::now();
            if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_print).count() >= 1500) {
                if (status.timestamp == 0) {
                    std::cout << "⚠️  WARNING: No VehicleStatus message received yet! Check topic names." << std::endl;
                } else {
                    std::cout << "DEBUG: [nav_state=" << (int)status.nav_state 
                              << ", arming_state=" << (int)status.arming_state 
                              << "] Offboard=" << (is_offboard ? "Y" : "N") 
                              << ", Armed=" << (is_armed ? "Y" : "N") << std::endl;
                }
                last_print = now;
            }

            if (is_offboard && is_armed) {
                std::cout << "✅ System Ready & Armed!" << std::endl;
                
                // 如果之前为了解锁使用了姿态模式，现在尝试切换回位置模式进行起飞
                if (drone->is_position_valid()) {
                    std::cout << "🔄 Switching back to POSITION mode for takeoff..." << std::endl;
                    drone->set_control_mode("position");
                    drone->update_position_setpoint(0.0, 0.0, 0.0, 0.0);
                    std::this_thread::sleep_for(500ms); 
                }
                break;
            }

            // 每 3 秒重试一次请求
            if (std::chrono::duration_cast<std::chrono::seconds>(now - last_request).count() >= 3) {
                last_request = now;

                if (!is_offboard) {
                    std::cout << "🔄 Requesting OFFBOARD mode (Current nav_state=" << (int)status.nav_state << ")..." << std::endl;
                    drone->publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0);
                } else if (!is_armed) {
                    std::cout << "🔓 Requesting ARM (Current arming_state=" << (int)status.arming_state << ")..." << std::endl;
                    drone->arm();
                }
            }
            
            std::this_thread::sleep_for(100ms);
        }

        // 5. 执行起飞任务
        if (rclcpp::ok()) {
            std::cout << "🚀 Mission Start: Taking off to 1.2m..." << std::endl;
            if (drone->takeoff(1.2, 10.0)) {
                std::cout << "✅ Takeoff successful, hovering 5s." << std::endl;
                
                auto hover_start = std::chrono::steady_clock::now();
                while (rclcpp::ok() && std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - hover_start).count() < 5) {
                    std::this_thread::sleep_for(100ms);
                }
                
                std::cout << "🛬 Mission end, landing..." << std::endl;
                drone->land();
            } else {
                std::cout << "❌ Takeoff failed or timed out." << std::endl;
            }
        }

    } catch (const std::exception& e) {
        std::cerr << "❌ Exception caught: " << e.what() << std::endl;
    }

    std::cout << "🛑 Shutting down..." << std::endl;
    vehicle->close();
    return 0;
}
