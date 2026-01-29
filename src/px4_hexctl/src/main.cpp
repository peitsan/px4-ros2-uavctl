#include <rclcpp/rclcpp.hpp>
#include "px4_hexctl/offboard_control.hpp"
#include "px4_hexctl/vehicle.hpp"
#include <chrono>
#include <thread>
#include <iostream>
#include <csignal>
#include <atomic>

using namespace std::chrono_literals;

// 全局变量，用于捕捉 Ctrl+C
std::atomic<bool> g_signal_triggered(false);

void signal_handler(int signum) {
    (void)signum;
    g_signal_triggered = true;
    // 不要在这里调用 rclcpp::shutdown()，否则会导致后续指令无法发出
}

int main(int argc, char* argv[]) {
    // 注册自定义信号处理
    std::signal(SIGINT, signal_handler);

    std::cout << "════════════════════════════════════════════════════════" << std::endl;
    std::cout << "🚀 PX4 Offboard Control - Professional Armed Sequence" << std::endl;
    std::cout << "════════════════════════════════════════════════════════" << std::endl;

    // 1. 初始化 Vehicle
    auto vehicle = std::make_shared<Vehicle>();
    auto drone = vehicle->drone();

    try {
        // ...existing code...
        std::cout << "⏳ Starting OFFBOARD & ARM sequence..." << std::endl;

        while (rclcpp::ok() && !g_signal_triggered) {
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
        if (rclcpp::ok() && !g_signal_triggered) {
            std::cout << "🚀 Mission Start: Taking off to 1.2m..." << std::endl;
            if (drone->takeoff(1.2, 10.0)) {
                std::cout << "✅ Takeoff successful, hovering 5s." << std::endl;
                
                auto hover_start = std::chrono::steady_clock::now();
                while (rclcpp::ok() && !g_signal_triggered && 
                       std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - hover_start).count() < 5) {
                    std::this_thread::sleep_for(100ms);
                }
                
                if (!g_signal_triggered) {
                    std::cout << "🛬 Mission end, landing..." << std::endl;
                    drone->land();
                }
            } else {
                std::cout << "❌ Takeoff failed or timed out." << std::endl;
            }
        }

    } catch (const std::exception& e) {
        std::cerr << "❌ Exception caught: " << e.what() << std::endl;
    }

    // 6. 处理退出逻辑 (SIGINT)
    if (g_signal_triggered) {
        std::cout << "\n🛑 [EXIT] Signal caught! Performing emergency landing and switching to MANUAL..." << std::endl;
        auto status = drone->get_vehicle_status();
        if (status.arming_state == 2) { // 如果已解锁
            // 发送降落命令
            std::cout << "🛬 Sending emergency LAND command..." << std::endl;
            drone->publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
            
            // 切换回 MANUAL 模式 (Main mode 1)
            std::cout << "🕹️ Switching back to MANUAL mode..." << std::endl;
            drone->publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0);
            
            std::this_thread::sleep_for(500ms); // 给指令一点发布时间
        }
    }

    std::cout << "🛑 Shutting down Vehicle node..." << std::endl;
    vehicle->close();
    
    // 最后再正式关闭 rclcpp
    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }
    
    std::cout << "👋 Node exited safely." << std::endl;
    return 0;
}
