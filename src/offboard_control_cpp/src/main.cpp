#include <rclcpp/rclcpp.hpp>
#include "offboard_control_cpp/offboard_control.hpp"
#include "offboard_control_cpp/vehicle.hpp"
#include <chrono>
#include <thread>


int main(int argc, char* argv[]) {
    std::cout << "════════════════════════════════════════════════════════" << std::endl;
    std::cout << "🚀 PX4 Offboard Control - Starting Application" << std::endl;
    std::cout << "════════════════════════════════════════════════════════" << std::endl;

    // 创建 Vehicle 实例（会自动初始化 ROS2 和启动心跳线程）
    std::cout << "📍 Initializing Vehicle..." << std::endl;
    auto vehicle = std::make_shared<Vehicle>();

    try {
        // 等待足够的时间让心跳线程稳定工作和订阅器连接
        std::cout << "⏳ Waiting for system initialization (10 seconds)..." << std::endl;
        std::cout << "   心跳线程应该已启动，正在发送 offboard control signals..." << std::endl;
        for (int i = 0; i < 10; i++) {
            std::cout << "   [" << i+1 << "/10]" << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        std::cout << "\n🔓 Sending ARM command..." << std::endl;
        vehicle->drone()->arm();
        std::cout << "✅ ARM command sent, waiting 2 seconds..." << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(2));

        std::cout << "\n📍 Preparing for takeoff..." << std::endl;
        bool ok = vehicle->drone()->takeoff_command_global(
            1.5,      // altitude_m
            0.0,      // pitch deg
            0.0,      // yaw deg
            NAN,      // lat
            NAN,      // lon
            300.0     // timeout
        );

        if (ok) {
            std::cout << "✅ Takeoff successful! Reached target altitude." << std::endl;
            
            std::cout << "\n🛸 Flying to waypoint..." << std::endl;
            vehicle->drone()->fly_to_trajectory_setpoint(5.0, 0.0, 2.0, 0.0, 100);
            std::cout << "✅ Waypoint reached!" << std::endl;
        } else {
            std::cout << "❌ Takeoff failed!" << std::endl;
        }

        std::cout << "\n🛬 Landing..." << std::endl;
        vehicle->drone()->land();
        std::cout << "✅ Landing complete." << std::endl;
        
        std::this_thread::sleep_for(std::chrono::seconds(2));
        
        std::cout << "\n🔒 Disarming..." << std::endl;
        vehicle->drone()->disarm();
        std::cout << "✅ Disarm command sent." << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "❌ Exception caught: " << e.what() << std::endl;
        std::cerr << "Attempting emergency shutdown..." << std::endl;
    }

    std::cout << "\n🛑 Shutting down..." << std::endl;
    // 清理资源
    vehicle->close();

    std::cout << "✅ Application terminated successfully." << std::endl;
    std::cout << "════════════════════════════════════════════════════════" << std::endl;
    return 0;
}
