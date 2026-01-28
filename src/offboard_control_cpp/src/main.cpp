#include <rclcpp/rclcpp.hpp>
#include "offboard_control_cpp/offboard_control.hpp"
#include "offboard_control_cpp/vehicle.hpp"
#include <chrono>
#include <thread>


int main(int argc, char* argv[]) {
    std::cout << "════════════════════════════════════════════════════════" << std::endl;
    std::cout << "🚀 PX4 Offboard Control - Attitude Control Mode" << std::endl;
    std::cout << "════════════════════════════════════════════════════════" << std::endl;
    std::cout << "⚠️  Note: Using attitude control (no position feedback)" << std::endl;
    std::cout << "         Suitable for GPS-denied/indoor environments" << std::endl;

    // 创建 Vehicle 实例（会自动初始化 ROS2 和启动心跳线程）
    std::cout << "📍 Initializing Vehicle..." << std::endl;
    auto vehicle = std::make_shared<Vehicle>();

    try {
        // 等待足够的时间让心跳线程稳定工作和订阅器连接
        std::cout << "⏳ Waiting for system initialization (10 seconds)..." << std::endl;
        std::cout << "   Offboard control signals should be transmitting..." << std::endl;
        for (int i = 0; i < 10; i++) {
            std::cout << "   [" << i+1 << "/10]" << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        std::cout << "\n🔓 Sending ARM command..." << std::endl;
        vehicle->drone()->arm();
        std::cout << "✅ ARM command sent, waiting 2 seconds..." << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(2));

        // 使用姿态控制而非位置控制(适合室内无 GPS 环境)
        std::cout << "\n📍 Setting up attitude control..." << std::endl;
        vehicle->drone()->set_control_mode("attitude");
        
        // 悬停 5 秒 - 发送小的姿态命令保持平衡
        std::cout << "🛸 Hovering for 5 seconds (attitude control)..." << std::endl;
        vehicle->drone()->update_attitude_setpoint(0.0, 0.0, 0.0, 0.5); // 50% 油门
        std::this_thread::sleep_for(std::chrono::seconds(5));

        std::cout << "✅ Hover test complete!" << std::endl;

        std::cout << "\n🛬 Landing..." << std::endl;
        // 降油门到 0
        vehicle->drone()->update_attitude_setpoint(0.0, 0.0, 0.0, 0.0);
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
