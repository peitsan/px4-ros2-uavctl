
#include "px4_hexctl/offboard_control.hpp"
#include "px4_hexctl/vehicle.hpp"
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <future>

Vehicle::Vehicle() {
    std::cout << "🌍 Initializing ROS2..." << std::endl;
    if (!rclcpp::ok()) {
        auto options = rclcpp::InitOptions();
        options.shutdown_on_signal = false;
        rclcpp::init(0, nullptr, options);
        owns_context_ = true;
    }
    drone_ = std::make_shared<OffboardControl>();
    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor_->add_node(drone_);
    spin_thread_ = std::thread([this]() { executor_->spin(); });
    RCLCPP_INFO(drone_->get_logger(), "🌀 Vehicle node spinning in background thread");
    
    // 等待一些时间让 spin 开始工作
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    drone_->heartbeat_thread_start();
    
    // 等待足够的时间让心跳信号建立（重要！）
    std::cout << "⏳ Waiting for heartbeat signals to establish (5 seconds)..." << std::endl;
    std::cout << "   🔌 Make sure MicroXRCEAgent is running on this device!" << std::endl;
    std::cout << "   🔍 Check: ps aux | grep MicroXRCEAgent" << std::endl;
    std::cout << "   🔍 Check: ros2 topic list | grep fmu/out" << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(2));
}

Vehicle::~Vehicle() {
    close();
}


void Vehicle::close() {
    if (closed_) return;
    closed_ = true;

    std::cout << "🛑 [Vehicle] Shutting down and cleaning up..." << std::endl;

    // 1️⃣ 停止心跳线程
    if (drone_) {
        std::cout << "  - Stopping heartbeat thread..." << std::endl;
        drone_->stop_heartbeat();
    }

    // 2️⃣ 取消执行器
    if (executor_) {
        std::cout << "  - Cancelling executor..." << std::endl;
        executor_->cancel();
    }

    // 3️⃣ 关闭 ROS2 
    if (owns_context_ && rclcpp::ok()) {
        std::cout << "  - Calling rclcpp::shutdown()..." << std::endl;
        rclcpp::shutdown();
    }

    // 4️⃣ 线程回收 (带超时保护)
    if (spin_thread_.joinable()) {
        std::cout << "  - Joining spin thread..." << std::endl;
        // 使用 lambda 表达式正确调用 join
        auto future = std::async(std::launch::async, [this]() { 
            if (spin_thread_.joinable()) spin_thread_.join(); 
        });
        
        if (future.wait_for(std::chrono::seconds(1)) == std::future_status::timeout) {
            std::cout << "  ⚠️ Spin thread join timed out! Detaching..." << std::endl;
            spin_thread_.detach();
        } else {
            std::cout << "  ✅ Spin thread joined successfully." << std::endl;
        }
    }

    executor_.reset();
    drone_.reset();
    std::cout << "✅ [Vehicle] Cleanup finished." << std::endl;
}
