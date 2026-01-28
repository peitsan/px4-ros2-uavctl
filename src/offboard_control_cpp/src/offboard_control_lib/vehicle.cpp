
#include "offboard_control_cpp/offboard_control.hpp"
#include "offboard_control_cpp/vehicle.hpp"
#include <rclcpp/rclcpp.hpp>
#include <thread>

Vehicle::Vehicle() {
    std::cout << "🌍 Initializing ROS2..." << std::endl;
    if (!rclcpp::ok()) {
        rclcpp::init(0, nullptr);
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
    std::this_thread::sleep_for(std::chrono::seconds(5));
    
    std::cout << "🔄 Engaging OFFBOARD mode..." << std::endl;
    drone_->engage_offboard_mode(10, 2.0);
}

Vehicle::~Vehicle() {
    close();
}


void Vehicle::close() {
    if (closed_) return;  // 防止重复关闭
    closed_ = true;

    RCLCPP_INFO(drone_->get_logger(), "🛑 Shutting down Vehicle...");

    // 1️⃣ 先停止心跳线程（自定义线程）
    drone_->stop_heartbeat();

    // 2️⃣ 请求 executor 停止
    if (executor_) {
        executor_->cancel();  // 通知 spin() 退出
    }

    // 3️⃣ 等待 spin_thread 退出
    if (spin_thread_.joinable()) {
        spin_thread_.join();
        std::cout << "✅ Spin thread has joined!" << std::endl;
    }

    // 4️⃣ 清理 executor 与节点
    executor_.reset();
    drone_.reset();

    // 5️⃣ 最后关闭 ROS2 系统
    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }

    std::cout << "✅ Vehicle shutdown complete!" << std::endl;
}