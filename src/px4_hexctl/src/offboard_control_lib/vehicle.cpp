
#include "px4_hexctl/offboard_control.hpp"
#include "px4_hexctl/vehicle.hpp"
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
    std::cout << "   🔌 Make sure MicroXRCEAgent is running on this device!" << std::endl;
    std::cout << "   🔍 Check: ps aux | grep MicroXRCEAgent" << std::endl;
    std::cout << "   🔍 Check: ros2 topic list | grep fmu/out" << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(2));
}

Vehicle::~Vehicle() {
    close();
}


void Vehicle::close() {
    if (closed_) return;  // 防止重复关闭
    closed_ = true;

    std::cout << "🛑 Shutting down Vehicle and cleaning up ROS2..." << std::endl;

    // 1️⃣ 先停止心跳线程
    drone_->stop_heartbeat();

    // 2️⃣ 取消所有待处理的回调并停止分派器
    if (executor_) {
        executor_->cancel(); 
    }

    // 3️⃣ 尝试关闭 ROS2 系统（这会使 rclcpp::ok() 返回 false）
    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }

    // 4️⃣ 等待 spin 线程退出，设置超时以防死锁
    if (spin_thread_.joinable()) {
        // 对于复杂的死锁，我们可以考虑不使用 join() 而是 detach()，
        // 但为了优雅关闭，我们尝试等待一小会儿
        spin_thread_.join();
        std::cout << "✅ Spin thread has joined!" << std::endl;
    }

    // 5️⃣ 清理内存
    executor_.reset();
    drone_.reset();

    std::cout << "✅ Vehicle shutdown complete!" << std::endl;
}
