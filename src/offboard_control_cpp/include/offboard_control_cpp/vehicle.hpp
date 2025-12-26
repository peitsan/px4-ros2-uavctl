#pragma once

#include <memory>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>

// Forward declaration for OffboardControl
class OffboardControl;

class Vehicle {
public:
    Vehicle();
    ~Vehicle();

    void close();
    std::shared_ptr<OffboardControl> drone() { return drone_; }

private:
    std::shared_ptr<OffboardControl> drone_;
    std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
    std::thread spin_thread_;
    bool closed_ = false;
};