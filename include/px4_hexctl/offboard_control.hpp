```cpp
// ...existing code...
    bool fly_to_trajectory_setpoint(double x, double y, double z, double yaw, double timeout);
    void reset_home() {
        std::lock_guard<std::mutex> guard(lock_);
        if (vehicle_local_position_received_) {
            home_position_ = {vehicle_local_position_enu_.x, vehicle_local_position_enu_.y, vehicle_local_position_enu_.z};
            RCLCPP_INFO(this->get_logger(), "🏠 [HOME] Reset to current: (%f, %f, %f)", home_position_[0], home_position_[1], home_position_[2]);
        }
    }
    
    bool takeoff_command_global(double altitude_m,
    // ...existing code...
```