#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include "px4_hexctl/offboard_control.hpp"
#include "px4_hexctl/vehicle.hpp"

#include <chrono>
#include <memory>

using namespace std::chrono_literals;

class OffboardTrackerManager : public rclcpp::Node {
public:
    OffboardTrackerManager()
        : Node("offboard_tracker_manager") {
        declare_parameter("cmd_vel_topic", "/qr_tracker/cmd_vel_body");
        declare_parameter("takeoff_alt", 1.5);
        declare_parameter("command_timeout", 0.5);
        declare_parameter("enable_adaptive_liftoff", false);
        declare_parameter("takeoff_thrust", 0.68);

        cmd_vel_topic_ = get_parameter("cmd_vel_topic").as_string();
        takeoff_alt_ = get_parameter("takeoff_alt").as_double();
        command_timeout_ = get_parameter("command_timeout").as_double();
        enable_adaptive_liftoff_ = get_parameter("enable_adaptive_liftoff").as_bool();
        takeoff_thrust_ = get_parameter("takeoff_thrust").as_double();

        auto vehicle = std::make_shared<Vehicle>();
        drone_ = vehicle->drone();

        cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            cmd_vel_topic_, 10,
            std::bind(&OffboardTrackerManager::cmd_callback, this, std::placeholders::_1));

        timer_ = create_wall_timer(50ms, std::bind(&OffboardTrackerManager::control_loop, this));

        RCLCPP_INFO(get_logger(), "🚀 Offboard tracker manager started. cmd_vel_topic=%s", cmd_vel_topic_.c_str());
    }

private:
    void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        last_cmd_ = *msg;
        last_cmd_time_ = now();
        have_cmd_ = true;
    }

    void control_loop() {
        if (!drone_) {
            return;
        }

        auto status = drone_->get_vehicle_status();
        bool is_offboard = (status.nav_state == 14);
        bool is_armed = (status.arming_state == 2);

        if (!is_offboard || !is_armed) {
            drone_->arm();
            drone_->engage_offboard_mode();
            return;
        }

        if (!takeoff_done_) {
            if (drone_->is_position_valid()) {
                if (!home_z_initialized_) {
                    home_z_ = drone_->get_local_position().z;
                    home_z_initialized_ = true;
                }
                double target_z = home_z_ + takeoff_alt_;
                if (drone_->get_local_position().z < target_z - 0.2) {
                    drone_->update_position_setpoint(0.0, 0.0, target_z, 0.0);
                    return;
                }
                takeoff_done_ = true;
            } else if (enable_adaptive_liftoff_) {
                drone_->update_attitude_setpoint(0.0, 0.0, 0.0, takeoff_thrust_);
                return;
            }
        }

        if (!takeoff_done_) {
            return;
        }

        if (!have_cmd_) {
            drone_->update_velocity_setpoint(0.0, 0.0, 0.0, 0.0);
            return;
        }

        double dt = (now() - last_cmd_time_).seconds();
        if (dt > command_timeout_) {
            drone_->update_velocity_setpoint(0.0, 0.0, 0.0, 0.0);
            return;
        }

        // Body frame: x forward, y right, z down
        // ENU: x east, y north, z up (assume yaw aligned)
        double vx = last_cmd_.linear.x;
        double vy = last_cmd_.linear.y;
        double vz = -last_cmd_.linear.z;

        drone_->update_velocity_setpoint(vx, vy, vz, 0.0);
    }

    std::shared_ptr<OffboardControl> drone_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::string cmd_vel_topic_;
    geometry_msgs::msg::Twist last_cmd_;
    rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};
    bool have_cmd_ = false;

    double takeoff_alt_ = 1.5;
    double command_timeout_ = 0.5;
    bool enable_adaptive_liftoff_ = false;
    double takeoff_thrust_ = 0.68;

    bool takeoff_done_ = false;
    bool home_z_initialized_ = false;
    double home_z_ = 0.0;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OffboardTrackerManager>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
