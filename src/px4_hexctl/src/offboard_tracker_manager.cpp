#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>

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
        declare_parameter("takeoff_alt", 1.2);
        declare_parameter("command_timeout", 0.5);
        declare_parameter("enable_adaptive_liftoff", false);
        declare_parameter("takeoff_thrust", 0.68);
        declare_parameter("takeoff_mode", "attitude");
        declare_parameter("hold_altitude", true);

        cmd_vel_topic_ = get_parameter("cmd_vel_topic").as_string();
        takeoff_alt_ = get_parameter("takeoff_alt").as_double();
        command_timeout_ = get_parameter("command_timeout").as_double();
        enable_adaptive_liftoff_ = get_parameter("enable_adaptive_liftoff").as_bool();
        takeoff_thrust_ = get_parameter("takeoff_thrust").as_double();
        takeoff_mode_ = get_parameter("takeoff_mode").as_string();
        hold_altitude_ = get_parameter("hold_altitude").as_bool();

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

        if (!startup_ready_) {
            if (!control_mode_set_) {
                if (!drone_->is_position_valid()) {
                    drone_->set_control_mode("attitude");
                    drone_->update_attitude_setpoint(0.0, 0.0, 0.0, 0.0);
                } else {
                    drone_->set_control_mode("position");
                    drone_->update_position_setpoint(0.0, 0.0, 0.0, 0.0);
                }
                control_mode_set_ = true;
            }

            if (prewarm_count_ < prewarm_target_) {
                if (takeoff_mode_ == "attitude" || !drone_->is_position_valid()) {
                    drone_->update_attitude_setpoint(0.0, 0.0, 0.0, takeoff_thrust_ * 0.0);
                } else {
                    drone_->update_position_setpoint(0.0, 0.0, 0.0, 0.0);
                }
                prewarm_count_++;
                return;
            }

            auto now_time = now();
            if ((now_time - last_request_time_).seconds() >= 3.0) {
                last_request_time_ = now_time;
                if (!is_offboard) {
                    drone_->publish_vehicle_command(
                        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0);
                } else if (!is_armed) {
                    drone_->arm();
                }
            }

            if (is_offboard && is_armed) {
                startup_ready_ = true;
            }
            return;
        }

        if (!takeoff_done_) {
            if (drone_->is_position_valid()) {
                if (!home_z_initialized_) {
                    home_z_ = drone_->get_local_position().z;
                    home_z_initialized_ = true;
                }
                double target_z = home_z_ + takeoff_alt_;

                if (takeoff_mode_ == "attitude") {
                    if (drone_->get_local_position().z < target_z - 0.1) {
                        drone_->update_attitude_setpoint(0.0, 0.0, 0.0, takeoff_thrust_);
                        return;
                    }
                    takeoff_done_ = true;
                } else {
                    if (drone_->get_local_position().z < target_z - 0.2) {
                        drone_->update_position_setpoint(0.0, 0.0, target_z, 0.0);
                        return;
                    }
                    takeoff_done_ = true;
                }
            } else if (enable_adaptive_liftoff_ || takeoff_mode_ == "attitude") {
                drone_->update_attitude_setpoint(0.0, 0.0, 0.0, takeoff_thrust_);
                return;
            }
        }

        if (!takeoff_done_) {
            return;
        }

        if (!have_cmd_) {
            if (hold_altitude_ && drone_->is_position_valid() && home_z_initialized_) {
                double target_z = home_z_ + takeoff_alt_;
                auto pos = drone_->get_local_position();
                drone_->update_position_setpoint(pos.x, pos.y, target_z, 0.0);
                return;
            }
            drone_->update_velocity_setpoint(0.0, 0.0, 0.0, 0.0);
            return;
        }

        double dt = (now() - last_cmd_time_).seconds();
        if (dt > command_timeout_) {
            if (hold_altitude_ && drone_->is_position_valid() && home_z_initialized_) {
                double target_z = home_z_ + takeoff_alt_;
                auto pos = drone_->get_local_position();
                drone_->update_position_setpoint(pos.x, pos.y, target_z, 0.0);
                return;
            }
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
    std::string takeoff_mode_ = "attitude";
    bool hold_altitude_ = true;

    bool takeoff_done_ = false;
    bool home_z_initialized_ = false;
    double home_z_ = 0.0;

    bool startup_ready_ = false;
    bool control_mode_set_ = false;
    int prewarm_count_ = 0;
    int prewarm_target_ = 40;
    rclcpp::Time last_request_time_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OffboardTrackerManager>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
