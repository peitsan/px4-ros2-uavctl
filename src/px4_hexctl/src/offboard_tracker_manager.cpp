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
        startup_stage_ = StartupStage::INIT;
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
            auto now_time = now();
            switch (startup_stage_) {
                case StartupStage::INIT: {
                    if (!drone_->is_position_valid()) {
                        RCLCPP_WARN(get_logger(), "⚠️ EKF XY invalid. Using ATTITUDE mode for arming.");
                        drone_->set_control_mode("attitude");
                        drone_->update_attitude_setpoint(0.0, 0.0, 0.0, 0.0);
                    } else {
                        RCLCPP_INFO(get_logger(), "📍 EKF valid. Using POSITION mode.");
                        drone_->set_control_mode("position");
                        drone_->update_position_setpoint(0.0, 0.0, 0.0, 0.0);
                    }
                    prewarm_end_time_ = now_time + rclcpp::Duration::from_seconds(1.0);
                    startup_stage_ = StartupStage::PREWARM;
                    RCLCPP_INFO(get_logger(), "📡 Pre-warming control signals (1 second)...");
                    return;
                }
                case StartupStage::PREWARM: {
                    if (takeoff_mode_ == "attitude" || !drone_->is_position_valid()) {
                        drone_->update_attitude_setpoint(0.0, 0.0, 0.0, 0.0);
                    } else {
                        drone_->update_position_setpoint(0.0, 0.0, 0.0, 0.0);
                    }
                    if (now_time >= prewarm_end_time_) {
                        startup_stage_ = StartupStage::REQUESTS;
                        last_request_time_ = now_time;
                        last_print_time_ = now_time;
                        RCLCPP_INFO(get_logger(), "⏳ Starting OFFBOARD & ARM sequence...");
                    }
                    return;
                }
                case StartupStage::REQUESTS: {
                    if ((now_time - last_print_time_).seconds() >= 1.5) {
                        if (status.timestamp == 0) {
                            RCLCPP_WARN(get_logger(), "⚠️ No VehicleStatus yet. Check fmu/out topics.");
                        } else {
                            RCLCPP_INFO(get_logger(), "DEBUG: nav_state=%d, arming_state=%d, offboard=%s, armed=%s",
                                status.nav_state, status.arming_state, is_offboard ? "Y" : "N", is_armed ? "Y" : "N");
                        }
                        last_print_time_ = now_time;
                    }

                    if ((now_time - last_request_time_).seconds() >= 3.0) {
                        last_request_time_ = now_time;
                        if (!is_offboard) {
                            RCLCPP_INFO(get_logger(), "🔄 Requesting OFFBOARD mode...");
                            drone_->publish_vehicle_command(
                                px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0);
                        } else if (!is_armed) {
                            RCLCPP_INFO(get_logger(), "🔓 Requesting ARM...");
                            drone_->arm();
                        }
                    }

                    if (is_offboard && is_armed) {
                        if (!drone_->is_position_valid()) {
                            RCLCPP_WARN(get_logger(), "🚀 [ADAPTIVE] EKF not ready. Attitude liftoff...");
                            liftoff_start_time_ = now_time;
                            liftoff_burst_end_time_ = now_time + rclcpp::Duration::from_seconds(1.0);
                            startup_stage_ = StartupStage::ADAPTIVE_LIFTOFF;
                        } else {
                            if (!home_z_initialized_) {
                                home_z_ = drone_->get_local_position().z;
                                home_z_initialized_ = true;
                            }
                            stabilize_end_time_ = now_time + rclcpp::Duration::from_seconds(0.5);
                            startup_stage_ = StartupStage::STABILIZE;
                            RCLCPP_INFO(get_logger(), "📍 EKF valid. Stabilizing at target altitude...");
                        }
                    }
                    return;
                }
                case StartupStage::ADAPTIVE_LIFTOFF: {
                    if (now_time <= liftoff_burst_end_time_) {
                        drone_->update_attitude_setpoint(0.0, 0.0, 0.0, 0.68);
                        return;
                    }

                    if ((now_time - liftoff_start_time_).seconds() < 5.0) {
                        drone_->update_attitude_setpoint(0.0, 0.0, 0.0, 0.58);
                        if (drone_->is_position_valid()) {
                            if (!home_z_initialized_) {
                                home_z_ = drone_->get_local_position().z;
                                home_z_initialized_ = true;
                            }
                            stabilize_end_time_ = now_time + rclcpp::Duration::from_seconds(0.5);
                            startup_stage_ = StartupStage::STABILIZE;
                            RCLCPP_INFO(get_logger(), "📍 EKF recovered. Stabilizing...");
                        }
                        return;
                    }

                    RCLCPP_ERROR(get_logger(), "❌ EKF failed to normalize after liftoff. Landing.");
                    drone_->land();
                    startup_failed_ = true;
                    return;
                }
                case StartupStage::STABILIZE: {
                    if (!home_z_initialized_ && drone_->is_position_valid()) {
                        home_z_ = drone_->get_local_position().z;
                        home_z_initialized_ = true;
                    }
                    if (home_z_initialized_) {
                        double target_z = home_z_ + takeoff_alt_;
                        drone_->set_control_mode("position");
                        drone_->update_position_setpoint(0.0, 0.0, target_z, 0.0);
                    }
                    if (now_time >= stabilize_end_time_) {
                        startup_ready_ = true;
                        takeoff_done_ = true;
                        RCLCPP_INFO(get_logger(), "✅ System Ready & Armed. Holding altitude.");
                    }
                    return;
                }
                case StartupStage::READY:
                    break;
            }
            return;
        }

        if (startup_failed_) {
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

    enum class StartupStage {
        INIT,
        PREWARM,
        REQUESTS,
        ADAPTIVE_LIFTOFF,
        STABILIZE,
        READY
    };

    StartupStage startup_stage_ = StartupStage::INIT;
    bool startup_ready_ = false;
    bool startup_failed_ = false;
    rclcpp::Time prewarm_end_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_request_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_print_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time liftoff_start_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time liftoff_burst_end_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time stabilize_end_time_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OffboardTrackerManager>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
