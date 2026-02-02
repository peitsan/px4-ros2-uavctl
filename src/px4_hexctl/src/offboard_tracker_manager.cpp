#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>

#include "px4_hexctl/offboard_control.hpp"
#include "px4_hexctl/vehicle.hpp"

#include <chrono>
#include <thread>
#include <iostream>
#include <csignal>
#include <atomic>
#include <mutex>

using namespace std::chrono_literals;

std::atomic<bool> g_signal_triggered(false);

void signal_handler(int signum) {
    (void)signum;
    g_signal_triggered = true;
}

struct CmdState {
    geometry_msgs::msg::Twist last_cmd{};
    rclcpp::Time last_time{0, 0, RCL_ROS_TIME};
    bool have_cmd{false};
    std::mutex mutex;
    rclcpp::Time last_log_time{0, 0, RCL_ROS_TIME};
};

struct StartupResult {
    bool success{false};
    double home_z{0.0};
    bool xy_valid{false};
};

StartupResult startup_sequence(
    const std::shared_ptr<OffboardControl> &drone,
    double takeoff_alt,
    double takeoff_thrust,
    bool allow_xy_invalid,
    double hover_thrust) {

    StartupResult result;

    if (!drone->is_position_valid()) {
        std::cout << "⚠️  EKF XY position is INVALID. Using ATTITUDE mode to bypass health checks for arming..." << std::endl;
        drone->set_control_mode("attitude");
        drone->update_attitude_setpoint(0.0, 0.0, 0.0, 0.0);
    } else {
        std::cout << "📍 EKF Position is VALID. Using standard POSITION mode..." << std::endl;
        drone->set_control_mode("position");
        drone->update_position_setpoint(0.0, 0.0, 0.0, 0.0);
    }

    std::cout << "📡 Pre-warming control signals (1 second)..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));

    auto last_request = std::chrono::steady_clock::now();
    std::cout << "⏳ Starting OFFBOARD & ARM sequence..." << std::endl;

    while (rclcpp::ok() && !g_signal_triggered) {
        auto now = std::chrono::steady_clock::now();
        auto status = drone->get_vehicle_status();

        bool is_offboard = (status.nav_state == 14);
        bool is_armed = (status.arming_state == 2);

        static auto last_print = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_print).count() >= 1500) {
            if (status.timestamp == 0) {
                std::cout << "⚠️  WARNING: No VehicleStatus message received yet! Check topic names." << std::endl;
            } else {
                std::cout << "DEBUG: [nav_state=" << static_cast<int>(status.nav_state)
                          << ", arming_state=" << static_cast<int>(status.arming_state)
                          << "] Offboard=" << (is_offboard ? "Y" : "N")
                          << ", Armed=" << (is_armed ? "Y" : "N") << std::endl;
            }
            last_print = now;
        }

        if (is_offboard && is_armed) {
            std::cout << "✅ System Ready & Armed! EKF Valid: "
                      << (drone->is_position_valid() ? "YES" : "NO") << std::endl;

            if (!drone->is_position_valid()) {
                std::cout << "🚀 [ADAPTIVE] EKF not ready. Performing Attitude-based liftoff..." << std::endl;
                for (int i = 0; i < 10; ++i) {
                    drone->update_attitude_setpoint(0.0, 0.0, 0.0, 0.68);
                    std::this_thread::sleep_for(100ms);
                    if (drone->is_position_valid()) break;
                }

                std::cout << "⏳ Waiting for EKF XY to stabilize while airborne..." << std::endl;
                auto liftoff_start = std::chrono::steady_clock::now();
                while (rclcpp::ok() && !drone->is_position_valid() &&
                       std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - liftoff_start).count() < 5) {
                    drone->update_attitude_setpoint(0.0, 0.0, 0.0, 0.58);
                    std::this_thread::sleep_for(200ms);
                }
            }

            if (drone->is_position_valid()) {
                std::cout << "📍 EKF is now VALID. Switching to POSITION mode for stabilizing..." << std::endl;
                drone->set_control_mode("position");

                auto pos = drone->get_local_position();
                result.home_z = pos.z;
                double target_z = result.home_z + takeoff_alt;
                drone->update_position_setpoint(0.0, 0.0, target_z, 0.0);
                std::this_thread::sleep_for(500ms);
                result.success = true;
                result.xy_valid = true;
            } else {
                if (allow_xy_invalid) {
                    std::cout << "⚠️ EKF XY still invalid. Continue in ATTITUDE hover mode." << std::endl;
                    drone->set_control_mode("attitude");
                    drone->update_attitude_setpoint(0.0, 0.0, 0.0, hover_thrust);
                    result.success = true;
                    result.xy_valid = false;
                } else {
                    std::cout << "❌ EKF failed to normalize after liftoff. Safety Landing..." << std::endl;
                    drone->land();
                }
            }
            break;
        }

        if (std::chrono::duration_cast<std::chrono::seconds>(now - last_request).count() >= 3) {
            last_request = now;

            if (!is_offboard) {
                std::cout << "🔄 Requesting OFFBOARD mode (Current nav_state=" << static_cast<int>(status.nav_state) << ")..." << std::endl;
                drone->publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0);
            } else if (!is_armed) {
                std::cout << "🔓 Requesting ARM (Current arming_state=" << static_cast<int>(status.arming_state) << ")..." << std::endl;
                drone->arm();
            }
        }

        std::this_thread::sleep_for(100ms);
    }

    return result;
}

int main(int argc, char* argv[]) {
    if (!rclcpp::ok()) {
        auto options = rclcpp::InitOptions();
        options.shutdown_on_signal = false;
        rclcpp::init(argc, argv, options);
    }
    std::signal(SIGINT, signal_handler);

    auto node = std::make_shared<rclcpp::Node>("offboard_tracker_manager");
    node->declare_parameter("cmd_vel_topic", "/qr_tracker/cmd_vel_body");
    node->declare_parameter("takeoff_alt", 1.2);
    node->declare_parameter("command_timeout", 0.5);
    node->declare_parameter("takeoff_thrust", 0.68);
    node->declare_parameter("allow_xy_invalid", true);
    node->declare_parameter("hover_thrust", 0.58);

    const auto cmd_vel_topic = node->get_parameter("cmd_vel_topic").as_string();
    const auto takeoff_alt = node->get_parameter("takeoff_alt").as_double();
    const auto command_timeout = node->get_parameter("command_timeout").as_double();
    const auto takeoff_thrust = node->get_parameter("takeoff_thrust").as_double();
    const auto allow_xy_invalid = node->get_parameter("allow_xy_invalid").as_bool();
    const auto hover_thrust = node->get_parameter("hover_thrust").as_double();

    std::cout << "════════════════════════════════════════════════════════" << std::endl;
    std::cout << "🚀 PX4 Offboard Tracker Manager" << std::endl;
    std::cout << "════════════════════════════════════════════════════════" << std::endl;

    auto vehicle = std::make_shared<Vehicle>();
    auto drone = vehicle->drone();

    CmdState cmd_state;
    auto cmd_sub = node->create_subscription<geometry_msgs::msg::Twist>(
        cmd_vel_topic, 10,
        [&cmd_state, &node](const geometry_msgs::msg::Twist::SharedPtr msg) {
            std::lock_guard<std::mutex> guard(cmd_state.mutex);
            cmd_state.last_cmd = *msg;
            cmd_state.last_time = rclcpp::Clock(RCL_ROS_TIME).now();
            cmd_state.have_cmd = true;
            auto now_time = node->now();
            if ((now_time - cmd_state.last_log_time).seconds() >= 0.8) {
                RCLCPP_INFO(node->get_logger(), "✅ move command received: vx=%.3f vy=%.3f vz=%.3f", msg->linear.x, msg->linear.y, msg->linear.z);
                cmd_state.last_log_time = now_time;
            }
        });

    (void)cmd_sub;

    auto startup = startup_sequence(drone, takeoff_alt, takeoff_thrust, allow_xy_invalid, hover_thrust);
    if (!startup.success || g_signal_triggered) {
        vehicle->close();
        if (rclcpp::ok()) rclcpp::shutdown();
        return 0;
    }

    std::cout << "✅ Hovering, waiting for /qr_tracker/cmd_vel_body" << std::endl;

    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node);
    rclcpp::WallRate rate(20.0);

    while (rclcpp::ok() && !g_signal_triggered) {
        exec.spin_some();

        geometry_msgs::msg::Twist cmd;
        bool use_cmd = false;
        rclcpp::Time last_time;

        {
            std::lock_guard<std::mutex> guard(cmd_state.mutex);
            cmd = cmd_state.last_cmd;
            use_cmd = cmd_state.have_cmd;
            last_time = cmd_state.last_time;
        }

        if (use_cmd) {
            double dt = (node->now() - last_time).seconds();
            if (dt > command_timeout) {
                use_cmd = false;
            }
        }

        if (startup.xy_valid) {
            auto pos = drone->get_local_position();
            double target_z = startup.home_z + takeoff_alt;

            if (!use_cmd) {
                drone->update_position_setpoint(pos.x, pos.y, target_z, 0.0);
            } else {
                double vx = cmd.linear.x;
                double vy = cmd.linear.y;
                double vz = cmd.linear.z;
                drone->update_velocity_setpoint(vx, vy, vz, 0.0);
            }
        } else {
            if (!use_cmd) {
                drone->update_attitude_setpoint(0.0, 0.0, 0.0, hover_thrust);
            } else {
                double vx = cmd.linear.x;
                double vy = cmd.linear.y;
                double vz = -cmd.linear.z;
                drone->update_velocity_setpoint(vx, vy, vz, 0.0);
            }
        }

        rate.sleep();
    }

    if (g_signal_triggered) {
        auto status = drone->get_vehicle_status();
        if (status.arming_state == 2) {
            drone->publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
        }
    }

    vehicle->close();
    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }
    return 0;
}// Note: duplicate content removed.
