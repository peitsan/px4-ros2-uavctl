#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <thread>
#include <atomic>
#include <csignal>

#include "px4_hexctl/offboard_control.hpp"
#include "px4_hexctl/vehicle.hpp"

using namespace std::chrono_literals;

static std::atomic<bool> g_stop(false);

static void signal_handler(int) {
    g_stop = true;
}

int main(int argc, char **argv) {
    if (!rclcpp::ok()) {
        auto options = rclcpp::InitOptions();
        options.shutdown_on_signal = false;
        rclcpp::init(argc, argv, options);
    }

    std::signal(SIGINT, signal_handler);

    auto vehicle = std::make_shared<Vehicle>();
    auto drone = vehicle->drone();

    try {
        drone->set_control_mode("position");
        drone->update_position_setpoint(0.0, 0.0, 0.0, 0.0);

        drone->engage_offboard_mode(20, 5.0);
        drone->arm();

        const auto wait_start = std::chrono::steady_clock::now();
        const double wait_timeout = 5.0;
        while (rclcpp::ok() && !g_stop && !drone->is_z_valid()) {
            if (std::chrono::duration<double>(std::chrono::steady_clock::now() - wait_start).count() > wait_timeout) {
                RCLCPP_ERROR(drone->get_logger(), "❌ No valid height data, aborting hover.");
                drone->land();
                drone->disarm();
                vehicle->close();
                return 1;
            }
            std::this_thread::sleep_for(200ms);
        }

        const double takeoff_height = 2.0;
        if (!drone->takeoff(takeoff_height, 20.0)) {
            RCLCPP_ERROR(drone->get_logger(), "❌ Takeoff failed, exiting.");
            drone->land();
            drone->disarm();
            vehicle->close();
            return 1;
        }

        const auto &pos = drone->get_local_position();
        drone->update_position_setpoint(pos.x, pos.y, pos.z, pos.heading);
        RCLCPP_INFO(drone->get_logger(), "🛸 Hovering at ENU (%.2f, %.2f, %.2f)", pos.x, pos.y, pos.z);

        const double hover_timeout = 20.0;
        auto hover_start = std::chrono::steady_clock::now();
        while (rclcpp::ok() && !g_stop &&
               std::chrono::duration<double>(std::chrono::steady_clock::now() - hover_start).count() < hover_timeout) {
            std::this_thread::sleep_for(200ms);
        }

        drone->land();
        drone->disarm();
    } catch (const std::exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("offboard_single_position"), "Exception: %s", e.what());
    }

    vehicle->close();
    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }
    return 0;
}