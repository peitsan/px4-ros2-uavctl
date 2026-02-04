#include <rclcpp/rclcpp.hpp>
#include "px4_hexctl/offboard_control.hpp"
#include "px4_hexctl/vehicle.hpp"

#include <atomic>
#include <csignal>
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

static std::atomic<bool> g_stop(false);

static void signal_handler(int) {
    g_stop = true;
}

int main(int argc, char* argv[]) {
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

        drone->engage_offboard_mode();
        drone->arm();

        const double takeoff_height = 1.5;
        const bool ok = drone->takeoff(takeoff_height, 20.0);

        if (ok && !g_stop) {
            const double hover_seconds = 5.0;
            auto start = std::chrono::steady_clock::now();
            while (rclcpp::ok() && !g_stop &&
                   std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() < hover_seconds) {
                std::this_thread::sleep_for(100ms);
            }
        }

        drone->land();
        drone->disarm();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("offboard_takeoff"), "Exception caught: %s", e.what());
    }

    vehicle->close();
    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }
    return 0;
}
