#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/goto_setpoint.hpp>
#include "offboard_control_cpp/offboard_control.hpp"
#include "offboard_control_cpp/vehicle.hpp"

#include <chrono>
#include <thread>
#include <mutex>
#include <unordered_map>
#include <cmath>
#include <array>
#include <vector>
#include <string>
#include <cmath> // for M_PI and NAN
#include <iostream>

using namespace std::chrono_literals;

// Helper: degrees -> radians
static inline double deg2rad(double deg) {
    return deg * M_PI / 180.0;
}

std::string namespace_ = "";  // Global namespace

// Class implementation
OffboardControl::OffboardControl() : Node("offboard_control_center") {
    RCLCPP_INFO(this->get_logger(), "🚀 [INIT] Initializing OffboardControl node...");

    rclcpp::QoS qos_profile(1);
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    qos_profile.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
    qos_profile.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);

    // Publishers
    offboard_control_mode_publisher_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
        namespace_ + "/fmu/in/offboard_control_mode", qos_profile);
    vehicle_command_publisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
        namespace_ + "/fmu/in/vehicle_command", qos_profile);
    trajectory_setpoint_publisher_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
        namespace_ + "/fmu/in/trajectory_setpoint", qos_profile);
    goto_setpoint_publisher_ = this->create_publisher<px4_msgs::msg::GotoSetpoint>(
        namespace_ + "/fmu/in/goto_setpoint", qos_profile);  // Assuming GotoSetpoint exists
    vehicle_attitude_setpoint_publisher_ = this->create_publisher<px4_msgs::msg::VehicleAttitudeSetpoint>(
        namespace_ + "/fmu/in/vehicle_attitude_setpoint_v1", qos_profile);

    RCLCPP_INFO(this->get_logger(), "[PUB] Created publishers under namespace: '%s'", namespace_.empty() ? "default" : namespace_.c_str());

    // Subscribers
    vehicle_local_position_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        namespace_ + "/fmu/out/vehicle_local_position_v1", qos_profile,
        std::bind(&OffboardControl::vehicle_local_position_callback, this, std::placeholders::_1));
    vehicle_status_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
        namespace_ + "/fmu/out/vehicle_status", qos_profile,
        std::bind(&OffboardControl::vehicle_status_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "[SUB] Subscribed to vehicle_local_position_v1 and vehicle_status");

    // State variables
    offboard_setpoint_counter_ = 0;
    takeoff_height_ = 2.0;
    home_position_ = {0.0, 0.0, 0.0};
    vehicle_local_position_enu_ = px4_msgs::msg::VehicleLocalPosition();
    vehicle_local_position_received_ = false;
    vehicle_status_ = px4_msgs::msg::VehicleStatus();
    control_mode_ = "position";

    // Flags and locks
    is_takeoff_complete_ = false;
    target_reached_ = false;

    // Target
    target_ = {0.0, 0.0, 0.0, 0.0};

    RCLCPP_INFO(this->get_logger(), "✅ [INIT] OffboardControl initialized successfully!");
}

OffboardControl::~OffboardControl() {
    if (heartbeat_thread_.joinable()) {
        stop_heartbeat_ = true;
        heartbeat_thread_.join();
    }
}

void OffboardControl::throttle_log(double interval_sec, const std::string& msg, const std::string& level, const std::string& tag) {
    auto now = std::chrono::system_clock::now().time_since_epoch().count() / 1e9;
    auto it = last_log_.find(tag);
    if (it == last_log_.end() || (now - it->second) > interval_sec) {
        if (level == "info") {
            RCLCPP_INFO(this->get_logger(), "%s", msg.c_str());
        } else if (level == "warn") {
            RCLCPP_WARN(this->get_logger(), "%s", msg.c_str());
        } else if (level == "error") {
            RCLCPP_ERROR(this->get_logger(), "%s", msg.c_str());
        }
        last_log_[tag] = now;
    }
}

void OffboardControl::heartbeat_thread_start() {
    //stop_heartbeat_.clear();
    stop_heartbeat_ = false;
    heartbeat_hz_ = 20;
    heartbeat_thread_ = std::thread(&OffboardControl::heartbeat_loop, this);
    RCLCPP_INFO(this->get_logger(), "🔁 [HEARTBEAT] Started heartbeat thread at %d Hz", heartbeat_hz_);
}

void OffboardControl::heartbeat_loop() {
    double rate = 1.0 / static_cast<double>(heartbeat_hz_);
    RCLCPP_DEBUG(this->get_logger(), "[HEARTBEAT] Entering heartbeat loop...");
    int heartbeat_count = 0;
    auto last_log_time = std::chrono::system_clock::now();
    
    while (!stop_heartbeat_ && rclcpp::ok()) {
        try {
            publish_offboard_control_heartbeat_signal(control_mode_);
            publish_current_setpoint();
            offboard_setpoint_counter_++;
            heartbeat_count++;
            
            // Every 100 heartbeats (5 seconds at 20Hz), show status
            auto now = std::chrono::system_clock::now();
            if (std::chrono::duration<double>(now - last_log_time).count() >= 5.0) {
                RCLCPP_INFO(this->get_logger(), "💓 [HEARTBEAT] Steady: %d signals sent, Counter: %d", 
                            heartbeat_count, offboard_setpoint_counter_);
                heartbeat_count = 0;
                last_log_time = now;
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "[HEARTBEAT] Exception in loop: %s", e.what());
        }
        std::this_thread::sleep_for(std::chrono::duration<double>(rate));
    }
    RCLCPP_INFO(this->get_logger(), "⏹️ [HEARTBEAT] Heartbeat thread exiting");
}

void OffboardControl::publish_current_setpoint() {
    std::lock_guard<std::mutex> guard(lock_);
    std::string mode = control_mode_;
    auto target = target_;
    if (mode == "position") {
        double x = target[0], y = target[1], z = target[2], yaw = target[3];
        publish_trajectory_setpoint({x, y, z}, {}, {}, {}, yaw, {});
    } else if (mode == "velocity") {
        double vx = target[0], vy = target[1], vz = target[2], yawspeed = target[3];
        publish_trajectory_setpoint({}, {vx, vy, vz}, {}, {}, {}, yawspeed);
    } else if (mode == "attitude") {
        double roll = target[0], pitch = target[1], yaw = target[2], thrust = target[3];
        auto q_d = euler_to_quaternion(roll, pitch, yaw);
        std::vector<double> thrust_body = {0.0, 0.0, -thrust};
        publish_attitude_setpoint(q_d, thrust_body);
    } else {
        RCLCPP_WARN(this->get_logger(), "[SETPOINT] Unsupported control mode: %s", mode.c_str());
    }
}

void OffboardControl::set_control_mode(const std::string& mode) {
    if (mode == "position" || mode == "velocity" || mode == "attitude") {
        std::lock_guard<std::mutex> guard(lock_);
        control_mode_ = mode;
        RCLCPP_INFO(this->get_logger(), "🔄 [MODE] Switched to %s control mode", mode.c_str());
    } else {
        RCLCPP_ERROR(this->get_logger(), "❌ [MODE] Invalid control mode: %s", mode.c_str());
    }
}

void OffboardControl::update_position_setpoint(double x, double y, double z, double yaw) {
    std::lock_guard<std::mutex> guard(lock_);
    if (control_mode_ != "position") {
        set_control_mode("position");
    }
    auto old_target = target_;
    target_ = {x, y, z, yaw};
    RCLCPP_DEBUG(this->get_logger(), "🎯 [POSITION] Updated from (%f,%f,%f,%f) → (%f,%f,%f,%f) (ENU)",
                 old_target[0], old_target[1], old_target[2], old_target[3],
                 target_[0], target_[1], target_[2], target_[3]);
}

void OffboardControl::update_velocity_setpoint(double vx, double vy, double vz, double yawspeed) {
    std::lock_guard<std::mutex> guard(lock_);
    if (control_mode_ != "velocity") {
        set_control_mode("velocity");
    }
    auto old_target = target_;
    target_ = {vx, vy, vz, yawspeed};
    RCLCPP_DEBUG(this->get_logger(), "🎯 [VELOCITY] Updated from (%f,%f,%f,%f) → (%f,%f,%f,%f) (ENU)",
                 old_target[0], old_target[1], old_target[2], old_target[3],
                 target_[0], target_[1], target_[2], target_[3]);
}

void OffboardControl::update_attitude_setpoint(double roll, double pitch, double yaw, double thrust) {
    std::lock_guard<std::mutex> guard(lock_);
    if (control_mode_ != "attitude") {
        set_control_mode("attitude");
    }
    auto old_target = target_;
    target_ = {roll, pitch, yaw, thrust};
    RCLCPP_DEBUG(this->get_logger(), "🎯 [ATTITUDE] Updated from (%f,%f,%f,%f) → (%f,%f,%f,%f)",
                 old_target[0], old_target[1], old_target[2], old_target[3],
                 target_[0], target_[1], target_[2], target_[3]);
}

std::array<double, 3> OffboardControl::ned_to_enu(double x_ned, double y_ned, double z_ned) {
    return {y_ned, x_ned, -z_ned};
}

std::array<double, 3> OffboardControl::enu_to_ned(double x_enu, double y_enu, double z_enu) {
    return {y_enu, x_enu, -z_enu};
}

double OffboardControl::normalize_yaw(double yaw_diff) {
    while (yaw_diff > M_PI) yaw_diff -= 2 * M_PI;
    while (yaw_diff < -M_PI) yaw_diff += 2 * M_PI;
    return std::abs(yaw_diff);
}

std::vector<double> OffboardControl::euler_to_quaternion(double roll, double pitch, double yaw) {
    double cy = std::cos(yaw * 0.5);
    double sy = std::sin(yaw * 0.5);
    double cp = std::cos(pitch * 0.5);
    double sp = std::sin(pitch * 0.5);
    double cr = std::cos(roll * 0.5);
    double sr = std::sin(roll * 0.5);
    double w = cr * cp * cy + sr * sp * sy;
    double x = sr * cp * cy - cr * sp * sy;
    double y = cr * sp * cy + sr * cp * sy;
    double z = cr * cp * sy - sr * sp * cy;
    return {w, x, y, z};
}

void OffboardControl::vehicle_local_position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
    try {
        auto [x_enu, y_enu, z_enu] = ned_to_enu(msg->x, msg->y, msg->z);
        double heading_enu = -msg->heading + M_PI_2;
        {
            std::lock_guard<std::mutex> guard(lock_);
            vehicle_local_position_enu_.x = x_enu;
            vehicle_local_position_enu_.y = y_enu;
            vehicle_local_position_enu_.z = z_enu;
            vehicle_local_position_enu_.heading = heading_enu;
            
            // 首次接收到位置数据时输出
            if (!vehicle_local_position_received_) {
                RCLCPP_INFO(this->get_logger(), "✅ [POSITION] FIRST POSITION RECEIVED! ENU=(%f, %f, %f)", x_enu, y_enu, z_enu);
            }
            vehicle_local_position_received_ = true;
        }
        std::string log_msg = "[POSITION] ENU=(" + std::to_string(x_enu) + ", " + std::to_string(y_enu) + ", " + std::to_string(z_enu) + "), heading=" + std::to_string(heading_enu * 180 / M_PI) + "°";
        throttle_log(2.0, log_msg, "info", "position");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "[POSITION] Callback error: %s", e.what());
    }
}

void OffboardControl::vehicle_status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
    std::string old_nav, old_arm;
    {
        std::lock_guard<std::mutex> guard(lock_);
        old_nav = std::to_string(vehicle_status_.nav_state);
        old_arm = std::to_string(vehicle_status_.arming_state);
        vehicle_status_ = *msg;
    }
    std::string log_msg = "[STATUS] nav_state=" + std::to_string(msg->nav_state) + " (was " + old_nav + "), arming_state=" + std::to_string(msg->arming_state) + " (was " + old_arm + ")";
    throttle_log(5.0, log_msg, "info", "status");
}

void OffboardControl::arm() {
    RCLCPP_INFO(this->get_logger(), "🔓 Sending ARM command...");
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
    RCLCPP_INFO(this->get_logger(), "✅ Arm command sent");

    // 等待位置数据,但设置 10 秒超时(室内环境可能没有位置)
    if (!vehicle_local_position_received_) {
        RCLCPP_WARN(this->get_logger(), "⚠️ Waiting for position data (may not be available in indoor environments)...");
    }

    int wait_count = 0;
    int max_wait_count = 20; // 10 seconds timeout (500ms * 20)
    
    while (!vehicle_local_position_received_ && rclcpp::ok() && wait_count < max_wait_count) {
        wait_count++;
        if (wait_count % 2 == 0) {  // Log every second
            RCLCPP_WARN(this->get_logger(), "   ⏳ Still waiting for position... (%d/%d s)", wait_count/2, max_wait_count/2);
        }
        std::this_thread::sleep_for(500ms);
    }
    
    // 不再强制要求位置数据 - 室内环境可能没有
    if (!vehicle_local_position_received_) {
        RCLCPP_WARN(this->get_logger(), "⚠️  Position data not available (expected in indoor/GPS-denied environments)");
        RCLCPP_WARN(this->get_logger(), "   Using attitude-only control mode");
        // 继续执行,不返回
    } else {
        RCLCPP_INFO(this->get_logger(), "✅ Position feedback established!");
    }

    {
        std::lock_guard<std::mutex> guard(lock_);
        if (vehicle_local_position_received_) {
            home_position_ = {vehicle_local_position_enu_.x, vehicle_local_position_enu_.y, vehicle_local_position_enu_.z};
            RCLCPP_INFO(this->get_logger(), "🏠 Home position recorded: (%f, %f, %f) (ENU)", 
                        home_position_[0], home_position_[1], home_position_[2]);
        } else {
            home_position_ = {0.0, 0.0, 0.0};
            RCLCPP_WARN(this->get_logger(), "⚠️ Using default home position (0,0,0) - no position data available");
        }
    }
}

void OffboardControl::disarm() {
    RCLCPP_INFO(this->get_logger(), "🔒 Sending DISARM command...");
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
    RCLCPP_INFO(this->get_logger(), "✅ Disarm command sent");
}

void OffboardControl::engage_offboard_mode(int prewarm_count, double prewarm_timeout) {
    RCLCPP_INFO(this->get_logger(), "🔄 Engaging OFFBOARD mode (prewarm: %d msgs or %f s)", prewarm_count, prewarm_timeout);
    RCLCPP_INFO(this->get_logger(), "   Current setpoint counter: %d", offboard_setpoint_counter_);

    auto start = std::chrono::system_clock::now();
    int last_counter = offboard_setpoint_counter_;
    
    while (offboard_setpoint_counter_ < prewarm_count && 
           std::chrono::duration<double>(std::chrono::system_clock::now() - start).count() < prewarm_timeout && 
           rclcpp::ok()) {
        std::this_thread::sleep_for(50ms);
        
        // 每秒输出一次进度
        double elapsed = std::chrono::duration<double>(std::chrono::system_clock::now() - start).count();
        if ((int)elapsed % 1 == 0 && offboard_setpoint_counter_ != last_counter) {
            RCLCPP_INFO(this->get_logger(), "   [Prewarm] Progress: %d/%d setpoints (%f s)", 
                        offboard_setpoint_counter_, prewarm_count, elapsed);
            last_counter = offboard_setpoint_counter_;
        }
    }

    if (offboard_setpoint_counter_ < prewarm_count) {
        RCLCPP_WARN(this->get_logger(), "⚠️ Prewarm insufficient: only %d/%d setpoints sent (may still work)", 
                    offboard_setpoint_counter_, prewarm_count);
    } else {
        RCLCPP_INFO(this->get_logger(), "✅ Prewarm complete: %d setpoints sent", offboard_setpoint_counter_);
    }

    RCLCPP_INFO(this->get_logger(), "🔄 Sending OFFBOARD mode command (VEHICLE_CMD_DO_SET_MODE)...");
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0);
    RCLCPP_INFO(this->get_logger(), "✅ OFFBOARD mode command sent!");
}

bool OffboardControl::hover(double duration, double timeout) {
    if (duration <= 0) {
        RCLCPP_ERROR(this->get_logger(), "❌ Hover duration must be positive!");
        return false;
    }

    if (timeout < 0) {
        timeout = duration + 10.0;
    }

    double cx, cy, cz, ch;
    {
        std::lock_guard<std::mutex> guard(lock_);
        if (!vehicle_local_position_received_) {
            RCLCPP_WARN(this->get_logger(), "⚠️ No valid position received; cannot hover.");
            return false;
        }
        cx = vehicle_local_position_enu_.x;
        cy = vehicle_local_position_enu_.y;
        cz = vehicle_local_position_enu_.z;
        ch = vehicle_local_position_enu_.heading;
    }

    update_position_setpoint(cx, cy, cz, ch);
    RCLCPP_INFO(this->get_logger(), "🛸 Starting hover at ENU (%f, %f, %f), yaw=%f° for %fs", cx, cy, cz, ch * 180 / M_PI, duration);

    auto start = std::chrono::system_clock::now().time_since_epoch().count() / 1e9;
    double last_log = start;
    while (rclcpp::ok() && (std::chrono::system_clock::now().time_since_epoch().count() / 1e9 - start) < timeout) {
        double elapsed = std::chrono::system_clock::now().time_since_epoch().count() / 1e9 - start;
        if (elapsed >= duration) {
            RCLCPP_INFO(this->get_logger(), "✅ Hover duration completed!");
            return true;
        }

        if (std::chrono::system_clock::now().time_since_epoch().count() / 1e9 - last_log >= 1.0) {
            throttle_log(1.0, "[HOVER] Elapsed: " + std::to_string(elapsed) + "/" + std::to_string(duration) + "s", "info", "hover");
            last_log = std::chrono::system_clock::now().time_since_epoch().count() / 1e9;
        }

        std::this_thread::sleep_for(100ms);
    }

    RCLCPP_WARN(this->get_logger(), "⚠️ Hover timed out!");
    return false;
}

bool OffboardControl::land(double latitude, double longitude, double altitude, double yaw, double abort_alt, int land_mode, double timeout) {
    RCLCPP_INFO(this->get_logger(), "🛬 Sending LAND command at lat=%f, lon=%f, alt=%f m, yaw=%s", latitude, longitude, altitude, std::isnan(yaw) ? "default" : std::to_string(yaw).c_str());

    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND, abort_alt, static_cast<double>(land_mode), 0.0, yaw, latitude, longitude, altitude);

    auto start = std::chrono::system_clock::now().time_since_epoch().count() / 1e9;
    double last_log = start;
    while (rclcpp::ok() && (std::chrono::system_clock::now().time_since_epoch().count() / 1e9 - start) < timeout) {
        double cz;
        uint8_t nav_state;
        {
            std::lock_guard<std::mutex> guard(lock_);
            cz = vehicle_local_position_enu_.z;
            nav_state = vehicle_status_.nav_state;
        }
        double remaining_time = timeout - (std::chrono::system_clock::now().time_since_epoch().count() / 1e9 - start);

        if (std::chrono::system_clock::now().time_since_epoch().count() / 1e9 - last_log >= 1.0) {
            throttle_log(1.0, "[LAND] Altitude: " + std::to_string(cz) + "m, nav_state=" + std::to_string(nav_state) + ", remaining time: " + std::to_string(remaining_time) + "s", "info", "land");
            last_log = std::chrono::system_clock::now().time_since_epoch().count() / 1e9;
        }

        if (cz < 0.1 || nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LAND || vehicle_status_.arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_DISARMED) {
            RCLCPP_INFO(this->get_logger(), "✅ Landing complete!");
            return true;
        }

        std::this_thread::sleep_for(100ms);
    }

    RCLCPP_WARN(this->get_logger(), "⚠️ Land timed out!");
    return false;
}

void OffboardControl::publish_offboard_control_heartbeat_signal(const std::string& control_mode) {
    auto msg = px4_msgs::msg::OffboardControlMode();
    msg.position = (control_mode == "position");
    msg.velocity = (control_mode == "velocity");
    msg.acceleration = (control_mode == "acceleration");
    msg.attitude = (control_mode == "attitude");
    msg.body_rate = (control_mode == "body_rate");
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    offboard_control_mode_publisher_->publish(msg);
}

void OffboardControl::publish_trajectory_setpoint(
    std::vector<double> position,
    std::vector<double> velocity,
    std::vector<double> acceleration,
    std::vector<double> jerk,
    double yaw,
    double yawspeed
) {
    auto msg = px4_msgs::msg::TrajectorySetpoint();
   

    std::vector<double> nan3 = {NAN, NAN, NAN};

    if (!position.empty()) {
        auto [x_ned, y_ned, z_ned] = enu_to_ned(position[0], position[1], position[2]);
         msg.position = {static_cast<float>(x_ned),
                static_cast<float>(y_ned),
                static_cast<float>(z_ned)};
    } else {
        msg.position = {NAN, NAN, NAN};
    }

    if (!velocity.empty()) {
        auto [vx_ned, vy_ned, vz_ned] = enu_to_ned(velocity[0], velocity[1], velocity[2]);
        msg.velocity = {static_cast<float>(vx_ned), 
                static_cast<float>(vy_ned), 
                static_cast<float>(vz_ned)};
    } else {
        msg.velocity = {NAN, NAN, NAN};
    }

    if (!acceleration.empty()) {
        auto [ax_ned, ay_ned, az_ned] = enu_to_ned(acceleration[0], acceleration[1], acceleration[2]);
        msg.acceleration = {static_cast<float>(ax_ned),
                    static_cast<float>(ay_ned),
                    static_cast<float>(az_ned)};
    } else {
        msg.acceleration = {NAN, NAN, NAN};
    }

    if (!jerk.empty()) {
        auto [jx_ned, jy_ned, jz_ned] = enu_to_ned(jerk[0], jerk[1], jerk[2]);
        msg.jerk = {static_cast<float>(jx_ned),
            static_cast<float>(jy_ned),
            static_cast<float>(jz_ned)};
    } else {
        msg.jerk = {NAN, NAN, NAN};
    }

    msg.yaw = std::isnan(yaw) ? NAN : -yaw + M_PI_2;

    msg.yawspeed = std::isnan(yawspeed) ? NAN : -yawspeed;

    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    trajectory_setpoint_publisher_->publish(msg);

    // Debug log (simplified)
    std::string log_str = "[PUB] TrajectorySetpoint NED: ";
    if (!position.empty()) log_str += "pos=(" + std::to_string(msg.position[0]) + "," + std::to_string(msg.position[1]) + "," + std::to_string(msg.position[2]) + "), ";
    RCLCPP_DEBUG(this->get_logger(), "%s", log_str.c_str());
}

void OffboardControl::publish_attitude_setpoint(const std::vector<double>& q_d, const std::vector<double>& thrust_body, double yaw_sp_move_rate) {
    auto msg = px4_msgs::msg::VehicleAttitudeSetpoint();
    msg.q_d = {static_cast<float>(q_d[0]), static_cast<float>(q_d[1]), static_cast<float>(q_d[2]), static_cast<float>(q_d[3])};
    msg.thrust_body = {static_cast<float>(thrust_body[0]), static_cast<float>(thrust_body[1]), static_cast<float>(thrust_body[2])};

    msg.yaw_sp_move_rate = std::isnan(yaw_sp_move_rate) ? NAN : static_cast<float>(yaw_sp_move_rate);

    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    vehicle_attitude_setpoint_publisher_->publish(msg);

    // Debug log
    std::string log_str = "[PUB] VehicleAttitudeSetpoint: q_d=[" + std::to_string(msg.q_d[0]) + "," + std::to_string(msg.q_d[1]) + "," + std::to_string(msg.q_d[2]) + "," + std::to_string(msg.q_d[3]) + "], thrust_body=[" + std::to_string(msg.thrust_body[0]) + "," + std::to_string(msg.thrust_body[1]) + "," + std::to_string(msg.thrust_body[2]) + "]";
    if (!std::isnan(yaw_sp_move_rate)) log_str += ", yaw_sp_move_rate=" + std::to_string(yaw_sp_move_rate);
    RCLCPP_DEBUG(this->get_logger(), "%s", log_str.c_str());
}

void OffboardControl::publish_vehicle_command(uint16_t command, double param1, double param2, double param3, double param4, double param5, double param6, double param7) {
    auto msg = px4_msgs::msg::VehicleCommand();
    msg.command = command;
    msg.param1 = param1;
    msg.param2 = param2;
    msg.param3 = param3;
    msg.param4 = param4;
    msg.param5 = param5;
    msg.param6 = param6;
    msg.param7 = param7;
    // Assume namespace handling
    int sys_id = 1;
    try {
        std::string stripped = namespace_.substr(namespace_.find("/px4_") + 5);
        sys_id = std::stoi(stripped) + 1;
    } catch (...) {
        sys_id = 1;
    }
    msg.target_system = sys_id;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    vehicle_command_publisher_->publish(msg);
}

bool OffboardControl::takeoff(double takeoff_height, double timeout) {
    if (takeoff_height <= 0) {
        RCLCPP_ERROR(this->get_logger(), "❌ Takeoff height must be positive!");
        return false;
    }

    double home_z;
    {
        std::lock_guard<std::mutex> guard(lock_);
        home_z = home_position_[2];
    }
    double target_alt = home_z + takeoff_height;
    double current_heading = vehicle_local_position_received_ ? vehicle_local_position_enu_.heading : 0.0;
    update_position_setpoint(home_position_[0], home_position_[1], target_alt, current_heading);

    RCLCPP_INFO(this->get_logger(), "🛫 Starting takeoff to %f m (from %f m)", target_alt, home_z);

    auto start = std::chrono::system_clock::now().time_since_epoch().count() / 1e9;
    while (rclcpp::ok() && (std::chrono::system_clock::now().time_since_epoch().count() / 1e9 - start) < timeout) {
        double current_z;
        {
            std::lock_guard<std::mutex> guard(lock_);
            current_z = vehicle_local_position_enu_.z;
        }
        double remaining = target_alt - current_z;
        throttle_log(1.0, "[TAKEOFF] Altitude: " + std::to_string(current_z) + "/" + std::to_string(target_alt) + " m, Δ=" + std::to_string(remaining) + " m", "info", "takeoff");
        if (remaining <= 0.1) {
            RCLCPP_INFO(this->get_logger(), "✅ Takeoff complete!");
            return true;
        }
        std::this_thread::sleep_for(100ms);
    }

    RCLCPP_WARN(this->get_logger(), "⚠️ Takeoff timed out!");
    return false;
}

// 1) 全局（WGS84）起飞命令：VEHICLE_CMD_NAV_TAKEOFF (22)
// param1: pitch (deg) ; param4: yaw (deg) ; param5: lat ; param6: lon ; param7: altitude (m AMSL)
// 说明：发送命令后会等待 vehicle_local_position_enu_.z 达到 home_z + altitude_m（或接近），并在超时返回 false。
bool OffboardControl::takeoff_command_global(double altitude_m,
                                             double pitch_deg,
                                             double yaw_deg,
                                             double latitude,
                                             double longitude,
                                             double timeout_s)
{
    if (altitude_m <= 0.0) {
        RCLCPP_ERROR(this->get_logger(), "❌ takeoff_command_global: altitude_m must be positive");
        return false;
    }

    // 构造并发送命令
    // publish_vehicle_command(command, p1, p2, p3, p4, p5, p6, p7)
    double altitude_sent = altitude_m + 1;
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF,
                            pitch_deg, // param1 pitch (deg)
                            0.0,       // param2 empty
                            0.0,       // param3 empty
                            std::isnan(yaw_deg) ? NAN : yaw_deg, // param4 yaw deg (or NAN)
                            std::isnan(latitude) ? NAN : latitude, // param5 lat
                            std::isnan(longitude) ? NAN : longitude, // param6 lon
                            altitude_sent); // param7 altitude AMSL (m)

    RCLCPP_INFO(this->get_logger(), "🛫 Sent VEHICLE_CMD_NAV_TAKEOFF: alt=%.2f m, pitch=%.1f°, yaw=%s",
                altitude_m, pitch_deg,
                std::isnan(yaw_deg) ? "NAN" : std::to_string(yaw_deg).c_str());

    // 现在等待实际上升：比较 vehicle_local_position_enu_.z 与 (home_z + altitude_m)
    double home_z;
    {
        std::lock_guard<std::mutex> guard(lock_);
        home_z = home_position_[2];
    }
    double target_z = home_z + altitude_m;

    double start = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
    double last_log = start;
    while (rclcpp::ok()) {
        double now = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
        if (now - start > timeout_s) {
            RCLCPP_WARN(this->get_logger(), "⚠️ takeoff_command_global timed out after %.1f s", timeout_s);
            return false;
        }

        double cz;
        {
            std::lock_guard<std::mutex> guard(lock_);
            cz = vehicle_local_position_enu_.z;
        }

        // 如果高度到位（小于容差，或超过目标）
        if ((target_z - cz) <= 0.1) {
            RCLCPP_INFO(this->get_logger(), "✅ takeoff_command_global reached target: target_z=%.3f, current_z=%.3f", target_z, cz);
            return true;
        }

        // 每隔 1s 打一次节流日志
        if (now - last_log >= 1.0) {
            throttle_log(1.0, "[TAKEOFF_GLOBAL] current_z=" + std::to_string(cz) + ", target_z=" + std::to_string(target_z), "info", "takeoff_global");
            last_log = now;
        }

        std::this_thread::sleep_for(100ms);
    }

    return false; // rclcpp 被关闭等异常情况
}


// 2) 局部（Local frame）起飞命令：VEHICLE_CMD_NAV_TAKEOFF_LOCAL (24)
// param1: pitch (rad) ; param3: ascend rate (m/s) ; param4: yaw (rad) ; param5: Y (m) ; param6: X (m) ; param7: Z (m)
// 说明：z_rel_m 是相对于本地参考系(ENU)的高度目标（通常为正），ascend_rate_m_s 可用来指定爬升速率。
// 这里等待 vehicle_local_position_enu_.z 接近 z_rel_m（地面到目标高度）。
bool OffboardControl::takeoff_command_local(double z_rel_m,
                                            double ascend_rate_m_s,
                                            double pitch_rad,
                                            double yaw_rad,
                                            double x_m,
                                            double y_m,
                                            double timeout_s)
{
    if (z_rel_m <= 0.0) {
        RCLCPP_ERROR(this->get_logger(), "❌ takeoff_command_local: z_rel_m must be positive");
        return false;
    }

    // param1 expect RAD for this local command per your spec
    publish_vehicle_command(24,
                            static_cast<double>(pitch_rad),  // param1 pitch (rad)
                            0.0,                            // param2 empty
                            ascend_rate_m_s,                // param3 ascend rate (m/s)
                            std::isnan(yaw_rad) ? NAN : yaw_rad, // param4 yaw (rad)
                            std::isnan(y_m) ? NAN : y_m,    // param5 Y pos (m)
                            std::isnan(x_m) ? NAN : x_m,    // param6 X pos (m)
                            z_rel_m);                       // param7 Z pos (m)

    RCLCPP_INFO(this->get_logger(), "🛫 Sent VEHICLE_CMD_NAV_TAKEOFF_LOCAL: z=%.2f m, ascend_rate=%.2f m/s, pitch=%.3f rad", z_rel_m, ascend_rate_m_s, pitch_rad);

    double start = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
    double last_log = start;
    while (rclcpp::ok()) {
        double now = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
        if (now - start > timeout_s) {
            RCLCPP_WARN(this->get_logger(), "⚠️ takeoff_command_local timed out after %.1f s", timeout_s);
            return false;
        }

        double cz;
        {
            std::lock_guard<std::mutex> guard(lock_);
            cz = vehicle_local_position_enu_.z;
        }

        // 对于本地命令，vehicle_local_position_enu_.z 通常表示当前位置 (m)
        // 期望 cz 接近 z_rel_m (允许 0.1m 容差)
        if (std::abs(cz - z_rel_m) <= 0.1 || cz >= z_rel_m) {
            RCLCPP_INFO(this->get_logger(), "✅ takeoff_command_local reached target: target_z=%.3f, current_z=%.3f", z_rel_m, cz);
            return true;
        }

        if (now - last_log >= 1.0) {
            throttle_log(1.0, "[TAKEOFF_LOCAL] current_z=" + std::to_string(cz) + ", target_z=" + std::to_string(z_rel_m), "info", "takeoff_local");
            last_log = now;
        }

        std::this_thread::sleep_for(100ms);
    }

    return false;
}


bool OffboardControl::simulated_land(double descent_rate, double ground_tolerance, double timeout) {
    if (descent_rate >= 0) {
        RCLCPP_ERROR(this->get_logger(), "❌ Descent rate must be negative for landing!");
        return false;
    }

    double cx, cy, cz, ch;
    {
        std::lock_guard<std::mutex> guard(lock_);
        if (!vehicle_local_position_received_) {
            RCLCPP_WARN(this->get_logger(), "⚠️ No valid position received; cannot land.");
            return false;
        }
        cx = vehicle_local_position_enu_.x;
        cy = vehicle_local_position_enu_.y;
        cz = vehicle_local_position_enu_.z;
        ch = vehicle_local_position_enu_.heading;
    }

    set_control_mode("velocity");
    RCLCPP_INFO(this->get_logger(), "🛬 Starting simulated land from altitude %f m with descent rate %f m/s", cz, descent_rate);

    auto start = std::chrono::system_clock::now().time_since_epoch().count() / 1e9;
    double last_log = start;
    double target_z = 0.0;
    while (rclcpp::ok() && (std::chrono::system_clock::now().time_since_epoch().count() / 1e9 - start) < timeout) {
        double current_z;
        uint8_t nav_state;
        {
            std::lock_guard<std::mutex> guard(lock_);
            current_z = vehicle_local_position_enu_.z;
            nav_state = vehicle_status_.nav_state;
        }

        double remaining_dist = current_z - target_z;
        double vz = std::max(descent_rate, -remaining_dist * 2.0);
        update_velocity_setpoint(0.0, 0.0, vz, 0.0);

        if (std::chrono::system_clock::now().time_since_epoch().count() / 1e9 - last_log >= 1.0) {
            throttle_log(1.0, "[SIM_LAND] Altitude: " + std::to_string(current_z) + "m, vz=" + std::to_string(vz) + " m/s, remaining: " + std::to_string(remaining_dist) + "m", "info", "sim_land");
            last_log = std::chrono::system_clock::now().time_since_epoch().count() / 1e9;
        }

        if (current_z <= ground_tolerance) {
            update_velocity_setpoint(0.0, 0.0, 0.0, 0.0);
            RCLCPP_INFO(this->get_logger(), "✅ Simulated landing complete! Altitude near ground.");
            return true;
        }

        std::this_thread::sleep_for(50ms);
    }

    RCLCPP_WARN(this->get_logger(), "⚠️ Simulated land timed out!");
    update_velocity_setpoint(0.0, 0.0, 0.0, 0.0);
    return false;
}

bool OffboardControl::fly_to_trajectory_setpoint(double x, double y, double z, double yaw, double timeout) {
    update_position_setpoint(x, y, z, yaw);
    RCLCPP_INFO(this->get_logger(), "✈️ Flying to ENU target: (%f, %f, %f), yaw=%f°", x, y, z, yaw * 180 / M_PI);

    auto start = std::chrono::system_clock::now().time_since_epoch().count() / 1e9;
    while (rclcpp::ok() && (std::chrono::system_clock::now().time_since_epoch().count() / 1e9 - start) < timeout) {
        double cx, cy, cz, ch;
        {
            std::lock_guard<std::mutex> guard(lock_);
            cx = vehicle_local_position_enu_.x;
            cy = vehicle_local_position_enu_.y;
            cz = vehicle_local_position_enu_.z;
            ch = vehicle_local_position_enu_.heading;
        }
        double dist = std::sqrt((cx - x) * (cx - x) + (cy - y) * (cy - y) + (cz - z) * (cz - z));
        double yaw_diff = normalize_yaw(ch - yaw);
        throttle_log(1.0, "[FLYTO] Remaining distance: " + std::to_string(dist) + " m, yaw diff: " + std::to_string(yaw_diff) + " rad (" + std::to_string(yaw_diff * 180 / M_PI) + "°)", "info", "flyto");
        if (dist < DISTANCE_TOLERANCE && yaw_diff < YAW_TOLERANCE) {
            RCLCPP_INFO(this->get_logger(), "✅ Target reached!");
            return true;
        }
        std::this_thread::sleep_for(100ms);
    }

    RCLCPP_WARN(this->get_logger(), "⚠️ fly_to_trajectory_setpoint timed out!");
    return false;
}