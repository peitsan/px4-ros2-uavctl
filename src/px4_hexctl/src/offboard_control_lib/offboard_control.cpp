#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_imu.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/goto_setpoint.hpp>
#include "px4_hexctl/offboard_control.hpp"
#include "px4_hexctl/vehicle.hpp"

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

// Class implementation
OffboardControl::OffboardControl() : OffboardControl("", "offboard_control_center") {}

OffboardControl::OffboardControl(const std::string& prefix, const std::string& node_name) 
    : Node(node_name), namespace_(prefix) {
    RCLCPP_INFO(this->get_logger(), "🚀 [INIT] Initializing OffboardControl node '%s' with namespace '%s'...", 
                node_name.c_str(), namespace_.c_str());

    // QoS Setup
    // 1. 标准传感器/状态数据 (Best Effort, Volatile)
    rclcpp::QoS sensor_qos(10);
    sensor_qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    sensor_qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    // 2. 命令发布 (Reliable, Volatile)
    rclcpp::QoS cmd_qos(10);
    cmd_qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    cmd_qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    // Publishers
    offboard_control_mode_publisher_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
        namespace_ + "/fmu/in/offboard_control_mode", sensor_qos);
    vehicle_command_publisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
        namespace_ + "/fmu/in/vehicle_command", cmd_qos);
    trajectory_setpoint_publisher_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
        namespace_ + "/fmu/in/trajectory_setpoint", sensor_qos);
    vehicle_attitude_setpoint_publisher_ = this->create_publisher<px4_msgs::msg::VehicleAttitudeSetpoint>(
        namespace_ + "/fmu/in/vehicle_attitude_setpoint", sensor_qos);

    RCLCPP_INFO(this->get_logger(), "[PUB] Created publishers under namespace: '%s'", namespace_.empty() ? "default" : namespace_.c_str());

    // Subscribers
    vehicle_local_position_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        namespace_ + "/fmu/out/vehicle_local_position", sensor_qos,
        std::bind(&OffboardControl::vehicle_local_position_callback, this, std::placeholders::_1));
    
    // 冗余订阅状态话题，解决不同 PX4 版本的兼容性问题 (例如 v1.14+ 可能带有 _v1 后缀)
    vehicle_status_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
        namespace_ + "/fmu/out/vehicle_status", sensor_qos,
        std::bind(&OffboardControl::vehicle_status_callback, this, std::placeholders::_1));

    // 尝试订阅带 _v1 后缀的路径 (用户环境下实际存在的话题)
    vehicle_status_alt_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
        namespace_ + "/fmu/out/vehicle_status_v1", sensor_qos,
        std::bind(&OffboardControl::vehicle_status_callback, this, std::placeholders::_1));

    vehicle_attitude_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>(
        namespace_ + "/fmu/out/vehicle_attitude", sensor_qos,
        std::bind(&OffboardControl::vehicle_attitude_callback, this, std::placeholders::_1));

    vehicle_imu_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleImu>(
        namespace_ + "/fmu/out/vehicle_imu", sensor_qos,
        std::bind(&OffboardControl::vehicle_imu_callback, this, std::placeholders::_1));

    vehicle_odometry_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
        namespace_ + "/fmu/out/vehicle_odometry", sensor_qos,
        std::bind(&OffboardControl::vehicle_odometry_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "[SUB] Status subscribers established (Binding to /fmu/out/vehicle_status_v1)");

    // State variables
    offboard_setpoint_counter_ = 0;
    takeoff_height_ = 1.0; // Default takeoff height
    home_position_ = {0.0, 0.0, 0.0};
    vehicle_local_position_enu_ = px4_msgs::msg::VehicleLocalPosition();
    vehicle_local_position_received_ = false;
    vehicle_status_ = px4_msgs::msg::VehicleStatus();
    vehicle_status_.timestamp = 0; // 用来检测是否收到过数据
    control_mode_ = "position";

    // Flags and locks
    is_takeoff_complete_ = false;
    target_reached_ = false;

    // Target
    target_ = {0.0, 0.0, 0.0, 0.0};

    reset_imu_estimator(0.0);

    RCLCPP_INFO(this->get_logger(), "✅ [INIT] OffboardControl initialized successfully!");
}

OffboardControl::~OffboardControl() {
    RCLCPP_INFO(this->get_logger(), "🛑 [CLEANUP] Shutting down OffboardControl...");
    
    // 停止起飞线程
    if (takeoff_thread_.joinable()) {
        takeoff_running_ = false;
        takeoff_thread_.join();
        RCLCPP_INFO(this->get_logger(), "✅ Takeoff thread stopped");
    }
    
    // 执行安全着陆和解锁
    try {
        // 短暂延迟以确保所有话题处理完毕
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        // 发送着陆命令
        RCLCPP_WARN(this->get_logger(), "🛬 Sending LAND command...");
        publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND, 
                                0.0,    // abort alt
                                0,      // land mode
                                0.0, 0.0, NAN, NAN, 0.0);
        
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        // 发送解锁命令
        RCLCPP_WARN(this->get_logger(), "🔒 Sending DISARM command...");
        publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
        
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        RCLCPP_INFO(this->get_logger(), "✅ [CLEANUP] Shutdown complete");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "❌ [CLEANUP] Exception during shutdown: %s", e.what());
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

void OffboardControl::stop_heartbeat() {
    stop_heartbeat_ = true;
    if (heartbeat_thread_.joinable()) {
        heartbeat_thread_.join();
        RCLCPP_INFO(this->get_logger(), "✅ Heartbeat thread stopped");
    }
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
            vehicle_local_position_enu_.timestamp = msg->timestamp; 
            xy_valid_ = msg->xy_valid;
            z_valid_ = msg->z_valid;
        }

        if (!vehicle_local_position_received_) {
            vehicle_local_position_received_ = true;
            RCLCPP_INFO(this->get_logger(), "✅ [POSITION] FIRST POSITION RECEIVED! ENU=(%f, %f, %f), VALID:(XY:%d, Z:%d)", x_enu, y_enu, z_enu, msg->xy_valid, msg->z_valid);
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "[POSITION] Callback error: %s", e.what());
    }
}

void OffboardControl::vehicle_status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
    if (vehicle_status_.timestamp == 0 && msg->timestamp > 0) {
        RCLCPP_INFO(this->get_logger(), "✅ [STATUS] FIRST STATUS RECEIVED! nav_state=%d, arming_state=%d", msg->nav_state, msg->arming_state);
    }
    
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

void OffboardControl::vehicle_attitude_callback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg) {
    std::lock_guard<std::mutex> guard(lock_);
    vehicle_attitude_ = *msg;
    vehicle_attitude_received_ = true;
}

void OffboardControl::reset_imu_estimator(double z0) {
    std::lock_guard<std::mutex> guard(lock_);
    imu_z_ = z0;
    imu_vz_ = 0.0;
    imu_bias_z_ = 0.0;
    imu_last_timestamp_ = 0;
    imu_estimator_initialized_ = false;
}

void OffboardControl::vehicle_imu_callback(const px4_msgs::msg::VehicleImu::SharedPtr msg) {
    if (!vehicle_attitude_received_) {
        return;
    }

    px4_msgs::msg::VehicleAttitude attitude;
    bool pos_valid;
    double pos_z;
    {
        std::lock_guard<std::mutex> guard(lock_);
        attitude = vehicle_attitude_;
        pos_valid = vehicle_local_position_received_ && z_valid_;
        pos_z = vehicle_local_position_enu_.z;
    }

    if (imu_last_timestamp_ == 0) {
        imu_last_timestamp_ = msg->timestamp;
        if (!imu_estimator_initialized_ && pos_valid) {
            imu_z_ = pos_z;
            imu_estimator_initialized_ = true;
        }
        return;
    }

    double dt = (msg->timestamp - imu_last_timestamp_) * 1e-6;
    imu_last_timestamp_ = msg->timestamp;
    if (dt <= 0.0 || dt > 0.1) {
        return;
    }

    const double dt_imu = (msg->delta_velocity_dt > 1e-6) ? msg->delta_velocity_dt : dt;

    const double qw = attitude.q[0];
    const double qx = attitude.q[1];
    const double qy = attitude.q[2];
    const double qz = attitude.q[3];

    const double r11 = 1.0 - 2.0 * (qy * qy + qz * qz);
    const double r12 = 2.0 * (qx * qy - qz * qw);
    const double r13 = 2.0 * (qx * qz + qy * qw);
    const double r21 = 2.0 * (qx * qy + qz * qw);
    const double r22 = 1.0 - 2.0 * (qx * qx + qz * qz);
    const double r23 = 2.0 * (qy * qz - qx * qw);
    const double r31 = 2.0 * (qx * qz - qy * qw);
    const double r32 = 2.0 * (qy * qz + qx * qw);
    const double r33 = 1.0 - 2.0 * (qx * qx + qy * qy);

    const double ax_b = msg->delta_velocity[0] / dt_imu;
    const double ay_b = msg->delta_velocity[1] / dt_imu;
    const double az_b = msg->delta_velocity[2] / dt_imu;

    const double ax_n = r11 * ax_b + r12 * ay_b + r13 * az_b;
    const double ay_n = r21 * ax_b + r22 * ay_b + r23 * az_b;
    const double az_n = r31 * ax_b + r32 * ay_b + r33 * az_b;

    const double g = 9.80665;
    const double az_n_lin = az_n - g;
    const double az_enu = -az_n_lin;

    if (!imu_estimator_initialized_) {
        if (pos_valid) {
            imu_z_ = pos_z;
        }
        imu_vz_ = 0.0;
        imu_bias_z_ = 0.0;
        imu_estimator_initialized_ = true;
    }

    const double az_corr = az_enu - imu_bias_z_;
    imu_vz_ += az_corr * dt;
    imu_z_ += imu_vz_ * dt;

    if (pos_valid) {
        const double error = pos_z - imu_z_;
        const double k_pos = 0.02;
        const double k_bias = 0.001;
        if (std::abs(imu_vz_) < 0.3) {
            imu_z_ += k_pos * error;
            imu_bias_z_ -= k_bias * error;
        }
    }

    if (std::abs(az_corr) < 0.2 && std::abs(imu_vz_) < 0.1) {
        imu_vz_ *= 0.9;
    }
}

void OffboardControl::vehicle_odometry_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
    if (msg->pose_frame != px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED) {
        std::lock_guard<std::mutex> guard(lock_);
        vehicle_odometry_received_ = false;
        return;
    }
    auto [x_enu, y_enu, z_enu] = ned_to_enu(msg->position[0], msg->position[1], msg->position[2]);
    std::lock_guard<std::mutex> guard(lock_);
    odom_z_raw_enu_ = z_enu;
    odom_raw_initialized_ = true;
    if (!odom_offset_initialized_) {
        odom_z_offset_ = z_enu;
        odom_offset_initialized_ = true;
        RCLCPP_INFO(this->get_logger(), "📌 ODOM Z offset initialized: %.3f m", odom_z_offset_);
    }
    const double z_rel = z_enu - odom_z_offset_;
    if (!odom_z_filtered_initialized_) {
        odom_z_filtered_ = z_rel;
        odom_z_filtered_initialized_ = true;
    } else {
        const double jump = std::abs(z_rel - odom_z_filtered_);
        if (jump > 0.3) {
            throttle_log(1.0,
                "[ODOM] Ignoring Z spike: z_rel=" + std::to_string(z_rel) +
                " filtered=" + std::to_string(odom_z_filtered_),
                "warn", "odom_spike");
            return;
        }
        if (jump >= 0.03) {
            const double alpha = 0.35;
            odom_z_filtered_ = alpha * z_rel + (1.0 - alpha) * odom_z_filtered_;
        }
    }
    vehicle_odometry_enu_ = *msg;
    vehicle_odometry_enu_.position[0] = x_enu;
    vehicle_odometry_enu_.position[1] = y_enu;
    vehicle_odometry_enu_.position[2] = odom_z_filtered_;
    vehicle_odometry_received_ = (msg->timestamp > 0);
}

void OffboardControl::arm() {
    RCLCPP_INFO(this->get_logger(), "🔓 Sending ARM command...");
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
    RCLCPP_INFO(this->get_logger(), "✅ Arm command sent");

    // 如果当前是姿态控制模式，不需要等待位置数据
    {
        std::lock_guard<std::mutex> guard(lock_);
        if (control_mode_ == "attitude") {
            RCLCPP_INFO(this->get_logger(), "🚀 Attitude mode detected, skipping position check.");
            return;
        }
    }

    // 等待位置数据...
    if (!vehicle_local_position_received_) {
        RCLCPP_WARN(this->get_logger(), "⚠️ Waiting for position data (may not be available in indoor environments)...");
    }

    int wait_count = 0;
    int max_wait_count = 6; // 3 seconds timeout (500ms * 6)
    
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
        
        // ============ 电机安全限速：限制爬升和下降速度 ============
        // 防止 PX4 位置控制器自动计算的速度过大导致电机过载
        // 注意：位置控制模式下 velocity[] 为空，此限制仅在显式速度模式时生效
        double max_vz_up = 0.6;      // 最大爬升速度 0.6 m/s
        double max_vz_down = 0.6;    // 最大下降速度 0.6 m/s
        double max_vxy = 1.0;         // 最大水平速度 1.0 m/s (提高灵敏度)
        
        // 限制垂直速度
        if (vz_ned < -max_vz_up) {    // 向上爬升（NED中负向）
            vz_ned = -max_vz_up;
        } else if (vz_ned > max_vz_down) {  // 向下下降（NED中正向）
            vz_ned = max_vz_down;
        }
        
        // 限制水平速度（XY向量限制）
        double vxy_mag = std::sqrt(vx_ned * vx_ned + vy_ned * vy_ned);
        if (vxy_mag > max_vxy) {
            double scale = max_vxy / vxy_mag;
            vx_ned *= scale;
            vy_ned *= scale;
        }
        
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

    if (!vehicle_local_position_received_) {
        RCLCPP_WARN(this->get_logger(), "⚠️ No position yet; waiting to initialize home position...");
        auto wait_start = std::chrono::system_clock::now();
        while (!vehicle_local_position_received_ && rclcpp::ok() &&
               std::chrono::duration<double>(std::chrono::system_clock::now() - wait_start).count() < 3.0) {
            std::this_thread::sleep_for(100ms);
        }
    }

    if (!imu_estimator_initialized_) {
        auto imu_wait_start = std::chrono::steady_clock::now();
        while (!imu_estimator_initialized_ && rclcpp::ok() &&
               std::chrono::duration<double>(std::chrono::steady_clock::now() - imu_wait_start).count() < 1.0) {
            std::this_thread::sleep_for(50ms);
        }
    }

    double home_x;
    double home_y;
    double home_z;
    {
        std::lock_guard<std::mutex> guard(lock_);
        if (!vehicle_local_position_received_) {
            RCLCPP_ERROR(this->get_logger(), "❌ Home position not initialized! Position data missing.");
            return false;
        }
        if (vehicle_odometry_received_ && odom_raw_initialized_) {
            odom_z_offset_ = odom_z_raw_enu_;
            odom_offset_initialized_ = true;
            odom_z_filtered_ = 0.0;
            odom_z_filtered_initialized_ = true;
            RCLCPP_INFO(this->get_logger(), "📌 ODOM Z offset reset at takeoff: %.3f m", odom_z_offset_);
        }
        const double local_x = vehicle_local_position_enu_.x;
        const double local_y = vehicle_local_position_enu_.y;
        const double local_z = vehicle_local_position_enu_.z;
        const bool local_z_valid = z_valid_;
        const bool odom_ready = vehicle_odometry_received_;
        const double odom_z = odom_ready ? vehicle_odometry_enu_.position[2] : 0.0;
        const bool odom_z_reasonable = std::abs(odom_z) < 1.0;
        const bool imu_ready = imu_estimator_initialized_ && !odom_ready;
        const double imu_z = imu_ready ? imu_z_ : 0.0;
        const bool local_z_reasonable = std::abs(local_z) < 1.0;

        home_x = local_x;
        home_y = local_y;
        if (odom_ready && odom_z_reasonable) {
            home_z = odom_z;
        } else if (imu_ready) {
            home_z = imu_z;
        } else if (local_z_valid && local_z_reasonable) {
            home_z = local_z;
        } else {
            home_z = 0.0;
            RCLCPP_WARN(this->get_logger(), "⚠️ Local Z invalid/unreasonable (z=%.3f, valid=%d); using home_z=0.0 m.",
                        local_z, local_z_valid);
        }

        home_position_ = {local_x, local_y, home_z};

        if (imu_ready && std::abs(imu_z - local_z) > 0.3) {
            RCLCPP_WARN(this->get_logger(), "⚠️ IMU height differs from local_z by %.3f m; using IMU height as home_z.",
                        imu_z - local_z);
        }
        RCLCPP_INFO(this->get_logger(), "📍 Using home_z = %.3f m", home_z);
    }
    double target_alt = home_z + takeoff_height;
    double current_heading = vehicle_local_position_received_ ? vehicle_local_position_enu_.heading : 0.0;
    const bool use_odom_height = vehicle_odometry_received_;
    const bool use_imu_height = !use_odom_height && imu_estimator_initialized_;
    if (use_odom_height) {
        RCLCPP_INFO(this->get_logger(), "🧭 Using ODOM height for takeoff control.");
    } else if (use_imu_height) {
        RCLCPP_INFO(this->get_logger(), "🧭 Using IMU height estimator for takeoff control.");
    } else {
        RCLCPP_WARN(this->get_logger(), "⚠️ IMU/ODOM not ready; using local position height.");
    }
    
    RCLCPP_INFO(this->get_logger(), "🛫 Starting smooth takeoff: home=%.3f m → target=%.3f m (+%.3f m)", 
                home_z, target_alt, takeoff_height);

    set_control_mode("velocity");

    const double safety_max_relative_alt = 3.0;
    const double omega = 1.2;
    const double zeta = 0.9;
    const double kp = 0.9;
    const double ki = 0.15;
    const double kd = 0.25;
    const double max_vz_up = 0.6;
    const double max_vz_down = 0.8;
    const double descend_boost_error = 0.10;
    const double descend_min_vz = -0.3;
    const double integral_limit = 0.6;
    const double settle_error = 0.05;
    const double settle_velocity = 0.05;
    const double settle_time = 0.8;

    double z_ref = home_z;
    double z_ref_dot = 0.0;
    double integral = 0.0;
    double last_error = 0.0;
    double stable_elapsed = 0.0;

    auto start = std::chrono::steady_clock::now();
    auto last_time = start;

    while (rclcpp::ok() && std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() < timeout) {
        auto now = std::chrono::steady_clock::now();
        double dt = std::chrono::duration<double>(now - last_time).count();
        if (dt <= 0.0) {
            dt = 0.02;
        }
        last_time = now;

        double current_z;
        {
            std::lock_guard<std::mutex> guard(lock_);
            if (use_imu_height) {
                current_z = imu_z_;
            } else if (use_odom_height) {
                current_z = vehicle_odometry_enu_.position[2];
            } else {
                current_z = vehicle_local_position_enu_.z;
            }
        }

        double relative_alt = current_z - home_z;
        if (relative_alt > safety_max_relative_alt) {
            const double local_z = vehicle_local_position_enu_.z;
            const double imu_z = imu_z_;
            const double odom_z = vehicle_odometry_enu_.position[2];
            RCLCPP_WARN(this->get_logger(),
                        "⚠️ Safety ceiling reached: relative_alt=%.3f m > %.2f m (odom=%.3f, local=%.3f, imu=%.3f, home=%.3f). Initiating LAND.",
                        relative_alt, safety_max_relative_alt, odom_z, local_z, imu_z, home_z);
            land();
            return false;
        }

        double ref_error = target_alt - z_ref;
        double z_ref_ddot = omega * omega * ref_error - 2.0 * zeta * omega * z_ref_dot;
        z_ref_dot += z_ref_ddot * dt;
        z_ref += z_ref_dot * dt;
        if (z_ref > target_alt) {
            z_ref = target_alt;
            if (z_ref_dot > 0.0) {
                z_ref_dot = 0.0;
            }
        }

        double target_error = target_alt - current_z;
        double error = z_ref - current_z;
        integral += error * dt;
        if (integral > integral_limit) {
            integral = integral_limit;
        } else if (integral < -integral_limit) {
            integral = -integral_limit;
        }
        if (target_error < 0.0) {
            integral = std::min(integral, 0.0);
        }
        double derivative = (error - last_error) / dt;
        last_error = error;

        double vz_cmd = z_ref_dot + kp * error + ki * integral + kd * derivative;
        if (vz_cmd > max_vz_up) {
            vz_cmd = max_vz_up;
        } else if (vz_cmd < -max_vz_down) {
            vz_cmd = -max_vz_down;
        }
        if (target_error < -descend_boost_error && vz_cmd > descend_min_vz) {
            vz_cmd = descend_min_vz;
        }

        update_velocity_setpoint(0.0, 0.0, vz_cmd, 0.0);

        if (std::abs(target_error) <= settle_error) {
            stable_elapsed += dt;
            if (stable_elapsed >= settle_time) {
                set_control_mode("position");
                update_position_setpoint(home_x, home_y, target_alt, current_heading);
                RCLCPP_INFO(this->get_logger(), "✅ Takeoff complete! Stable hover at %.3f m (target: %.3f m)", current_z, target_alt);
                return true;
            }
        } else {
            stable_elapsed = 0.0;
        }

        throttle_log(1.0,
            "[TAKEOFF] Alt=" + std::to_string(current_z) +
            "/" + std::to_string(target_alt) + " m, err=" + std::to_string(target_error) +
            " m, vz_cmd=" + std::to_string(vz_cmd),
            "info", "takeoff");

        std::this_thread::sleep_for(50ms);
    }

    double final_z;
    {
        std::lock_guard<std::mutex> guard(lock_);
        if (use_imu_height) {
            final_z = imu_z_;
        } else if (use_odom_height) {
            final_z = vehicle_odometry_enu_.position[2];
        } else {
            final_z = vehicle_local_position_enu_.z;
        }
    }

    RCLCPP_WARN(this->get_logger(), "⚠️ Takeoff timed out! Final altitude: %.3f m (target: %.3f m, error: %.3f m)",
                final_z, target_alt, target_alt - final_z);
    set_control_mode("position");
    update_position_setpoint(home_x, home_y, final_z, current_heading);
    return false;
}

void OffboardControl::start_takeoff_async(double takeoff_height, double timeout) {
    if (takeoff_running_) {
        RCLCPP_WARN(this->get_logger(), "⚠️ Takeoff already running!");
        return;
    }
    
    takeoff_running_ = true;
    takeoff_height_ = takeoff_height;
    
    if (takeoff_thread_.joinable()) {
        takeoff_thread_.join();
    }
    
    takeoff_thread_ = std::thread(&OffboardControl::takeoff_async_loop, this, takeoff_height, timeout);
    RCLCPP_INFO(this->get_logger(), "🚀 [ASYNC] Takeoff thread started (height=%.3f m, timeout=%.1f s)", takeoff_height, timeout);
}

void OffboardControl::takeoff_async_loop(double takeoff_height, double timeout) {
    const bool ok = takeoff(takeoff_height, timeout);
    if (!ok) {
        RCLCPP_WARN(this->get_logger(), "⚠️ [ASYNC] Takeoff failed or timed out");
    }
    takeoff_running_ = false;
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