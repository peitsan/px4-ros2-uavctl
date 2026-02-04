#pragma once

#include <rclcpp/rclcpp.hpp>
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

#include <chrono>
#include <thread>
#include <mutex>
#include <unordered_map>
#include <cmath>
#include <array>
#include <vector>
#include <string>
#include <atomic>

constexpr double DISTANCE_TOLERANCE = 0.5;
constexpr double YAW_TOLERANCE = 0.1;

class OffboardControl : public rclcpp::Node {
public:
    OffboardControl();
    OffboardControl(const std::string& prefix, const std::string& node_name);
    ~OffboardControl();

    // Core API
    void heartbeat_thread_start();
    void publish_current_setpoint();

    void set_control_mode(const std::string &mode);
    void update_position_setpoint(double x, double y, double z, double yaw);
    void update_velocity_setpoint(double vx, double vy, double vz, double yawspeed);
    void update_attitude_setpoint(double roll, double pitch, double yaw, double thrust);

    // Vehicle commands
    void arm();
    void disarm();
    void engage_offboard_mode(int prewarm_count = 10, double prewarm_timeout = 5.0);
    bool takeoff(double takeoff_height = 2.0, double timeout = 20.0);
    void start_takeoff_async(double takeoff_height = 2.0, double timeout = 20.0);
    bool hover(double duration, double timeout = -1.0);
    bool land(double latitude = NAN, double longitude = NAN, double altitude = 0.0,
              double yaw = NAN, double abort_alt = 0.0, int land_mode = 0, double timeout = 60.0);
    void stop_heartbeat();
    bool simulated_land(double descent_rate, double ground_tolerance, double timeout);
    bool fly_to_trajectory_setpoint(double x, double y, double z, double yaw, double timeout);
    
    void reset_home() {
        std::lock_guard<std::mutex> guard(lock_);
        if (vehicle_local_position_received_) {
            home_position_ = {vehicle_local_position_enu_.x, vehicle_local_position_enu_.y, vehicle_local_position_enu_.z};
            RCLCPP_INFO(this->get_logger(), "🏠 [HOME] Reset to current: (%f, %f, %f)", home_position_[0], home_position_[1], home_position_[2]);
        }
    }

    bool takeoff_command_global(double altitude_m,
                                double pitch_deg = 15.0,
                                double yaw_deg = NAN,
                                double latitude = NAN,
                                double longitude = NAN,
                                double timeout_s = 20.0);

    bool takeoff_command_local(double z_rel_m,
                            double ascend_rate_m_s = 1.0,
                            double pitch_rad = 0.00,
                            double yaw_rad = NAN,
                            double x_m = NAN,
                            double y_m = NAN,
                            double timeout_s = 20.0);

    const px4_msgs::msg::VehicleStatus& get_vehicle_status() const { return vehicle_status_; }
    const px4_msgs::msg::VehicleLocalPosition& get_local_position() const { return vehicle_local_position_enu_; }

    /**
     * @brief Check if position data is valid (checked by EKF)
     */
    bool is_position_valid() const { 
        return (vehicle_local_position_enu_.timestamp > 0 && xy_valid_ && z_valid_); 
    }

    /**
     * @brief Check if XY position is valid
     */
    bool is_xy_valid() const {
        return (vehicle_local_position_enu_.timestamp > 0 && xy_valid_);
    }

    /**
     * @brief Check if Z position (height) is valid
     */
    bool is_z_valid() const {
        return (vehicle_local_position_enu_.timestamp > 0 && z_valid_);
    }

    void publish_vehicle_command(uint16_t command, double param1 = 0.0, double param2 = 0.0,
                                 double param3 = 0.0, double param4 = 0.0, double param5 = 0.0,
                                 double param6 = 0.0, double param7 = 0.0);

private:
    bool xy_valid_ = false;
    bool z_valid_ = false;
    // ...existing code...
    void throttle_log(double interval_sec, const std::string &msg, const std::string &level = "info", const std::string &tag = "default");
    void heartbeat_loop();
    void takeoff_async_loop(double takeoff_height, double timeout);
    void publish_offboard_control_heartbeat_signal(const std::string &control_mode);
    void publish_trajectory_setpoint(std::vector<double> position = {},
                                     std::vector<double> velocity = {},
                                     std::vector<double> acceleration = {},
                                     std::vector<double> jerk = {},
                                     double yaw = NAN,
                                     double yawspeed = NAN);
    void publish_attitude_setpoint(const std::vector<double> &q_d, const std::vector<double> &thrust_body, double yaw_sp_move_rate = NAN);

    // Conversions
    std::array<double, 3> ned_to_enu(double x_ned, double y_ned, double z_ned);
    std::array<double, 3> enu_to_ned(double x_enu, double y_enu, double z_enu);
    double normalize_yaw(double yaw_diff);
    std::vector<double> euler_to_quaternion(double roll, double pitch, double yaw);

    // Callbacks
    void vehicle_local_position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
    void vehicle_status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
    void vehicle_attitude_callback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg);
    void vehicle_imu_callback(const px4_msgs::msg::VehicleImu::SharedPtr msg);
    void vehicle_odometry_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);
    void reset_imu_estimator(double z0);

    // Publishers
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<px4_msgs::msg::GotoSetpoint>::SharedPtr goto_setpoint_publisher_;
    rclcpp::Publisher<px4_msgs::msg::VehicleAttitudeSetpoint>::SharedPtr vehicle_attitude_setpoint_publisher_;

    // Subscribers
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr vehicle_local_position_subscriber_;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_subscriber_;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_alt_subscriber_;
    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr vehicle_attitude_subscriber_;
    rclcpp::Subscription<px4_msgs::msg::VehicleImu>::SharedPtr vehicle_imu_subscriber_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odometry_subscriber_;

    // Internal state
    std::string namespace_;
    std::string control_mode_;
    std::array<double, 4> target_;
    std::array<double, 3> home_position_;
    px4_msgs::msg::VehicleLocalPosition vehicle_local_position_enu_;
    px4_msgs::msg::VehicleStatus vehicle_status_;
    px4_msgs::msg::VehicleAttitude vehicle_attitude_;
    px4_msgs::msg::VehicleOdometry vehicle_odometry_enu_;
    bool vehicle_local_position_received_;
    bool vehicle_odometry_received_ = false;
    bool odom_offset_initialized_ = false;
    double odom_z_offset_ = 0.0;
    bool odom_z_filtered_initialized_ = false;
    double odom_z_filtered_ = 0.0;
    bool odom_raw_initialized_ = false;
    double odom_z_raw_enu_ = 0.0;
    bool vehicle_attitude_received_ = false;
    bool is_takeoff_complete_;
    bool target_reached_;
    double takeoff_height_;
    int offboard_setpoint_counter_;

    // IMU-based vertical estimator (ENU z)
    bool imu_estimator_initialized_ = false;
    double imu_z_ = 0.0;
    double imu_vz_ = 0.0;
    double imu_bias_z_ = 0.0;
    uint64_t imu_last_timestamp_ = 0;
    
    // Takeoff async state
    std::thread takeoff_thread_;
    std::atomic<bool> takeoff_running_{false};

    // Flight controller status monitoring
    std::atomic<bool> fmu_status_error_{false};
    std::string last_fmu_error_message_;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr fmu_status_subscriber_;

    // Heartbeat
    std::thread heartbeat_thread_;
    std::atomic<bool> stop_heartbeat_{false};
    int heartbeat_hz_;

    // Logging and thread safety
    std::unordered_map<std::string, double> last_log_;
    std::mutex lock_;
};
