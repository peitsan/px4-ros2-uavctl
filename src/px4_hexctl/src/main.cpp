#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <Eigen/Dense>
#include <chrono>
#include <thread>
#include <iostream>

using namespace std::chrono_literals;

class TrajOffboard : public rclcpp::Node {
public:
    TrajOffboard() : Node("traj_offboard"), offboard_setpoint_counter_(0) {
        // 配置 QoS 以确保与 PX4 兼容 (Best Effort)
        rclcpp::QoS qos_profile(1);
        qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
        qos_profile.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
        qos_profile.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);

        offboard_control_mode_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", qos_profile);
        traj_setpoint_pub_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", qos_profile);
        vehicle_cmd_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", qos_profile);

        // 设置起点和终点 (NED 坐标系: x北, y东, z下)
        start_p_ = Eigen::Vector3d(-0.58, -0.08, -1.29);
        end_p_   = Eigen::Vector3d( 1.2 , -0.08, -1.29);

        speed_ = 0.5; 
        double dist = (end_p_ - start_p_).norm();
        total_time_ = (speed_ > 1e-3) ? dist / speed_ : 5.0;

        // 使用 50Hz 计时器 (20ms) 以满足 PX4 高频控制需求
        timer_ = create_wall_timer(20ms, std::bind(&TrajOffboard::onTimer, this));
        RCLCPP_INFO(get_logger(), "🚀 PX4 TrajOffboard Initialized (Straight Line Mission)");
    }

private:
    void onTimer() {
        // 1. 始终发布心跳 (Heartbeat)
        publish_offboard_control_mode();

        // 2. 预热阶段 (Pre-warm): 必须先发 Setpoint，PX4 才允许切 Offboard
        if (offboard_setpoint_counter_ < 20) {
            publish_trajectory_setpoint(start_p_);
            offboard_setpoint_counter_++;
            return;
        }

        // 3. 切换 Offboard 模式
        if (!offboard_enabled_) {
            set_offboard_mode();
            offboard_enabled_ = true;
            return;
        }

        // 4. 解锁 (Arm)
        if (!armed_) {
            arm_vehicle();
            armed_ = true;
            return;
        }

        // 5. 任务逻辑
        if (!mission_started_) {
            mission_start_time_ = now();
            mission_started_ = true;
            RCLCPP_INFO(get_logger(), "✅ Mission started: Flying straight line.");
        }

        double t = (now() - mission_start_time_).seconds();
        double s = (total_time_ > 1e-6) ? (t / total_time_) : 1.0;
        if (s > 1.0) s = 1.0;
        if (s < 0.0) s = 0.0;

        Eigen::Vector3d pos = start_p_ + s * (end_p_ - start_p_);
        publish_trajectory_setpoint(pos);
    }

    void publish_offboard_control_mode() {
        px4_msgs::msg::OffboardControlMode msg{};
        msg.position = true;
        msg.velocity = false;
        msg.acceleration = false;
        msg.attitude = false;
        msg.body_rate = false;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        offboard_control_mode_pub_->publish(msg);
    }

    void publish_trajectory_setpoint(const Eigen::Vector3d &pos) {
        px4_msgs::msg::TrajectorySetpoint msg{};
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        msg.position = {static_cast<float>(pos.x()), static_cast<float>(pos.y()), static_cast<float>(pos.z())};
        msg.yaw = 0.0f;
        traj_setpoint_pub_->publish(msg);
    }

    void arm_vehicle() {
        publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
        RCLCPP_INFO(get_logger(), "🔓 Arming...");
    }

    void set_offboard_mode() {
        publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
        RCLCPP_INFO(get_logger(), "🔄 Switching to Offboard mode...");
    }

    void publish_vehicle_command(uint16_t command, float p1 = 0.0f, float p2 = 0.0f) {
        px4_msgs::msg::VehicleCommand cmd{};
        cmd.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        cmd.command = command;
        cmd.param1 = p1;
        cmd.param2 = p2;
        cmd.target_system = 1;
        cmd.target_component = 1;
        cmd.source_system = 1;
        cmd.source_component = 1;
        cmd.from_external = true;
        vehicle_cmd_pub_->publish(cmd);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr traj_setpoint_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_cmd_pub_;

    uint64_t offboard_setpoint_counter_;
    bool armed_ = false;
    bool offboard_enabled_ = false;
    bool mission_started_ = false;

    Eigen::Vector3d start_p_, end_p_;
    rclcpp::Time mission_start_time_{0, 0, RCL_ROS_TIME};
    double speed_ = 0.5;
    double total_time_ = 5.0;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajOffboard>());
    rclcpp::shutdown();
    return 0;
}
