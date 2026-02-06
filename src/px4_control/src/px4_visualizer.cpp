#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>

class PX4Visualizer : public rclcpp::Node
{
public:
    PX4Visualizer() : Node("px4_visualizer")
    {
        // QoS
        rclcpp::QoS qos(10);
        qos.best_effort();

        // Subscribers
        attitude_sub_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>(
            "/fmu/out/vehicle_attitude", qos,
            std::bind(&PX4Visualizer::attitude_callback, this, std::placeholders::_1));

        local_pos_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position", qos,
            std::bind(&PX4Visualizer::local_position_callback, this, std::placeholders::_1));

        setpoint_sub_ = this->create_subscription<px4_msgs::msg::TrajectorySetpoint>(
            "/fmu/in/trajectory_setpoint", qos,
            std::bind(&PX4Visualizer::setpoint_callback, this, std::placeholders::_1));

        // Publishers
        vehicle_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/px4_visualizer/vehicle_pose", 10);

        rclcpp::QoS marker_qos(10);
        marker_qos.transient_local();
        vehicle_vel_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/px4_visualizer/vehicle_velocity", marker_qos);

        vehicle_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/px4_visualizer/vehicle_path", 10);
        setpoint_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/px4_visualizer/setpoint_path", 10);

        vehicle_path_.header.frame_id = "map";
        setpoint_path_.header.frame_id = "map";

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&PX4Visualizer::timer_callback, this));
        
        RCLCPP_INFO(this->get_logger(), "PX4 Visualizer started");
    }

private:
    void attitude_callback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg)
    {
        // NED->ENU conversion
        attitude_[0] = msg->q[0];
        attitude_[1] = msg->q[1];
        attitude_[2] = -msg->q[2];
        attitude_[3] = -msg->q[3];
    }

    void local_position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
    {
        // NED->ENU conversion
        position_[0] = msg->x;
        position_[1] = -msg->y;
        position_[2] = -msg->z;

        velocity_[0] = msg->vx;
        velocity_[1] = -msg->vy;
        velocity_[2] = -msg->vz;
    }

    void setpoint_callback(const px4_msgs::msg::TrajectorySetpoint::SharedPtr msg)
    {
        setpoint_[0] = msg->position[0];
        setpoint_[1] = -msg->position[1];
        setpoint_[2] = -msg->position[2];
    }

    visualization_msgs::msg::Marker create_arrow_marker(int id, const std::array<float, 3> &tail, const std::array<float, 3> &vec)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();
        marker.ns = "arrow";
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.scale.x = 0.1;
        marker.scale.y = 0.2;
        marker.scale.z = 0.0;

        marker.color.r = 0.5;
        marker.color.g = 0.5;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        geometry_msgs::msg::Point tail_point;
        tail_point.x = tail[0];
        tail_point.y = tail[1];
        tail_point.z = tail[2];

        geometry_msgs::msg::Point head_point;
        double dt = 0.3;
        head_point.x = tail[0] + dt * vec[0];
        head_point.y = tail[1] + dt * vec[1];
        head_point.z = tail[2] + dt * vec[2];

        marker.points.push_back(tail_point);
        marker.points.push_back(head_point);

        return marker;
    }

    geometry_msgs::msg::PoseStamped vector2PoseMsg(const std::array<float,3> &pos, const std::array<float,4> &att)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = this->now();
        pose.header.frame_id = "map";

        pose.pose.position.x = pos[0];
        pose.pose.position.y = pos[1];
        pose.pose.position.z = pos[2];

        pose.pose.orientation.w = att[0];
        pose.pose.orientation.x = att[1];
        pose.pose.orientation.y = att[2];
        pose.pose.orientation.z = att[3];

        return pose;
    }

    void timer_callback()
    {
        // Vehicle Pose
        auto pose_msg = vector2PoseMsg(position_, attitude_);
        vehicle_pose_pub_->publish(pose_msg);

        // Vehicle Path
        vehicle_path_.header.stamp = this->now();
        vehicle_path_.poses.push_back(pose_msg);
        vehicle_path_pub_->publish(vehicle_path_);

        // Setpoint Path
        auto sp_pose = vector2PoseMsg(setpoint_, attitude_);
        setpoint_path_.header.stamp = this->now();
        setpoint_path_.poses.push_back(sp_pose);
        setpoint_path_pub_->publish(setpoint_path_);

        // Velocity Marker
        auto vel_marker = create_arrow_marker(1, position_, velocity_);
        vehicle_vel_pub_->publish(vel_marker);
    }

    // Subscribers
    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_pos_sub_;
    rclcpp::Subscription<px4_msgs::msg::TrajectorySetpoint>::SharedPtr setpoint_sub_;

    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr vehicle_pose_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr vehicle_vel_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr vehicle_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr setpoint_path_pub_;

    // State
    std::array<float,3> position_{0.0,0.0,0.0};
    std::array<float,3> velocity_{0.0,0.0,0.0};
    std::array<float,3> setpoint_{0.0,0.0,0.0};
    std::array<float,4> attitude_{1.0,0.0,0.0,0.0};

    nav_msgs::msg::Path vehicle_path_;
    nav_msgs::msg::Path setpoint_path_;

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PX4Visualizer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

