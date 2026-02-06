#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <cmath>
#include <chrono>

using namespace std::chrono_literals;

class Figure8Visualizer : public rclcpp::Node {
public:
  Figure8Visualizer() : Node("figure8_visualizer"), start_time_(this->get_clock()->now())
  {
    radius_ = declare_parameter("radius_m", 3.0);
    period_ = declare_parameter("period_s", 20.0);
    altitude_ = declare_parameter("altitude_m", 2.0);

    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/figure8/drone_pose", 10);
    path_pub_ = create_publisher<nav_msgs::msg::Path>("/figure8/trajectory_path", 10);
    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("/figure8/target_point", 10);
    path_.header.frame_id = "map";

    timer_ = create_wall_timer(100ms, std::bind(&Figure8Visualizer::on_timer, this));
    RCLCPP_INFO(get_logger(), "Figure-8 Visualizer started");
  }

private:
  void on_timer() {
    auto now = this->get_clock()->now();
    double t = (now - start_time_).seconds();
    double A = radius_;
    double omega = 2 * M_PI / period_;
    float x = A * std::sin(omega * t);
    float y = (A / 2.0) * std::sin(2 * omega * t);
    float z = altitude_;

    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = now;
    pose.header.frame_id = "map";
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = z;
    pose.pose.orientation.w = 1.0;
    pose_pub_->publish(pose);

    path_.header.stamp = now;
    path_.poses.push_back(pose);
    path_pub_->publish(path_);

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = now;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;
    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.3;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.2;
    marker.color.b = 0.2;
    marker_pub_->publish(marker);
  }

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  nav_msgs::msg::Path path_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time start_time_;
  double radius_, period_, altitude_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Figure8Visualizer>());
  rclcpp::shutdown();
  return 0;
}
