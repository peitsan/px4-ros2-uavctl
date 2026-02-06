#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;
using px4_msgs::msg::OffboardControlMode;
using px4_msgs::msg::TrajectorySetpoint;
using px4_msgs::msg::VehicleCommand;

class OffboardFigure8 : public rclcpp::Node {
public:
  OffboardFigure8() : Node("offboard_figure8"), counter_(0), start_time_(this->get_clock()->now())
  {
    radius_ = declare_parameter("radius_m", 1.5);
    period_ = declare_parameter("period_s", 20.0);
    altitude_ = declare_parameter("altitude_m", 1.0);
    yaw_deg_ = declare_parameter("yaw_deg", 0.0);

    offboard_mode_pub_ = create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    traj_pub_ = create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
    cmd_pub_ = create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

    timer_ = create_wall_timer(100ms, std::bind(&OffboardFigure8::on_timer, this));

    RCLCPP_INFO(get_logger(), "Offboard Figure-8 controller started");
  }

private:
  void publish_offboard_mode() {
    OffboardControlMode msg{};
    msg.position = true;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    offboard_mode_pub_->publish(msg);
  }

  void publish_trajectory() {
    double t = (this->get_clock()->now() - start_time_).seconds();
    double A = radius_;
    double omega = 2 * M_PI / period_;
    float x = A * std::sin(omega * t);
    float y = (A / 2.0) * std::sin(2 * omega * t);
    float z = -altitude_;

    TrajectorySetpoint sp{};
    sp.position = {x, y, z};
    sp.yaw = yaw_deg_ * M_PI / 180.0;
    sp.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    traj_pub_->publish(sp);
  }

  void publish_vehicle_cmd(uint16_t command, float p1=0.0f, float p2=0.0f) {
    VehicleCommand cmd{};
    cmd.param1 = p1;
    cmd.param2 = p2;
    cmd.command = command;
    cmd.target_system = 1;
    cmd.target_component = 1;
    cmd.source_system = 1;
    cmd.source_component = 1;
    cmd.from_external = true;
    cmd.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    cmd_pub_->publish(cmd);
  }

  void arm() {
    publish_vehicle_cmd(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
    RCLCPP_INFO(get_logger(), "Arm command sent");
  }

  void on_timer() {
    if (counter_ == 10) {
      publish_vehicle_cmd(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
      arm();
    }
    publish_offboard_mode();
    publish_trajectory();
    if (counter_ < 11) counter_++;
  }

  rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_mode_pub_;
  rclcpp::Publisher<TrajectorySetpoint>::SharedPtr traj_pub_;
  rclcpp::Publisher<VehicleCommand>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  uint64_t counter_;
  rclcpp::Time start_time_;
  double radius_, period_, altitude_, yaw_deg_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OffboardFigure8>());
  rclcpp::shutdown();
  return 0;
}
