#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <algorithm>
#include <string>

using namespace std::chrono_literals;

class SmsClickTargetBridge : public rclcpp::Node {
public:
    SmsClickTargetBridge()
        : Node("sms_click_target_bridge") {
        declare_parameter("target_x", 0.0);
        declare_parameter("target_y", 0.0);
        declare_parameter("target_z", 0.0);
        declare_parameter("target_valid", false);
        declare_parameter("frame_id", std::string("base_link"));
        declare_parameter("publish_rate_hz", 20.0);
        declare_parameter("target_topic", std::string("/qr_tracker/expected_position"));

        declare_parameter("publish_cmd_vel", true);
        declare_parameter("cmd_vel_topic", std::string("/qr_tracker/cmd_vel_body"));
        declare_parameter("kpx_track", 0.2);
        declare_parameter("kpy_track", 0.2);
        declare_parameter("kpz_track", 0.2);
        declare_parameter("tracking_delta_x", 2.0);
        declare_parameter("tracking_delta_y", 0.0);
        declare_parameter("tracking_delta_z", 0.0);
        declare_parameter("track_z", false);
        declare_parameter("max_vxy", 1.0);
        declare_parameter("max_vz", 0.8);

        load_parameters();

        param_cb_handle_ = add_on_set_parameters_callback(
            std::bind(&SmsClickTargetBridge::on_param_change, this, std::placeholders::_1));

        target_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(target_topic_, 10);
        cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);

        auto period = std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_hz_));
        timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::bind(&SmsClickTargetBridge::on_timer, this));

        RCLCPP_INFO(get_logger(), "SMS click target bridge started.");
        RCLCPP_INFO(get_logger(), "target_topic: %s", target_topic_.c_str());
        RCLCPP_INFO(get_logger(), "cmd_vel_topic: %s", cmd_vel_topic_.c_str());
    }

private:
    void load_parameters() {
        target_x_ = get_parameter("target_x").as_double();
        target_y_ = get_parameter("target_y").as_double();
        target_z_ = get_parameter("target_z").as_double();
        target_valid_ = get_parameter("target_valid").as_bool();
        frame_id_ = get_parameter("frame_id").as_string();
        publish_rate_hz_ = get_parameter("publish_rate_hz").as_double();
        target_topic_ = get_parameter("target_topic").as_string();

        publish_cmd_vel_ = get_parameter("publish_cmd_vel").as_bool();
        cmd_vel_topic_ = get_parameter("cmd_vel_topic").as_string();
        kpx_track_ = get_parameter("kpx_track").as_double();
        kpy_track_ = get_parameter("kpy_track").as_double();
        kpz_track_ = get_parameter("kpz_track").as_double();
        tracking_delta_x_ = get_parameter("tracking_delta_x").as_double();
        tracking_delta_y_ = get_parameter("tracking_delta_y").as_double();
        tracking_delta_z_ = get_parameter("tracking_delta_z").as_double();
        track_z_ = get_parameter("track_z").as_bool();
        max_vxy_ = get_parameter("max_vxy").as_double();
        max_vz_ = get_parameter("max_vz").as_double();
    }

    rcl_interfaces::msg::SetParametersResult on_param_change(
        const std::vector<rclcpp::Parameter> &params) {
        for (const auto &param : params) {
            if (param.get_name() == "target_x") target_x_ = param.as_double();
            else if (param.get_name() == "target_y") target_y_ = param.as_double();
            else if (param.get_name() == "target_z") target_z_ = param.as_double();
            else if (param.get_name() == "target_valid") target_valid_ = param.as_bool();
            else if (param.get_name() == "frame_id") frame_id_ = param.as_string();
            else if (param.get_name() == "publish_rate_hz") publish_rate_hz_ = param.as_double();
            else if (param.get_name() == "target_topic") target_topic_ = param.as_string();
            else if (param.get_name() == "publish_cmd_vel") publish_cmd_vel_ = param.as_bool();
            else if (param.get_name() == "cmd_vel_topic") cmd_vel_topic_ = param.as_string();
            else if (param.get_name() == "kpx_track") kpx_track_ = param.as_double();
            else if (param.get_name() == "kpy_track") kpy_track_ = param.as_double();
            else if (param.get_name() == "kpz_track") kpz_track_ = param.as_double();
            else if (param.get_name() == "tracking_delta_x") tracking_delta_x_ = param.as_double();
            else if (param.get_name() == "tracking_delta_y") tracking_delta_y_ = param.as_double();
            else if (param.get_name() == "tracking_delta_z") tracking_delta_z_ = param.as_double();
            else if (param.get_name() == "track_z") track_z_ = param.as_bool();
            else if (param.get_name() == "max_vxy") max_vxy_ = param.as_double();
            else if (param.get_name() == "max_vz") max_vz_ = param.as_double();
        }

        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "ok";
        return result;
    }

    void on_timer() {
        if (!target_valid_) {
            if (publish_cmd_vel_) {
                geometry_msgs::msg::Twist stop;
                cmd_vel_pub_->publish(stop);
            }
            return;
        }

        geometry_msgs::msg::PointStamped target_msg;
        target_msg.header.stamp = now();
        target_msg.header.frame_id = frame_id_;
        target_msg.point.x = target_x_;
        target_msg.point.y = target_y_;
        target_msg.point.z = target_z_;
        target_pub_->publish(target_msg);

        if (!publish_cmd_vel_) {
            return;
        }

        double vx = kpx_track_ * (target_x_ - tracking_delta_x_);
        double vy = kpy_track_ * (target_y_ - tracking_delta_y_);
        double vz = track_z_ ? (kpz_track_ * (target_z_ - tracking_delta_z_)) : 0.0;

        vx = std::clamp(vx, -max_vxy_, max_vxy_);
        vy = std::clamp(vy, -max_vxy_, max_vxy_);
        vz = std::clamp(vz, -max_vz_, max_vz_);

        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = vx;
        cmd.linear.y = vy;
        cmd.linear.z = vz;
        cmd_vel_pub_->publish(cmd);
    }

    double target_x_ = 0.0;
    double target_y_ = 0.0;
    double target_z_ = 0.0;
    bool target_valid_ = false;
    std::string frame_id_ = "base_link";
    double publish_rate_hz_ = 20.0;
    std::string target_topic_ = "/qr_tracker/expected_position";

    bool publish_cmd_vel_ = true;
    std::string cmd_vel_topic_ = "/qr_tracker/cmd_vel_body";
    double kpx_track_ = 0.2;
    double kpy_track_ = 0.2;
    double kpz_track_ = 0.2;
    double tracking_delta_x_ = 2.0;
    double tracking_delta_y_ = 0.0;
    double tracking_delta_z_ = 0.0;
    bool track_z_ = false;
    double max_vxy_ = 1.0;
    double max_vz_ = 0.8;

    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr target_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SmsClickTargetBridge>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
