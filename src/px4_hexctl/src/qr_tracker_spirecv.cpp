#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <sv_world.h>

#include <geometry_msgs/msg/twist.hpp>

#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <array>

using namespace std::chrono_literals;

class QRTrackerSpireCV : public rclcpp::Node {
public:
    QRTrackerSpireCV()
        : Node("qr_tracker_spirecv") {
        declare_parameter("rgb_topic", "/camera/camera/color/image_raw");
        declare_parameter("depth_topic", "/camera/camera/depth/image_rect_raw");
        declare_parameter("target_id", 1);
        declare_parameter("takeoff_alt", 1.5);
        declare_parameter("kpx_track", 0.2);
        declare_parameter("kpy_track", 0.2);
        declare_parameter("kpz_track", 0.2);
        declare_parameter("tracking_delta_x", 2.5);
        declare_parameter("tracking_delta_y", 0.0);
        declare_parameter("tracking_delta_z", 0.0);
        declare_parameter("camera_offset_x", 0.0);
        declare_parameter("camera_offset_y", 0.0);
        declare_parameter("camera_offset_z", 0.0);
        declare_parameter("vision_threshold", 10);
        declare_parameter("track_z", false);
        declare_parameter("max_vxy", 1.0);
        declare_parameter("max_vz", 0.8);
        declare_parameter("enable_adaptive_liftoff", false);
        declare_parameter("takeoff_thrust", 0.68);
        declare_parameter("spirecv_calib", std::string(""));
        declare_parameter("spirecv_algo", std::string(""));
        declare_parameter("cmd_vel_topic", "/qr_tracker/cmd_vel_body");

        rgb_topic_ = get_parameter("rgb_topic").as_string();
        depth_topic_ = get_parameter("depth_topic").as_string();
        target_id_ = get_parameter("target_id").as_int();
        takeoff_alt_ = get_parameter("takeoff_alt").as_double();
        kpx_track_ = get_parameter("kpx_track").as_double();
        kpy_track_ = get_parameter("kpy_track").as_double();
        kpz_track_ = get_parameter("kpz_track").as_double();
        tracking_delta_[0] = get_parameter("tracking_delta_x").as_double();
        tracking_delta_[1] = get_parameter("tracking_delta_y").as_double();
        tracking_delta_[2] = get_parameter("tracking_delta_z").as_double();
        camera_offset_[0] = get_parameter("camera_offset_x").as_double();
        camera_offset_[1] = get_parameter("camera_offset_y").as_double();
        camera_offset_[2] = get_parameter("camera_offset_z").as_double();
        vision_threshold_ = get_parameter("vision_threshold").as_int();
        track_z_ = get_parameter("track_z").as_bool();
        max_vxy_ = get_parameter("max_vxy").as_double();
        max_vz_ = get_parameter("max_vz").as_double();
        enable_adaptive_liftoff_ = get_parameter("enable_adaptive_liftoff").as_bool();
        takeoff_thrust_ = get_parameter("takeoff_thrust").as_double();

        spirecv_calib_ = get_parameter("spirecv_calib").as_string();
        spirecv_algo_ = get_parameter("spirecv_algo").as_string();
        cmd_vel_topic_ = get_parameter("cmd_vel_topic").as_string();

        if (spirecv_calib_.empty()) {
            spirecv_calib_ = sv::get_home() + "/SpireCV/confs/calib_webcam_1280x720.yaml";
        }
        if (spirecv_algo_.empty()) {
            spirecv_algo_ = sv::get_home() + "/SpireCV/confs/sv_algorithm_params.json";
        }

        detector_.loadCameraParams(spirecv_calib_);
        detector_.loadAlgorithmParams(spirecv_algo_);

        rgb_sub_ = create_subscription<sensor_msgs::msg::Image>(
            rgb_topic_, 10,
            std::bind(&QRTrackerSpireCV::rgb_callback, this, std::placeholders::_1));
        depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
            depth_topic_, 10,
            std::bind(&QRTrackerSpireCV::depth_callback, this, std::placeholders::_1));

        timer_ = create_wall_timer(50ms, std::bind(&QRTrackerSpireCV::control_loop, this));

        cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);

        RCLCPP_INFO(get_logger(), "🚀 SpireCV QR Tracker started.");
        RCLCPP_INFO(get_logger(), "RGB: %s", rgb_topic_.c_str());
        RCLCPP_INFO(get_logger(), "Depth: %s", depth_topic_.c_str());
    }

private:
    void depth_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            last_depth_ = cv_bridge::toCvCopy(msg)->image;
        } catch (const cv_bridge::Exception &e) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Depth cv_bridge exception: %s", e.what());
        }
    }

    void rgb_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            cv::Mat img = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
            if (detector_.image_width > 0 && detector_.image_height > 0) {
                cv::resize(img, img, cv::Size(detector_.image_width, detector_.image_height));
            }

            sv::TargetsInFrame tgts(frame_id_++);
            detector_.detect(img, tgts);

            bool found = false;
            sv::Target best_target;

            for (const auto &tgt : tgts.targets) {
                if (!tgt.mode) {
                    continue;
                }
                if (target_id_ >= 0 && tgt.tracked_id != target_id_) {
                    continue;
                }
                best_target = tgt;
                found = true;
                break;
            }

            if (found) {
                pos_body_frame_[0] = best_target.pz + camera_offset_[0];
                pos_body_frame_[1] = -best_target.px + camera_offset_[1];
                pos_body_frame_[2] = -best_target.py + camera_offset_[2];

                if (best_target.score == 1) {
                    vision_regain_++;
                    vision_lost_ = 0;
                } else {
                    vision_regain_ = 0;
                    vision_lost_++;
                }

                if (vision_regain_ > vision_threshold_) {
                    is_detected_ = true;
                }
            } else {
                vision_regain_ = 0;
                vision_lost_++;
            }

            if (vision_lost_ > vision_threshold_) {
                is_detected_ = false;
            }

        } catch (const cv_bridge::Exception &e) {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "RGB cv_bridge exception: %s", e.what());
        }
    }

    void control_loop() {
        geometry_msgs::msg::Twist cmd;
        if (!is_detected_) {
            cmd_vel_pub_->publish(cmd);
            return;
        }

        double vx = kpx_track_ * (pos_body_frame_[0] - tracking_delta_[0]);
        double vy = kpy_track_ * (pos_body_frame_[1] - tracking_delta_[1]);
        double vz = track_z_ ? (kpz_track_ * (pos_body_frame_[2] - tracking_delta_[2])) : 0.0;

        vx = std::clamp(vx, -max_vxy_, max_vxy_);
        vy = std::clamp(vy, -max_vxy_, max_vxy_);
        vz = std::clamp(vz, -max_vz_, max_vz_);

        cmd.linear.x = vx;
        cmd.linear.y = vy;
        cmd.linear.z = vz;
        cmd_vel_pub_->publish(cmd);
    }

    std::string rgb_topic_;
    std::string depth_topic_;

    int target_id_ = 1;
    double takeoff_alt_ = 1.5;
    double kpx_track_ = 0.2;
    double kpy_track_ = 0.2;
    double kpz_track_ = 0.2;
    std::array<double, 3> tracking_delta_{2.5, 0.0, 0.0};
    std::array<double, 3> camera_offset_{0.0, 0.0, 0.0};
    std::array<double, 3> pos_body_frame_{0.0, 0.0, 0.0};
    int vision_threshold_ = 10;
    bool track_z_ = false;
    double max_vxy_ = 1.0;
    double max_vz_ = 0.8;
    bool enable_adaptive_liftoff_ = false;
    double takeoff_thrust_ = 0.68;

    std::string spirecv_calib_;
    std::string spirecv_algo_;
    std::string cmd_vel_topic_;

    sv::ArucoDetector detector_;
    int frame_id_ = 0;
    int vision_lost_ = 0;
    int vision_regain_ = 0;
    bool is_detected_ = false;

    cv::Mat last_depth_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto tracker = std::make_shared<QRTrackerSpireCV>();
    rclcpp::spin(tracker);
    rclcpp::shutdown();
    return 0;
}
