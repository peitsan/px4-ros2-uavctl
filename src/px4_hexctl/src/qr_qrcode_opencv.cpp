#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#include <chrono>
#include <mutex>

using namespace std::chrono_literals;

class QrCodeOpenCvTracker : public rclcpp::Node {
public:
    QrCodeOpenCvTracker() : Node("qr_qrcode_opencv") {
        declare_parameter("image_topic", std::string("/camera"));
        declare_parameter("cmd_vel_topic", std::string("/qr_tracker/cmd_vel_body"));
        declare_parameter("qr_size_m", 0.2);
        declare_parameter("target_distance_m", 2.0);
        declare_parameter("min_distance_m", 0.6);
        declare_parameter("kp_distance", 0.5);
        declare_parameter("max_forward_speed", 0.6);
        declare_parameter("publish_debug_image", true);
        declare_parameter("debug_image_topic", std::string("/qr_tracker/debug_image"));
        declare_parameter("camera_fx", 554.0);
        declare_parameter("camera_fy", 554.0);
        declare_parameter("camera_cx", 320.0);
        declare_parameter("camera_cy", 240.0);

        image_topic_ = get_parameter("image_topic").as_string();
        cmd_vel_topic_ = get_parameter("cmd_vel_topic").as_string();
        qr_size_m_ = get_parameter("qr_size_m").as_double();
        target_distance_m_ = get_parameter("target_distance_m").as_double();
        min_distance_m_ = get_parameter("min_distance_m").as_double();
        kp_distance_ = get_parameter("kp_distance").as_double();
        max_forward_speed_ = get_parameter("max_forward_speed").as_double();
        publish_debug_image_ = get_parameter("publish_debug_image").as_bool();
        debug_image_topic_ = get_parameter("debug_image_topic").as_string();
        camera_fx_ = get_parameter("camera_fx").as_double();
        camera_fy_ = get_parameter("camera_fy").as_double();
        camera_cx_ = get_parameter("camera_cx").as_double();
        camera_cy_ = get_parameter("camera_cy").as_double();

        camera_matrix_ = (cv::Mat1d(3, 3) <<
            camera_fx_, 0.0, camera_cx_,
            0.0, camera_fy_, camera_cy_,
            0.0, 0.0, 1.0);
        dist_coeffs_ = cv::Mat::zeros(1, 5, CV_64F);

        image_sub_ = create_subscription<sensor_msgs::msg::Image>(
            image_topic_, 10, std::bind(&QrCodeOpenCvTracker::image_callback, this, std::placeholders::_1));

        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
        if (publish_debug_image_) {
            debug_pub_ = create_publisher<sensor_msgs::msg::Image>(debug_image_topic_, 10);
        }
        timer_ = create_wall_timer(50ms, std::bind(&QrCodeOpenCvTracker::control_loop, this));

        RCLCPP_INFO(get_logger(), "✅ OpenCV QR tracker started. image_topic=%s, cmd_vel_topic=%s",
            image_topic_.c_str(), cmd_vel_topic_.c_str());
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            cv::Mat frame = cv_ptr->image;

            std::vector<int> ids;
            std::vector<std::vector<cv::Point2f>> corners;
            cv::aruco::detectMarkers(frame, dictionary_, corners, ids, detector_params_);

            if (!ids.empty()) {
                std::vector<cv::Vec3d> rvecs, tvecs;
                cv::aruco::estimatePoseSingleMarkers(corners, qr_size_m_, camera_matrix_, dist_coeffs_, rvecs, tvecs);

                size_t best_idx = 0;
                double best_dist = tvecs[0][2];
                for (size_t i = 1; i < tvecs.size(); ++i) {
                    if (tvecs[i][2] < best_dist) {
                        best_dist = tvecs[i][2];
                        best_idx = i;
                    }
                }

                {
                    std::lock_guard<std::mutex> guard(state_mutex_);
                    last_distance_m_ = best_dist;
                    last_seen_time_ = now();
                    has_detection_ = true;
                }

                if (publish_debug_image_) {
                    cv::aruco::drawDetectedMarkers(frame, corners, ids);
                    cv::aruco::drawAxis(frame, camera_matrix_, dist_coeffs_, rvecs[best_idx], tvecs[best_idx], qr_size_m_ * 0.5);
                }
            }

            if (publish_debug_image_ && debug_pub_) {
                auto out_msg = cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::BGR8, frame).toImageMsg();
                debug_pub_->publish(*out_msg);
            }
        } catch (const cv_bridge::Exception &e) {
            RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    void control_loop() {
        geometry_msgs::msg::Twist cmd{};
        bool publish = false;

        {
            std::lock_guard<std::mutex> guard(state_mutex_);
            if (has_detection_ && (now() - last_seen_time_).seconds() < 0.5) {
                double distance = last_distance_m_;
                double error = distance - target_distance_m_;
                double vx = kp_distance_ * error;

                if (distance < min_distance_m_) {
                    vx = -std::abs(max_forward_speed_);
                }

                vx = std::max(-max_forward_speed_, std::min(max_forward_speed_, vx));
                cmd.linear.x = vx;
                publish = true;
            }
        }

        if (!publish) {
            cmd.linear.x = 0.0;
        }

        cmd_pub_->publish(cmd);
    }

    std::string image_topic_;
    std::string cmd_vel_topic_;
    double qr_size_m_ = 0.2;
    double target_distance_m_ = 2.0;
    double min_distance_m_ = 0.6;
    double kp_distance_ = 0.5;
    double max_forward_speed_ = 0.6;
    bool publish_debug_image_ = true;
    std::string debug_image_topic_ = "/qr_tracker/debug_image";
    double camera_fx_ = 554.0;
    double camera_fy_ = 554.0;
    double camera_cx_ = 320.0;
    double camera_cy_ = 240.0;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    cv::Ptr<cv::aruco::Dictionary> dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11);
    cv::Ptr<cv::aruco::DetectorParameters> detector_params_ = cv::aruco::DetectorParameters::create();
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    std::mutex state_mutex_;
    rclcpp::Time last_seen_time_{0, 0, RCL_ROS_TIME};
    double last_distance_m_ = 0.0;
    bool has_detection_ = false;
    std::string last_text_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<QrCodeOpenCvTracker>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}