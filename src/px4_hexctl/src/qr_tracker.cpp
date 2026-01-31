#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include "px4_hexctl/offboard_control.hpp"
#include "px4_hexctl/vehicle.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <vector>

using namespace std::chrono_literals;

class QRTracker : public rclcpp::Node {
public:
    QRTracker() : Node("qr_tracker") {
        // Parameters
        this->declare_parameter("target_id", 0);
        this->declare_parameter("altitude", 2.0);
        this->declare_parameter("kp_xy", 0.5);
        this->declare_parameter("kp_yaw", 0.3);
        
        target_id_ = this->get_parameter("target_id").as_int();
        target_alt_ = this->get_parameter("altitude").as_double();
        kp_xy_ = this->get_parameter("kp_xy").as_double();
        kp_yaw_ = this->get_parameter("kp_yaw").as_double();

        // Subscriptions
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10, std::bind(&QRTracker::image_callback, this, std::placeholders::_1));

        // ArUco Dictionary
        dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
        parameters_ = cv::aruco::DetectorParameters::create();

        // Timer for Control Loop (20Hz)
        timer_ = this->create_wall_timer(50ms, std::bind(&QRTracker::control_loop, this));

        RCLCPP_INFO(this->get_logger(), "🚀 QR Tracker Node started. Target ArUco ID: %d", target_id_);
    }

    void set_drone(std::shared_ptr<OffboardControl> drone) {
        drone_ = drone;
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            cv::Mat frame = cv_ptr->image;

            std::vector<int> ids;
            std::vector<std::vector<cv::Point2f>> corners;
            cv::aruco::detectMarkers(frame, dictionary_, corners, ids, parameters_);

            bool found = false;
            for (size_t i = 0; i < ids.size(); i++) {
                if (ids[i] == target_id_) {
                    cv::Point2f center(0, 0);
                    for (auto p : corners[i]) center += p;
                    center *= 0.25f;

                    // Image center
                    float img_center_x = frame.cols / 2.0f;
                    float img_center_y = frame.rows / 2.0f;

                    // Compute error (normalized -1 to 1)
                    error_x_ = (center.x - img_center_x) / img_center_x;
                    error_y_ = (center.y - img_center_y) / img_center_y;
                    
                    target_visible_ = true;
                    last_detection_time_ = this->now();
                    found = true;

                    // Draw for visualization
                    cv::aruco::drawDetectedMarkers(frame, corners, ids);
                    cv::circle(frame, center, 5, cv::Scalar(0, 255, 0), -1);
                    break;
                }
            }

            if (!found) {
                if ((this->now() - last_detection_time_).seconds() > 1.0) {
                    target_visible_ = false;
                }
            }

            // (Optional) Show image
            // cv::imshow("QR Tracker Feedback", frame);
            // cv::waitKey(1);

        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    void control_loop() {
        if (!drone_) return;

        auto status = drone_->get_vehicle_status();
        if (status.nav_state != 14 || status.arming_state != 2) {
            return;
        }

        if (target_visible_) {
            // Forward-looking camera logic
            // error_x (horizontal): Target is left/right of center -> use Yaw or Y strafe
            // error_y (vertical): Target is above/below center -> use Altitude (Z)
            
            double vy = -error_x_ * kp_xy_; // Horizontal error -> Y velocity
            double vz = -error_y_ * 1.5;    // Vertical error -> Z velocity (height correction)
            
            // Forward movement (keep a "distance" - here simplified to fixed speed if visible)
            // Or we could use the size of the ArUco marker to estimate distance
            double vx = 0.5; // Slowly move forward while visible
            
            // Limit velocities
            vx = std::clamp(vx, -0.5, 1.0);
            vy = std::clamp(vy, -1.0, 1.0);
            vz = std::clamp(vz, -1.0, 1.0);

            drone_->update_velocity_setpoint(vx, vy, vz, 0.0);
            
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                "🎯 Tracking: vx=%.2f, vy=%.2f, vz=%.2f", vx, vy, vz);
        } else {
            // Hover or rotate to find target
            drone_->update_velocity_setpoint(0.0, 0.0, 0.0, 0.2); // Rotate slowly to find
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    cv::Ptr<cv::aruco::DetectorParameters> parameters_;

    std::shared_ptr<OffboardControl> drone_;
    
    int target_id_;
    double target_alt_;
    double kp_xy_, kp_yaw_;

    bool target_visible_ = false;
    double error_x_ = 0.0;
    double error_y_ = 0.0;
    rclcpp::Time last_detection_time_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    
    // Create the tracker node
    auto tracker = std::make_shared<QRTracker>();
    
    // Create the vehicle/drone node (using existing library)
    auto vehicle = std::make_shared<Vehicle>();
    tracker->set_drone(vehicle->drone());

    // Use a multi-threaded executor to run both nodes
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(tracker);
    executor.add_node(vehicle->drone()); // The drone() method returns the OffboardControl node

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting QR Tracker multi-threaded executor...");
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}
