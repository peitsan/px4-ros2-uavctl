#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#ifdef HAS_IMAGE_VIEW_MSGS
#include <image_view_msgs/msg/mouse_event.hpp>
#endif
#include <geometry_msgs/msg/point_stamped.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include "px4_hexctl/offboard_control.hpp"
#include "px4_hexctl/vehicle.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <mutex>
#include <array>

using namespace std::chrono_literals;

class QRTracker : public rclcpp::Node {
public:
    QRTracker() : Node("qr_tracker") {
        // Parameters
        this->declare_parameter("target_id", 0);
        this->declare_parameter("altitude", 2.0);
        this->declare_parameter("kp_xy", 0.5);
        this->declare_parameter("kp_yaw", 0.3);
        this->declare_parameter("auto_track", false);
        this->declare_parameter("rqt_mouse_topic", "/image_view/mouse_event");
        this->declare_parameter("qgc_mouse_topic", "/qgc/mouse_event");
        this->declare_parameter("rqt_click_point_topic", "/image_view/click_point");
        this->declare_parameter("qgc_click_point_topic", "/qgc/click_point");
        this->declare_parameter("tracking_delta_x", 2.5);
        this->declare_parameter("tracking_delta_y", 0.0);
        this->declare_parameter("tracking_delta_z", 0.0);
        this->declare_parameter("camera_offset_x", 0.0);
        this->declare_parameter("camera_offset_y", 0.0);
        this->declare_parameter("camera_offset_z", 0.0);
        this->declare_parameter("marker_size", 0.2);
        this->declare_parameter("camera_fx", 554.0);
        this->declare_parameter("camera_fy", 554.0);
        this->declare_parameter("camera_cx", 320.0);
        this->declare_parameter("camera_cy", 240.0);
        
        target_id_ = this->get_parameter("target_id").as_int();
        target_alt_ = this->get_parameter("altitude").as_double();
        kp_xy_ = this->get_parameter("kp_xy").as_double();
        kp_yaw_ = this->get_parameter("kp_yaw").as_double();
        auto_track_ = this->get_parameter("auto_track").as_bool();
        rqt_mouse_topic_ = this->get_parameter("rqt_mouse_topic").as_string();
        qgc_mouse_topic_ = this->get_parameter("qgc_mouse_topic").as_string();
        rqt_click_point_topic_ = this->get_parameter("rqt_click_point_topic").as_string();
        qgc_click_point_topic_ = this->get_parameter("qgc_click_point_topic").as_string();
        tracking_delta_[0] = this->get_parameter("tracking_delta_x").as_double();
        tracking_delta_[1] = this->get_parameter("tracking_delta_y").as_double();
        tracking_delta_[2] = this->get_parameter("tracking_delta_z").as_double();
        camera_offset_[0] = this->get_parameter("camera_offset_x").as_double();
        camera_offset_[1] = this->get_parameter("camera_offset_y").as_double();
        camera_offset_[2] = this->get_parameter("camera_offset_z").as_double();
        marker_size_ = this->get_parameter("marker_size").as_double();
        camera_fx_ = this->get_parameter("camera_fx").as_double();
        camera_fy_ = this->get_parameter("camera_fy").as_double();
        camera_cx_ = this->get_parameter("camera_cx").as_double();
        camera_cy_ = this->get_parameter("camera_cy").as_double();

        camera_matrix_ = (cv::Mat1d(3, 3) <<
            camera_fx_, 0.0, camera_cx_,
            0.0, camera_fy_, camera_cy_,
            0.0, 0.0, 1.0);
        dist_coeffs_ = cv::Mat::zeros(1, 5, CV_64F);

        active_target_id_ = target_id_;
        tracking_enabled_ = auto_track_;

        // Subscriptions
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera", 10, std::bind(&QRTracker::image_callback, this, std::placeholders::_1));

        #ifdef HAS_IMAGE_VIEW_MSGS
        rqt_mouse_sub_ = this->create_subscription<image_view_msgs::msg::MouseEvent>(
            rqt_mouse_topic_, 10, std::bind(&QRTracker::mouse_event_callback, this, std::placeholders::_1));
        qgc_mouse_sub_ = this->create_subscription<image_view_msgs::msg::MouseEvent>(
            qgc_mouse_topic_, 10, std::bind(&QRTracker::mouse_event_callback, this, std::placeholders::_1));
        #else
        rqt_click_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            rqt_click_point_topic_, 10, std::bind(&QRTracker::point_click_callback, this, std::placeholders::_1));
        qgc_click_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            qgc_click_point_topic_, 10, std::bind(&QRTracker::point_click_callback, this, std::placeholders::_1));
        #endif

        // ArUco Dictionary
        dictionary_ = cv::makePtr<cv::aruco::Dictionary>(
            cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250));
        parameters_ = cv::makePtr<cv::aruco::DetectorParameters>();

        // Timer for Control Loop (20Hz)
        timer_ = this->create_wall_timer(50ms, std::bind(&QRTracker::control_loop, this));

        RCLCPP_INFO(this->get_logger(), "🚀 QR Tracker Node started. Target ArUco ID: %d", target_id_);
        RCLCPP_INFO(this->get_logger(), "🖱️ Left click on marker to track, right click to cancel.");
        #ifdef HAS_IMAGE_VIEW_MSGS
        RCLCPP_INFO(this->get_logger(), "✅ Using image_view_msgs mouse events.");
        #else
        RCLCPP_WARN(this->get_logger(), "⚠️ image_view_msgs not found. Use PointStamped clicks on %s or %s (z<0 for right click).",
                rqt_click_point_topic_.c_str(), qgc_click_point_topic_.c_str());
        #endif
    }

    void set_drone(std::shared_ptr<OffboardControl> drone) {
        drone_ = drone;
    }

private:
    struct MarkerInfo {
        int id;
        std::array<cv::Point2f, 4> corners;
    };
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            cv::Mat frame = cv_ptr->image;

            std::vector<int> ids;
            std::vector<std::vector<cv::Point2f>> corners;
            cv::aruco::detectMarkers(frame, dictionary_, corners, ids, parameters_);

            std::vector<MarkerInfo> markers;
            bool found = false;

            std::vector<cv::Vec3d> rvecs, tvecs;
            if (!ids.empty()) {
                cv::aruco::estimatePoseSingleMarkers(corners, marker_size_, camera_matrix_, dist_coeffs_, rvecs, tvecs);
            }

            for (size_t i = 0; i < ids.size(); i++) {
                MarkerInfo info;
                info.id = ids[i];
                for (size_t k = 0; k < 4; k++) {
                    info.corners[k] = corners[i][k];
                }
                markers.push_back(info);

                if (tracking_enabled_ && ids[i] == active_target_id_) {
                    const auto &tvec = tvecs[i];
                    // Camera frame (OpenCV): x right, y down, z forward
                    // Body frame (PX4): x forward, y right, z down
                    pos_body_frame_[0] = tvec[2] + camera_offset_[0];
                    pos_body_frame_[1] = -tvec[0] + camera_offset_[1];
                    pos_body_frame_[2] = -tvec[1] + camera_offset_[2];

                    target_visible_ = true;
                    last_detection_time_ = this->now();
                    found = true;

                    // Draw for visualization
                    cv::aruco::drawDetectedMarkers(frame, corners, ids);
                }
            }

            {
                std::lock_guard<std::mutex> guard(marker_lock_);
                last_markers_ = markers;
                last_image_width_ = frame.cols;
                last_image_height_ = frame.rows;
            }

            if (!tracking_enabled_) {
                target_visible_ = false;
            } else if (!found && (this->now() - last_detection_time_).seconds() > 1.0) {
                target_visible_ = false;
            }

        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    #ifdef HAS_IMAGE_VIEW_MSGS
    void mouse_event_callback(const image_view_msgs::msg::MouseEvent::SharedPtr msg) {
        if (msg->button == image_view_msgs::msg::MouseEvent::BUTTON_RIGHT) {
            tracking_enabled_ = false;
            target_visible_ = false;
            RCLCPP_INFO(this->get_logger(), "🛑 Tracking canceled by right click. Hovering.");
            return;
        }

        if (msg->button != image_view_msgs::msg::MouseEvent::BUTTON_LEFT) {
            return;
        }

        int selected_id = -1;
        if (select_target_from_click(msg->x, msg->y, selected_id)) {
            active_target_id_ = selected_id;
            tracking_enabled_ = true;
            RCLCPP_INFO(this->get_logger(), "🎯 Target selected: ArUco ID=%d", active_target_id_);
        } else {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "⚠️ Click was not on any detected marker. No tracking started.");
        }
    }
    #else
    void point_click_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        int button = (msg->point.z < 0.0) ? 2 : 1; // z<0 -> right click, otherwise left
        if (button == 2) {
            tracking_enabled_ = false;
            target_visible_ = false;
            RCLCPP_INFO(this->get_logger(), "🛑 Tracking canceled by right click (PointStamped). Hovering.");
            return;
        }

        int selected_id = -1;
        if (select_target_from_click(static_cast<float>(msg->point.x), static_cast<float>(msg->point.y), selected_id)) {
            active_target_id_ = selected_id;
            tracking_enabled_ = true;
            RCLCPP_INFO(this->get_logger(), "🎯 Target selected: ArUco ID=%d", active_target_id_);
        } else {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "⚠️ Click was not on any detected marker. No tracking started.");
        }
    }
    #endif

    bool select_target_from_click(float x, float y, int &selected_id) {
        std::vector<MarkerInfo> markers;
        {
            std::lock_guard<std::mutex> guard(marker_lock_);
            markers = last_markers_;
        }

        if (markers.empty()) {
            return false;
        }

        cv::Point2f click_pt(x, y);
        const double margin_px = 5.0;
        double best_dist = 1e9;
        bool found = false;

        for (const auto &marker : markers) {
            std::vector<cv::Point2f> poly(marker.corners.begin(), marker.corners.end());
            double dist = cv::pointPolygonTest(poly, click_pt, true);
            if (dist >= -margin_px && std::abs(dist) < best_dist) {
                best_dist = std::abs(dist);
                selected_id = marker.id;
                found = true;
            }
        }

        return found;
    }

    void control_loop() {
        if (!drone_) return;

        auto status = drone_->get_vehicle_status();
        
        // --- Auto Start Logic (Adaptive Takeoff from offboard_circle.cpp) ---
        static bool initial_takeoff_done = false;
        if (!initial_takeoff_done) {
            bool is_offboard = (status.nav_state == 14);
            bool is_armed = (status.arming_state == 2);

            if (!is_offboard || !is_armed) {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "🛫 Waiting for OFFBOARD & ARMED...");
                drone_->arm();
                drone_->engage_offboard_mode();
                return;
            }

            // We are OFFBOARD & ARMED.
            if (!drone_->is_position_valid()) {
                // Attitude-based liftoff sequence (boost then hover)
                if (!liftoff_started_) {
                    liftoff_started_ = true;
                    liftoff_start_time_ = this->now();
                }

                double elapsed = (this->now() - liftoff_start_time_).seconds();
                double thrust_cmd = (elapsed < 1.0) ? 0.72 : 0.58;

                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "🚀 [ADAPTIVE] EKF not ready. Attitude liftoff thrust=%.2f", thrust_cmd);
                drone_->update_attitude_setpoint(0.0, 0.0, 0.0, thrust_cmd);
                return;
            }

            // EKF is valid, use standard position takeoff
            auto pos = drone_->get_local_position();
            if (!home_z_initialized_) {
                home_z_ = pos.z;
                home_z_initialized_ = true;
            }

            double target_z = home_z_ + target_alt_;
            if (pos.z < target_z - 0.2) {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "🚀 Climbing to target altitude: %.2f (Current Z: %.2f)", target_z, pos.z);
                drone_->update_position_setpoint(0.0, 0.0, target_z, 0.0);
                return;
            }

            RCLCPP_INFO(this->get_logger(), "✅ Target altitude reached. Starting QR tracking.");
            initial_takeoff_done = true;
        }
        // -------------------------

        if (status.nav_state != 14 || status.arming_state != 2) {
            return;
        }

        if (!tracking_enabled_) {
            drone_->update_velocity_setpoint(0.0, 0.0, 0.0, 0.0);
            return;
        }

        if (target_visible_) {
            // ROS1-style body-frame tracking logic
            double vx = kp_xy_ * (pos_body_frame_[0] - tracking_delta_[0]);
            double vy = kp_xy_ * (pos_body_frame_[1] - tracking_delta_[1]);
            double vz = kp_xy_ * (pos_body_frame_[2] - tracking_delta_[2]);

            // Limit velocities
            vx = std::clamp(vx, -1.0, 1.0);
            vy = std::clamp(vy, -1.0, 1.0);
            vz = std::clamp(vz, -1.0, 1.0);

            drone_->update_velocity_setpoint(vx, vy, vz, 0.0);

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "🎯 Tracking: body=(%.2f,%.2f,%.2f) vel=(%.2f,%.2f,%.2f)",
                pos_body_frame_[0], pos_body_frame_[1], pos_body_frame_[2], vx, vy, vz);
        } else {
            // Hover when target is lost
            drone_->update_velocity_setpoint(0.0, 0.0, 0.0, 0.0);
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "⏸️ Target lost, hovering...");
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    #ifdef HAS_IMAGE_VIEW_MSGS
    rclcpp::Subscription<image_view_msgs::msg::MouseEvent>::SharedPtr rqt_mouse_sub_;
    rclcpp::Subscription<image_view_msgs::msg::MouseEvent>::SharedPtr qgc_mouse_sub_;
    #else
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr rqt_click_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr qgc_click_sub_;
    #endif
    rclcpp::TimerBase::SharedPtr timer_;
    
    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    cv::Ptr<cv::aruco::DetectorParameters> parameters_;

    std::shared_ptr<OffboardControl> drone_;
    
    int target_id_;
    double target_alt_;
    double kp_xy_, kp_yaw_;

    std::array<double, 3> tracking_delta_{0.0, 0.0, 0.0};
    std::array<double, 3> camera_offset_{0.0, 0.0, 0.0};
    std::array<double, 3> pos_body_frame_{0.0, 0.0, 0.0};

    double marker_size_ = 0.2;
    double camera_fx_ = 554.0;
    double camera_fy_ = 554.0;
    double camera_cx_ = 320.0;
    double camera_cy_ = 240.0;
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;

    bool auto_track_ = false;
    std::string rqt_mouse_topic_;
    std::string qgc_mouse_topic_;
    std::string rqt_click_point_topic_;
    std::string qgc_click_point_topic_;

    int active_target_id_ = 0;
    bool tracking_enabled_ = false;

    std::mutex marker_lock_;
    std::vector<MarkerInfo> last_markers_;
    int last_image_width_ = 0;
    int last_image_height_ = 0;

    bool liftoff_started_ = false;
    rclcpp::Time liftoff_start_time_{0, 0, RCL_ROS_TIME};
    bool home_z_initialized_ = false;
    double home_z_ = 0.0;

    bool target_visible_ = false;
    rclcpp::Time last_detection_time_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    
    // Create the vehicle/drone node (starts its own spinning background thread)
    auto vehicle = std::make_shared<Vehicle>();
    
    // Create the tracker node
    auto tracker = std::make_shared<QRTracker>();
    tracker->set_drone(vehicle->drone());

    // Spin only the tracker node in the main thread
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting QR Tracker node...");
    rclcpp::spin(tracker);
    
    rclcpp::shutdown();
    return 0;
}
