/**
 * @file qr_qrcode_opencv.cpp
 * @brief AprilTag 二维码视觉追踪节点 - 基于 OpenCV 的相对位置估计
 * 
 * 功能描述:
 * --------
 * 本节点通过实时处理相机视频流，检测 AprilTag 36h11 标记，并估计无人机与
 * 标记的相对位置关系 (3D 相对距离和角度偏差)。
 * 
 * 处理流程:
 * 1. 接收图像帧 (camera/image_raw)
 * 2. 使用 OpenCV ArUco 模块检测 AprilTag 标记
 * 3. 通过摄像头标定参数 (内参矩阵) 估计 6DOF 位姿 (tvec/rvec)
 * 4. 将摄像头坐标系转换为飞行体坐标系 (body frame)
 * 5. 计算相对位置误差 (δx, δy, δz)
 * 6. 发布相对位置目标值供飞控管理器使用
 * 
 * 关键概念:
 * --------
 * - tvec (平移向量): [x_cam, -y_cam, z_cam] 代表标记相对于摄像头的 3D 位置
 * - Body Frame: 飞行体坐标系 (x 向前, y 向右, z 向上)
 * - Relative Position: (x_body - 0.8, y_body, z_body) 表示与目标距离的误差
 * - 0.8s 节流日志: 每 0.8 秒输出一次相对位置信息，便于诊断
 * 
 * 坐标变换:
 * --------
 * 摄像头坐标系 → 飞行体坐标系:
 *   x_body = tvec[2]  (摄像头 z → 体 x，向前)
 *   y_body = tvec[0]  (摄像头 x → 体 y，向右)
 *   z_body = -tvec[1] (摄像头 -y → 体 z，向上)
 * 
 * 控制律 (相对位置 PID):
 * --------
 * vx = kp_distance * (x_body - 0.8)    // 距离控制: 保持 0.8m 前向距离
 * vy = kp_lateral  * y_body            // 横向控制: 保持标记在中心线上
 * vz = kp_vertical * z_body            // 竖直控制: 保持高度对齐
 * 
 * @author px4-ros2-uavctl team
 * @date 2026-02
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#include <chrono>
#include <mutex>

using namespace std::chrono_literals;

/**
 * @class QrCodeOpenCvTracker
 * @brief AprilTag 视觉追踪节点
 * 
 * 职责:
 * - 实时检测和追踪 AprilTag 标记
 * - 估计飞行器与标记的相对位置 (基于 6DOF 位姿估计)
 * - 发布相对位置目标供飞控使用
 * - 提供调试图像输出
 */
class QrCodeOpenCvTracker : public rclcpp::Node {
public:
    /**
     * @brief 构造函数 - 初始化节点和所有参数
     * 
     * 初始化步骤:
     * 1. 声明所有 ROS2 参数 (摄像头标定、控制增益等)
     * 2. 构建摄像头内参矩阵和畸变系数
     * 3. 创建订阅者 (图像输入)
     * 4. 创建发布者 (速度命令、相对位置、调试图像)
     * 5. 启动控制循环定时器 (50ms 周期)
     */
    QrCodeOpenCvTracker() : Node("qr_qrcode_opencv") {
        // ========== 话题名称配置 ==========
        declare_parameter("image_topic", std::string("/camera"));  ///< 输入图像流话题
        declare_parameter("cmd_vel_topic", std::string("/qr_tracker/cmd_vel_body"));  ///< 速度命令输出话题
        declare_parameter("target_pos_topic", std::string("/qr_tracker/relative_position"));  ///< 相对位置输出话题
        
        // ========== 标记和距离配置 ==========
        declare_parameter("qr_size_m", 0.2);  ///< AprilTag 边长 (米)
        declare_parameter("target_distance_m", 0.8);  ///< 目标前向距离 (米)
        declare_parameter("min_distance_m", 0.6);  ///< 最小安全距离 (米)
        
        // ========== 控制增益 (比例系数) ==========
        declare_parameter("kp_distance", 0.5);  ///< 前向距离 PID 增益
        declare_parameter("kp_lateral", 0.8);   ///< 横向偏差 PID 增益
        declare_parameter("kp_vertical", 0.8);  ///< 竖直偏差 PID 增益
        
        // ========== 速度限制 ==========
        declare_parameter("max_forward_speed", 0.6);  ///< 最大前向速度 (m/s)
        declare_parameter("max_lateral_speed", 0.5);  ///< 最大横向速度 (m/s)
        declare_parameter("max_vertical_speed", 0.5); ///< 最大竖直速度 (m/s)
        
        // ========== 调试和输出配置 ==========
        declare_parameter("publish_velocity", false);  ///< 是否发布速度命令 (可选)
        declare_parameter("publish_debug_image", true); ///< 是否发布调试图像
        declare_parameter("debug_image_topic", std::string("/qr_tracker/debug_image"));  ///< 调试图像话题
        
        // ========== 摄像头标定参数 ==========
        declare_parameter("camera_fx", 554.0);  ///< 焦距 x 像素 (内参)
        declare_parameter("camera_fy", 554.0);  ///< 焦距 y 像素 (内参)
        declare_parameter("camera_cx", 320.0);  ///< 主点 x 坐标 (内参)
        declare_parameter("camera_cy", 240.0);  ///< 主点 y 坐标 (内参)

        // 从参数服务器读取所有配置值
        image_topic_ = get_parameter("image_topic").as_string();
        cmd_vel_topic_ = get_parameter("cmd_vel_topic").as_string();
        target_pos_topic_ = get_parameter("target_pos_topic").as_string();
        qr_size_m_ = get_parameter("qr_size_m").as_double();
        target_distance_m_ = get_parameter("target_distance_m").as_double();
        min_distance_m_ = get_parameter("min_distance_m").as_double();
        kp_distance_ = get_parameter("kp_distance").as_double();
        kp_lateral_ = get_parameter("kp_lateral").as_double();
        kp_vertical_ = get_parameter("kp_vertical").as_double();
        max_forward_speed_ = get_parameter("max_forward_speed").as_double();
        max_lateral_speed_ = get_parameter("max_lateral_speed").as_double();
        max_vertical_speed_ = get_parameter("max_vertical_speed").as_double();
        publish_velocity_ = get_parameter("publish_velocity").as_bool();
        publish_debug_image_ = get_parameter("publish_debug_image").as_bool();
        debug_image_topic_ = get_parameter("debug_image_topic").as_string();
        camera_fx_ = get_parameter("camera_fx").as_double();
        camera_fy_ = get_parameter("camera_fy").as_double();
        camera_cx_ = get_parameter("camera_cx").as_double();
        camera_cy_ = get_parameter("camera_cy").as_double();

        // ===== 构建摄像头内参矩阵 =====
        // 内参矩阵 K = | fx  0  cx |
        //              | 0  fy  cy |
        //              | 0   0   1 |
        // 用于将 2D 像素坐标转换为 3D 相机坐标
        camera_matrix_ = (cv::Mat1d(3, 3) <<
            camera_fx_, 0.0, camera_cx_,
            0.0, camera_fy_, camera_cy_,
            0.0, 0.0, 1.0);
        
        // 畸变系数 (假设相机已经预校正，这里设置为零)
        dist_coeffs_ = cv::Mat::zeros(1, 5, CV_64F);

        // ===== 创建 ROS2 通信接口 =====
        // 订阅摄像头图像流 (10 帧缓冲)
        image_sub_ = create_subscription<sensor_msgs::msg::Image>(
            image_topic_, 10, std::bind(&QrCodeOpenCvTracker::image_callback, this, std::placeholders::_1));

        // 发布速度命令 (可选，用于调试)
        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
        
        // 发布相对位置目标 (飞控管理器将订阅此话题)
        target_pos_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(target_pos_topic_, 10);
        
        // 发布调试图像 (用于 RViz 可视化)
        if (publish_debug_image_) {
            debug_pub_ = create_publisher<sensor_msgs::msg::Image>(debug_image_topic_, 10);
        }
        
        // ===== 创建控制循环定时器 =====
        // 50ms 周期 = 20Hz，符合 PX4 OFFBOARD 模式最低频率要求 (2Hz+)
        timer_ = create_wall_timer(50ms, std::bind(&QrCodeOpenCvTracker::control_loop, this));

        RCLCPP_INFO(get_logger(), "✅ OpenCV QR tracker started. image_topic=%s, cmd_vel_topic=%s",
            image_topic_.c_str(), cmd_vel_topic_.c_str());
    }

private:
    /**
     * @brief 图像回调函数 - 实时处理视频流
     * 
     * 处理步骤:
     * 1. cv_bridge 将 ROS Image 消息转换为 OpenCV Mat 格式
     * 2. 使用 ArUco 检测器查找所有 AprilTag 标记
     * 3. 对每个检测到的标记执行 6DOF 位姿估计 (solvePnP)
     * 4. 选择最近的标记 (tvec[2] 最小，表示距离最近)
     * 5. 线程安全地保存 tvec/rvec 到状态变量
     * 6. 可选地在调试图像上绘制检测结果
     * 
     * @param msg ROS Image 消息
     */
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            // 将 ROS Image 消息转换为 OpenCV BGR8 格式
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            cv::Mat frame = cv_ptr->image;

            // ===== AprilTag 检测 =====
            // 使用 OpenCV ArUco 模块检测 DICT_APRILTAG_36h11 字典中的标记
            std::vector<int> ids;
            std::vector<std::vector<cv::Point2f>> corners;
            cv::aruco::detectMarkers(frame, dictionary_, corners, ids, detector_params_);

            if (!ids.empty()) {
                // ===== 6DOF 位姿估计 =====
                // estimatePoseSingleMarkers 使用 solvePnP 算法估计每个标记的位姿
                // 输出:
                //   rvecs - 旋转向量 (旋转角度和轴)
                //   tvecs - 平移向量 [x_cam, -y_cam, z_cam]
                std::vector<cv::Vec3d> rvecs, tvecs;
                cv::aruco::estimatePoseSingleMarkers(corners, qr_size_m_, camera_matrix_, dist_coeffs_, rvecs, tvecs);

                // ===== 选择最近的标记 =====
                // 根据 tvec[2] (前向距离) 选择最近的标记
                // 这是多标记场景中的简单启发式方法
                size_t best_idx = 0;
                double best_dist = tvecs[0][2];
                for (size_t i = 1; i < tvecs.size(); ++i) {
                    if (tvecs[i][2] < best_dist) {
                        best_dist = tvecs[i][2];
                        best_idx = i;
                    }
                }

                // ===== 线程安全地保存检测状态 =====
                {
                    std::lock_guard<std::mutex> guard(state_mutex_);
                    last_distance_m_ = best_dist;
                    last_tvec_ = tvecs[best_idx];  // 保存 tvec 用于坐标变换
                    last_seen_time_ = now();
                    has_detection_ = true;
                }

                // ===== 绘制调试图像 =====
                if (publish_debug_image_) {
                    // 绘制所有检测到的标记轮廓
                    cv::aruco::drawDetectedMarkers(frame, corners, ids);
                    // 绘制选中标记的坐标轴 (用于可视化 6DOF 位姿)
                    cv::aruco::drawAxis(frame, camera_matrix_, dist_coeffs_, rvecs[best_idx], tvecs[best_idx], qr_size_m_ * 0.5);
                }

                // ===== 计算标记中心 =====
                // 将 4 个角点的平均值作为标记中心 (像素坐标)
                const auto &best_corners = corners[best_idx];
                cv::Point2f center(0.0f, 0.0f);
                for (const auto &pt : best_corners) {
                    center.x += pt.x;
                    center.y += pt.y;
                }
                center.x /= 4.0f;
                center.y /= 4.0f;

                {
                    std::lock_guard<std::mutex> guard(state_mutex_);
                    last_center_x_ = center.x;
                    last_center_y_ = center.y;
                }

                // 在调试图像上标记中心点
                if (publish_debug_image_) {
                    cv::drawMarker(frame, center, cv::Scalar(0, 255, 0), cv::MARKER_CROSS, 20, 2);
                }
            }

            // ===== 发布调试图像 =====
            if (publish_debug_image_ && debug_pub_) {
                auto out_msg = cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::BGR8, frame).toImageMsg();
                debug_pub_->publish(*out_msg);
            }
        } catch (const cv_bridge::Exception &e) {
            RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    /**
     * @brief 控制循环 - 生成控制命令 (50ms 周期)
     * 
     * 核心流程:
     * 1. 从线程安全的状态变量读取最新的 tvec (平移向量)
     * 2. 检查检测的时效性 (需要在 0.5s 内有新检测)
     * 3. 将摄像头坐标系转换为飞行体坐标系
     * 4. 根据相对位置误差计算 PID 输出速度命令
     * 5. 应用速度限制和安全边界
     * 6. 发布相对位置目标消息 (供飞控管理器订阅)
     * 7. 可选发布速度命令消息
     * 8. 每 0.8 秒输出一次诊断日志
     * 
     * 坐标变换公式:
     *   x_body = tvec[2]   (摄像头前向距离 → 飞行体前向)
     *   y_body = tvec[0]   (摄像头右向 → 飞行体右向)
     *   z_body = -tvec[1]  (摄像头下向 → 飞行体上向)
     * 
     * 相对位置定义 (目标值):
     *   Δx = x_body - 0.8  (距离误差，负值表示太近)
     *   Δy = y_body        (横向误差，0 表示中心)
     *   Δz = z_body        (竖直误差，0 表示高度对齐)
     */
    void control_loop() {
        geometry_msgs::msg::Twist cmd{};
        geometry_msgs::msg::PointStamped target_msg{};
        bool publish = false;
        bool has_target = false;

        {
            std::lock_guard<std::mutex> guard(state_mutex_);
            // 检查是否有有效的检测且在时间范围内 (0.5 秒超时)
            if (has_detection_ && (now() - last_seen_time_).seconds() < 0.5) {
                const auto &tvec = last_tvec_;
                
                // ===== 坐标系变换: 摄像头 → 飞行体 =====
                // 摄像头坐标系 (OpenCV): x right, y down, z forward
                // 飞行体坐标系 (航空): x forward, y right, z up
                double x_body = tvec[2];   // 摄像头 z → 体 x (前向距离)
                double y_body = tvec[0];   // 摄像头 x → 体 y (右向)
                double z_body = -tvec[1];  // 摄像头 -y → 体 z (上向)

                // ===== PID 控制律 =====
                // 计算速度命令使无人机趋向目标位置
                double vx = kp_distance_ * (x_body - target_distance_m_);  // 保持 0.8m 前向距离
                double vy = kp_lateral_ * y_body;                          // 保持标记在中心线上
                double vz = kp_vertical_ * z_body;                         // 保持高度对齐

                // ===== 安全边界: 避免碰撞 =====
                // 如果检测到太近，强制后退
                if (x_body < min_distance_m_) {
                    vx = -std::abs(max_forward_speed_);
                }

                // ===== 应用速度限制 =====
                // 限制命令速度在安全范围内
                vx = std::max(-max_forward_speed_, std::min(max_forward_speed_, vx));
                vy = std::max(-max_lateral_speed_, std::min(max_lateral_speed_, vy));
                vz = std::max(-max_vertical_speed_, std::min(max_vertical_speed_, vz));

                cmd.linear.x = vx;
                cmd.linear.y = vy;
                cmd.linear.z = vz;

                // ===== 构建相对位置消息 =====
                // 飞控管理器将订阅此消息并将其转换为位置目标
                target_msg.header.stamp = now();
                target_msg.header.frame_id = "body";
                target_msg.point.x = x_body - target_distance_m_;  // Δx: 距离误差
                target_msg.point.y = y_body;                        // Δy: 横向误差
                target_msg.point.z = z_body;                        // Δz: 竖直误差
                publish = true;
                has_target = true;
            }
        }

        // 若无有效检测，发布零指令
        if (!publish) {
            cmd.linear.x = 0.0;
            cmd.linear.y = 0.0;
            cmd.linear.z = 0.0;
            target_msg.header.stamp = now();
            target_msg.header.frame_id = "body";
            target_msg.point.x = 0.0;
            target_msg.point.y = 0.0;
            target_msg.point.z = 0.0;
        }

        // ===== 0.8 秒节流日志 =====
        // 定期输出诊断信息，便于监控追踪状态
        auto now_time = now();
        if ((now_time - last_log_time_).seconds() >= 0.8) {
            if (has_target) {
                RCLCPP_INFO(get_logger(), "📍 relative position: x=%.3f y=%.3f z=%.3f",
                    target_msg.point.x, target_msg.point.y, target_msg.point.z);
            } else {
                RCLCPP_INFO(get_logger(), "📍 relative position: x=0.000 y=0.000 z=0.000 (no detection)");
            }
            last_log_time_ = now_time;
        }

        // ===== 发布输出消息 =====
        if (publish_velocity_) {
            cmd_pub_->publish(cmd);  // 可选：发布速度命令 (用于调试)
        }
        target_pos_pub_->publish(target_msg);  // 始终发布相对位置目标 (飞控使用)
    }

    // ========== 参数存储 ==========
    std::string image_topic_;          ///< 输入图像话题名
    std::string cmd_vel_topic_;        ///< 速度命令话题名
    std::string target_pos_topic_;     ///< 相对位置话题名
    double qr_size_m_;                 ///< AprilTag 边长 (米)
    double target_distance_m_;         ///< 目标前向距离 (米)
    double min_distance_m_;            ///< 最小安全距离 (米)
    double kp_distance_;               ///< 距离控制增益
    double max_forward_speed_;         ///< 最大前向速度 (m/s)
    double kp_lateral_;                ///< 横向控制增益
    double kp_vertical_;               ///< 竖直控制增益
    double max_lateral_speed_;         ///< 最大横向速度 (m/s)
    double max_vertical_speed_;        ///< 最大竖直速度 (m/s)
    bool publish_debug_image_;         ///< 是否发布调试图像
    bool publish_velocity_;            ///< 是否发布速度命令
    std::string debug_image_topic_;    ///< 调试图像话题名
    double camera_fx_, camera_fy_;     ///< 摄像头焦距 (像素)
    double camera_cx_, camera_cy_;     ///< 摄像头主点 (像素)

    // ========== ROS2 通信接口 ==========
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;  ///< 图像订阅者
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;     ///< 速度命令发布者
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr target_pos_pub_;  ///< 相对位置发布者
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;     ///< 调试图像发布者
    rclcpp::TimerBase::SharedPtr timer_;                                   ///< 控制循环定时器

    // ========== OpenCV ArUco 参数 ==========
    /// AprilTag 36h11 字典 (标准 4x4 位码)
    cv::Ptr<cv::aruco::Dictionary> dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11);
    /// 检测器参数 (使用默认值)
    cv::Ptr<cv::aruco::DetectorParameters> detector_params_ = cv::aruco::DetectorParameters::create();
    /// 摄像头内参矩阵 K
    cv::Mat camera_matrix_;
    /// 摄像头畸变系数
    cv::Mat dist_coeffs_;

    // ========== 状态变量 (线程安全) ==========
    std::mutex state_mutex_;           ///< 保护状态变量的互斥锁
    rclcpp::Time last_seen_time_;      ///< 上次成功检测的时间
    double last_distance_m_;           ///< 上次检测的前向距离 (m)
    cv::Vec3d last_tvec_;              ///< 上次检测的平移向量
    float last_center_x_, last_center_y_;  ///< 上次检测的标记中心像素坐标
    bool has_detection_;               ///< 是否有有效检测
    rclcpp::Time last_log_time_;       ///< 上次日志输出时间
};

/**
 * @brief 主函数 - 初始化 ROS2 并启动节点
 * 
 * ROS2 生命周期:
 * 1. rclcpp::init() - 初始化 ROS2 全局上下文
 * 2. std::make_shared() - 创建节点实例 (自动调用构造函数)
 * 3. rclcpp::spin() - 主事件循环 (阻塞直到 Ctrl+C)
 * 4. rclcpp::shutdown() - 清理资源
 * 
 * 当按下 Ctrl+C 时，节点优雅地退出
 */
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<QrCodeOpenCvTracker>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}