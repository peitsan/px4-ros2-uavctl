#include <rclcpp/rclcpp.hpp>
#include "px4_hexctl/offboard_control.hpp"
#include <chrono>

using namespace std::chrono_literals;

/**
 * @brief QRTrackerAltitudeNode
 * 
 * 这是一个简化的演示节点，模拟视觉跟踪逻辑，但使用高度控制（定高模式）。
 * 它首先执行起飞，然后维持在设定高度，并根据是否看到目标发送速度指令。
 */
class QRTrackerAltitudeNode : public rclcpp::Node {
public:
    QRTrackerAltitudeNode() : Node("qr_tracker_altitude") {
        // 参数声明
        this->declare_parameter("altitude", 1.5);
        this->declare_parameter("takeoff_thrust", 0.68); // 针对仿真环境的起飞推力

        altitude_target_ = this->get_parameter("altitude").as_double();
        takeoff_thrust_ = this->get_parameter("takeoff_thrust").as_double();

        // 初始化离板控制库
        drone_ = std::make_shared<OffboardControl>();
        
        // 创建控制循环
        timer_ = this->create_wall_timer(50ms, std::bind(&QRTrackerAltitudeNode::control_loop, this));
        
        RCLCPP_INFO(this->get_logger(), "🚀 QR Tracker Altitude Node Started");
    }

private:
    void control_loop() {
        if (!drone_) return;

        auto status = drone_->get_vehicle_status();
        
        // 状态机
        if (status.arming_state != 2 || status.nav_state != 14) {
            // 未解锁或不在离板模式，尝试解锁并进入离板
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Waiting for Offboard and Armed status...");
            
            drone_->arm();
            drone_->engage_offboard_mode(); // 使用 engage_offboard_mode
            return;
        }

        // --- 进入 Offboard 后的逻辑 ---
        
        // 获取当前位置
        auto pos = drone_->get_local_position();
        double current_z = pos.z;
        
        // 简化的垂直控制：如果高度不足 0.3m，认为还在地面，强制给起飞推力
        if (!drone_->is_position_valid() || current_z < 0.3) { 
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Liftoff thrust applied via Attitude Setpoint...");
            // 使用姿态控制进行强制起飞 (Roll=0, Pitch=0, Yaw=0, Thrust)
            drone_->update_attitude_setpoint(0.0, 0.0, 0.0, takeoff_thrust_);
            return;
        }

        // 高度闭环逻辑 (简单 P 控制)
        double error_z = altitude_target_ - current_z;
        double vz = error_z * 0.5; // P=0.5
        vz = std::clamp(vz, -1.0, 1.0);

        // 水平指令：模拟跟踪，暂时悬停
        double vx = 0.0;
        double vy = 0.0;
        double yaw_rate = 0.0;

        drone_->update_velocity_setpoint(vx, vy, vz, yaw_rate);
        
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
            "Altitude Hold: z=%.2f target=%.2f vz=%.2f", current_z, altitude_target_, vz);
    }

    std::shared_ptr<OffboardControl> drone_;
    rclcpp::TimerBase::SharedPtr timer_;
    double altitude_target_;
    double takeoff_thrust_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<QRTrackerAltitudeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
