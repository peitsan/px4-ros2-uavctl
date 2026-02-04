#include <string>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <action_msgs/msg/goal_status_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <cmath>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <mavros_msgs/srv/command_long.hpp>

class DroneHover : public rclcpp::Node
{
public:
    DroneHover() : Node("offboard_single_position")
    {		
        state_sub_ = this->create_subscription<mavros_msgs::msg::State>("mavros/state", 10, std::bind(&DroneHover::state_callback, this, std::placeholders::_1));        
		auto qos = rclcpp::SensorDataQoS();
		local_pos_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("mavros/local_position/pose", qos, std::bind(&DroneHover::local_pos_cb, this, std::placeholders::_1));
        local_pos_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("mavros/setpoint_position/local", 10);
        arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>("mavros/cmd/arming");
        set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("mavros/set_mode");
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&DroneHover::timer_callback, this));
    }

private:
    void state_callback(const mavros_msgs::msg::State::SharedPtr msg)
    {
        current_state_ = *msg;
    } 
    void local_pos_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
	    local_pos = *msg;
	    if (!flag_init_position && (local_pos.pose.position.z != 0))
	    {
			init_position_x_take_off = local_pos.pose.position.x;
			init_position_y_take_off = local_pos.pose.position.y;
			init_position_z_take_off = local_pos.pose.position.z;
			flag_init_position = true;
	    }
	   // tf2::fromMsg(local_pos.pose.pose.orientation, quat);
	   // tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
	}
    void timer_callback()
    {
        if (current_state_.mode != "OFFBOARD"&& set_mode==false)
        {
            auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
            request->custom_mode = "OFFBOARD";
            set_mode_client_->async_send_request(request);
        }
        else if (!current_state_.armed&& set_arm==false)
        {
            auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
            request->value = true;
            arming_client_->async_send_request(request); 
        }
        if(current_state_.mode  == "OFFBOARD") set_mode = true;
        if(current_state_.armed == true)       set_arm = true;        
        target_pose_.header.stamp = this->now();
        target_pose_.pose.position.x = init_position_x_take_off + 0;
        target_pose_.pose.position.y = init_position_y_take_off + 0;
        target_pose_.pose.position.z = init_position_z_take_off + 2;        
        local_pos_pub_->publish(target_pose_);
    }
    
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_pos_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr local_pos_sub_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
    rclcpp::TimerBase::SharedPtr timer_;

    mavros_msgs::msg::State current_state_;
    geometry_msgs::msg::PoseStamped target_pose_;
	tf2::Quaternion quat;
	double roll, pitch, yaw;
	float init_position_x_take_off = 0;
	float init_position_y_take_off = 0;
	float init_position_z_take_off = 0;
	bool flag_init_position = false;
	geometry_msgs::msg::PoseStamped local_pos;
    bool set_mode{false};
    bool set_arm{false};
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DroneHover>());
    rclcpp::shutdown();
    return 0;
}
