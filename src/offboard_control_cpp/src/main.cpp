#include <rclcpp/rclcpp.hpp>
#include "offboard_control_cpp/offboard_control.hpp"
#include "offboard_control_cpp/vehicle.hpp"


int main(int argc, char* argv[]) {
    // 初始化 ROS2 节点系统
    //rclcpp::init(argc, argv);

    // 创建 Vehicle 实例
    auto vehicle = std::make_shared<Vehicle>();

    try {
        // 解锁
        vehicle->drone()->arm();

        bool ok = vehicle->drone()->takeoff_command_global(1.5, 0.0 /*pitch deg*/, 0.0 /*yaw deg*/, NAN /*lat*/, NAN /*lon*/, 300.0 /*timeout*/);

        //bool ok2 = vehicle->drone()->takeoff_command_local(1.5 /*z rel m*/, 1.0 /*ascend m/s*/, 0.261799387 /*15deg in rad*/, NAN /*yaw*/, NAN /*x*/, NAN /*y*/, 20.0 /*timeout*/);

        // 起飞
        if (ok) {
            // 飞到指定轨迹点 (x, y, z, yaw)
            vehicle->drone()->fly_to_trajectory_setpoint(5.0, 0.0, 2.0, 0.0, 100);
        }

        
        vehicle->drone()->land();
        // 上锁
        vehicle->drone()->disarm();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Exception caught: %s", e.what());
    }

    // 清理资源
    vehicle->close();

    // 关闭 ROS2
    //rclcpp::shutdown();
    return 0;
}
