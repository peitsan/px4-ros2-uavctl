from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    realsense_pkg = get_package_share_directory("realsense2_camera")
    realsense_launch = os.path.join(realsense_pkg, "launch", "rs_lt_launch.py")

    tracker_node = Node(
        package="px4_hexctl",
        executable="qr_qrcode_opencv",
        name="qr_qrcode_opencv",
        output="screen",
        parameters=[{
            "image_topic": "/camera",
            "cmd_vel_topic": "/qr_tracker/cmd_vel_body",
            "qr_size_m": 0.2,
            "target_distance_m": 2.0,
            "min_distance_m": 0.6,
            "kp_distance": 0.5,
            "max_forward_speed": 0.6,
            "camera_fx": 554.0,
            "camera_fy": 554.0,
            "camera_cx": 320.0,
            "camera_cy": 240.0,
        }],
    )

    offboard_node = Node(
        package="px4_hexctl",
        executable="offboard_tracker_manager",
        name="offboard_tracker_manager",
        output="screen",
        parameters=[{
            "cmd_vel_topic": "/qr_tracker/cmd_vel_body",
            "takeoff_alt": 1.5,
            "command_timeout": 0.5,
            "enable_adaptive_liftoff": False,
        }],
    )

    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(realsense_launch)),
        TimerAction(period=3.0, actions=[tracker_node]),
        TimerAction(period=3.0, actions=[offboard_node]),
    ])