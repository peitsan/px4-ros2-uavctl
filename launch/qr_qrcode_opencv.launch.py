from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    start_image_view = LaunchConfiguration("start_image_view")
    realsense_pkg = get_package_share_directory("realsense2_camera")
    realsense_launch = os.path.join(realsense_pkg, "launch", "rs_lt_launch.py")

    image_view = ExecuteProcess(
        cmd=["rqt", "--standalone", "rqt_image_view", "--force-discover"],
        output="screen",
        condition=IfCondition(start_image_view),
    )

    tracker_node = Node(
        package="px4_hexctl",
        executable="qr_qrcode_opencv",
        name="qr_qrcode_opencv",
        output="screen",
        parameters=[{
            "image_topic": "/camera/d435i/color/image_raw",
            "cmd_vel_topic": "/qr_tracker/cmd_vel_body",
            "qr_size_m": 0.2,
            "target_distance_m": 2.0,
            "min_distance_m": 0.6,
            "kp_distance": 0.5,
            "max_forward_speed": 0.6,
            "publish_debug_image": True,
            "debug_image_topic": "/qr_tracker/debug_image",
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
        DeclareLaunchArgument(
            "start_image_view",
            default_value="true",
            description="Open rqt_image_view window to visualize detection",
        ),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(realsense_launch)),
        TimerAction(period=0.2, actions=[image_view]),
        TimerAction(period=3.0, actions=[tracker_node]),
        TimerAction(period=3.0, actions=[offboard_node]),
    ])