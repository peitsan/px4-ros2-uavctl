from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    realsense_pkg = get_package_share_directory('realsense_camera')
    realsense_launch = os.path.join(realsense_pkg, 'launch', 'rs_lt_launch.py')

    tracker_node = Node(
        package='px4_hexctl',
        executable='qr_tracker_spirecv',
        name='qr_tracker_spirecv',
        output='screen',
        parameters=[{
            'rgb_topic': '/camera/d435i/color/image_raw',
            'depth_topic': '/camera/d435i/depth/image_rect_raw',
            'target_id': 1,
            'takeoff_alt': 1.5,
            'tracking_delta_x': 2.5,
            'tracking_delta_y': 0.0,
            'tracking_delta_z': 0.0,
            'track_z': False,
            'cmd_vel_topic': '/qr_tracker/cmd_vel_body'
        }]
    )

    offboard_node = Node(
        package='px4_hexctl',
        executable='offboard_tracker_manager',
        name='offboard_tracker_manager',
        output='screen',
        parameters=[{
            'cmd_vel_topic': '/qr_tracker/cmd_vel_body',
            'takeoff_alt': 1.5,
            'command_timeout': 0.5,
            'enable_adaptive_liftoff': False
        }]
    )

    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(realsense_launch)),
        TimerAction(period=3.0, actions=[offboard_node]),
        TimerAction(period=5.0, actions=[tracker_node]),
    ])
