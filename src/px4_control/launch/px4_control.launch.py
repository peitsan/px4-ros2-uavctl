#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_dir = get_package_share_directory('px4_control')

    # RViz 配置文件
    rviz1_config = os.path.join(package_dir,  'px4.rviz')
    rviz2_config = os.path.join(package_dir, 'rviz2.rviz')

    return LaunchDescription([

        # 外部 PX4 SITL / MicroXRCEAgent 进程
        Node(
            package='px4_control',
            executable='process',
            name='process',
            output='screen'
        ),

        # Figure-8 可视化节点
        Node(
            package='px4_control',
            executable='figure8_visualizer',
            name='figure8_visualizer',
            output='screen'
        ),

        # PX4 Vehicle 可视化节点
        Node(
            package='px4_control',
            executable='px4_visualizer',
            name='px4_visualizer',
            output='screen'
        ),

        # 八字轨迹 Offboard 控制节点
        Node(
            package='px4_control',
            executable='offboard_figure8',
            name='offboard_figure8',
            output='screen'
        ),

        # RViz 1
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_px4',
            output='screen',
            arguments=['-d', rviz1_config]
        ),

        # RViz 2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_figure8',
            output='screen',
            arguments=['-d', rviz2_config]
        ),
    ])

