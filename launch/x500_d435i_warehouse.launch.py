#!/usr/bin/env python3
"""
X500 + D435i Camera in Warehouse Environment - Launch Script
启动X500无人机 + D435i深度相机 + Warehouse仓库环境
"""

import os
import xacro
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def setup_ign_resource_path(context):
    """Setup Ignition/Gazebo resource paths"""
    ws_path = "/home/ubuntu/Desktop/px4-ros2-uavctl"
    os.environ['GZ_SIM_RESOURCE_PATH'] = (
        f"{ws_path}/urdf:"
        f"{ws_path}/world:"
        f"/usr/share/gz/gz-sim7/models:"
        f"/usr/share/ignition/ignition-gazebo6/models"
    )
    return []

def generate_launch_description():
    """Generate launch description for X500 + D435i in Warehouse"""
    
    ws_path = "/home/ubuntu/Desktop/px4-ros2-uavctl"
    uav_model = 'x500_d435i_full'
    uav_ns = 'x500'
    world_file = 'warehouse_x500_d435i.sdf'
    
    # Initial pose: x, y, z, roll, pitch, yaw
    pose = ['2.0', '2.0', '0.5', '0.0', '0.0', '0.0']
    
    # ============ ENVIRONMENT VARIABLES ============
    set_env_vars = OpaqueFunction(function=setup_ign_resource_path)
    
    # ============ LAUNCH ARGUMENTS ============
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock')
    
    # ============ GAZEBO SIMULATION ============
    # Start Gazebo with Warehouse world
    gazebo_process = ExecuteProcess(
        cmd=[
            'gz', 'sim',
            f'{ws_path}/world/{world_file}',
            '-v', '3',
            '-r'
        ],
        output='screen',
        shell=False
    )
    
    # ============ ROBOT DESCRIPTION ============
    # Process XACRO file to generate URDF
    xacro_file = os.path.join(ws_path, 'urdf', f'{uav_model}.xacro')
    
    doc = xacro.process_file(xacro_file, mappings={'ns': uav_ns})
    robot_desc = doc.toprettyxml(indent='  ')
    
    # ============ SPAWN ROBOT ============
    # Spawn X500 + D435i in Gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-string', robot_desc,
            '-x', pose[0], '-y', pose[1], '-z', pose[2],
            '-R', pose[3], '-P', pose[4], '-Y', pose[5],
            '-name', uav_ns,
            '-allow_renaming', 'false'
        ]
    )
    
    # ============ ROBOT STATE PUBLISHER ============
    # Publish robot state (TF transforms)
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=uav_ns,
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )
    
    # ============ ROS2-GAZEBO BRIDGE ============
    # Bridge camera and IMU topics from Gazebo to ROS2
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Camera topics
            '/camera/color/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/color/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/camera/depth/image_rect_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/infra1/image_rect_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/infra2/image_rect_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            # IMU topic
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            # Odometry topic
            f'/{uav_ns}/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            # Joint states
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
        output='screen',
        remappings=[
            ('/odometry', f'/{uav_ns}/odometry'),
        ]
    )
    
    # ============ RVIZ2 VISUALIZATION ============
    # Launch RViz2 for visualization
    rviz_config = os.path.join(ws_path, 'rviz', 'x500_d435i.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        remappings=[
            ('/camera/color/image_raw', '/camera/color/image_raw'),
        ]
    )
    
    # ============ CAMERA INFO PUBLISHER ============
    # Publish camera intrinsics
    camera_info_pub = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/camera/color/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ],
        output='screen'
    )
    
    # ============ BUILD LAUNCH DESCRIPTION ============
    ld = LaunchDescription()
    
    # Add environment setup
    ld.add_action(set_env_vars)
    
    # Add launch arguments
    ld.add_action(use_sim_time)
    
    # Add core processes
    ld.add_action(gazebo_process)
    
    # Wait for Gazebo to start before spawning
    import time
    ld.add_action(ExecuteProcess(
        cmd=['sleep', '5'],
        output='screen'
    ))
    
    # Add robot-related nodes
    ld.add_action(spawn_entity)
    ld.add_action(robot_state_pub)
    ld.add_action(ros_gz_bridge)
    
    # Add visualization
    ld.add_action(rviz_node)
    
    return ld
