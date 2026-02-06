#!/usr/bin/env python3
"""
VINS-Fusion + PX4 Offboard Hover Controller
基于VINS视觉里程计实现PX4自动悬停
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

import numpy as np
from tf_transformations import quaternion_from_euler, euler_from_quaternion
import math
import time

# ROS2 messages
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TwistStamped
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleStatus, VehicleCommand


class VINSOffboardHover(Node):
    """VINS-based offboard hover controller for PX4"""
    
    def __init__(self):
        super().__init__('vins_offboard_hover')
        
        # Configuration
        self.declare_parameter('arm_timeout', 5.0)
        self.declare_parameter('takeoff_height', 1.5)
        self.declare_parameter('hover_timeout', 30.0)
        
        self.arm_timeout = self.get_parameter('arm_timeout').value
        self.takeoff_height = self.get_parameter('takeoff_height').value
        self.hover_timeout = self.get_parameter('hover_timeout').value
        
        # State variables
        self.vehicle_status = None
        self.vins_odometry = None
        self.home_pose = None
        self.current_pose = None
        
        self.offboard_setpoint_counter = 0
        self.is_armed = False
        self.is_offboard = False
        self.start_time = time.time()
        self.takeoff_time = None
        
        # Callback groups for non-blocking execution
        self.sensor_cb_group = MutuallyExclusiveCallbackGroup()
        self.control_cb_group = MutuallyExclusiveCallbackGroup()
        
        # Publishers
        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            10
        )
        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            10
        )
        self.vehicle_cmd_pub = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            10
        )
        
        # Subscribers
        self.vins_sub = self.create_subscription(
            Odometry,
            '/vins_estimator/odometry',
            self.vins_callback,
            10,
            callback_group=self.sensor_cb_group
        )
        
        self.vehicle_status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.vehicle_status_callback,
            10,
            callback_group=self.sensor_cb_group
        )
        
        # Control timer
        self.control_timer = self.create_timer(
            0.1,  # 10Hz control loop
            self.control_loop,
            callback_group=self.control_cb_group
        )
        
        self.get_logger().info('VINS Offboard Hover Controller initialized')
        self.get_logger().info(f'  Takeoff height: {self.takeoff_height}m')
        self.get_logger().info(f'  Hover timeout: {self.hover_timeout}s')
    
    def vins_callback(self, msg: Odometry):
        """Handle VINS odometry updates"""
        self.vins_odometry = msg
        self.current_pose = msg.pose.pose
        
        # Set home position on first VINS measurement
        if self.home_pose is None:
            self.home_pose = msg.pose.pose
            self.get_logger().info(
                f'Home position set: '
                f'x={self.home_pose.position.x:.2f}, '
                f'y={self.home_pose.position.y:.2f}, '
                f'z={self.home_pose.position.z:.2f}'
            )
    
    def vehicle_status_callback(self, msg: VehicleStatus):
        """Handle vehicle status updates"""
        self.vehicle_status = msg
        
        if msg.arming_state == VehicleStatus.ARMING_STATE_ARMED and not self.is_armed:
            self.is_armed = True
            self.get_logger().warn('🚁 Vehicle ARMED')
            
        if msg.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and not self.is_offboard:
            self.is_offboard = True
            self.get_logger().warn('✈️ OFFBOARD mode ACTIVE')
            self.takeoff_time = time.time()
    
    def control_loop(self):
        """Main control loop"""
        
        # Wait for VINS initialization
        if self.vins_odometry is None or self.home_pose is None:
            self.get_logger().debug('Waiting for VINS odometry...')
            return
        
        # Increment counter
        self.offboard_setpoint_counter += 1
        
        # Publish offboard control mode
        self.publish_offboard_mode()
        
        # State machine
        elapsed = time.time() - self.start_time
        
        if elapsed < self.arm_timeout and not self.is_armed:
            # Arm vehicle
            self.arm_vehicle()
            self.get_logger().info(f'[{elapsed:.1f}s] Arming vehicle...')
            
        elif self.is_armed and not self.is_offboard:
            # Switch to offboard mode and start takeoff
            self.publish_trajectory_setpoint(
                self.home_pose.position.x,
                self.home_pose.position.y,
                -self.takeoff_height,  # NED coordinate
                0.0, 0.0, 0.0
            )
            if self.offboard_setpoint_counter % 10 == 0:
                self.get_logger().info('Publishing offboard setpoints...')
            
        elif self.is_offboard:
            # Offboard active - hold position
            elapsed_offboard = time.time() - self.takeoff_time
            
            if elapsed_offboard < self.hover_timeout:
                # Maintain hover at home position
                self.publish_trajectory_setpoint(
                    self.home_pose.position.x,
                    self.home_pose.position.y,
                    -self.takeoff_height,  # NED coordinate
                    0.0, 0.0, 0.0
                )
                
                if self.offboard_setpoint_counter % 50 == 0:
                    current_z = -self.current_pose.position.z
                    self.get_logger().info(
                        f'[{elapsed_offboard:.1f}s] Hovering at z={current_z:.2f}m'
                    )
            else:
                # Timeout - land
                self.get_logger().warn(f'Hover timeout reached ({self.hover_timeout}s) - LANDING')
                self.land_vehicle()
    
    def publish_offboard_mode(self):
        """Publish offboard control mode"""
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        
        self.offboard_mode_pub.publish(msg)
    
    def publish_trajectory_setpoint(self, x, y, z, vx=0.0, vy=0.0, vz=0.0):
        """Publish trajectory setpoint"""
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = [x, y, z]
        msg.velocity = [vx, vy, vz]
        msg.acceleration = [0.0, 0.0, 0.0]
        msg.jerk = [float('nan')] * 3
        msg.yaw = 0.0
        msg.yaw_rate = 0.0
        
        self.trajectory_setpoint_pub.publish(msg)
    
    def arm_vehicle(self):
        """Send arm command"""
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.param1 = 1.0
        msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        
        self.vehicle_cmd_pub.publish(msg)
    
    def land_vehicle(self):
        """Send land command"""
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.command = VehicleCommand.VEHICLE_CMD_NAV_LAND
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        
        self.vehicle_cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    
    node = VINSOffboardHover()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
