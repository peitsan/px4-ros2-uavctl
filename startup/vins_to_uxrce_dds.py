#!/usr/bin/env python3

import os

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry


class VinsToUxrceBridge(Node):
    def __init__(self) -> None:
        super().__init__("vins_to_uxrce_bridge")
        self._input_topic = os.environ.get("VINS_ODOM_TOPIC", "/odometry")
        self._output_topic = os.environ.get("PX4_ODOM_TOPIC", "/fmu/in/vehicle_visual_odometry")

        self._publisher = self.create_publisher(VehicleOdometry, self._output_topic, qos_profile_sensor_data)
        self.create_subscription(Odometry, self._input_topic, self._on_odom, qos_profile_sensor_data)
        self.get_logger().info(f"Bridge {self._input_topic} -> {self._output_topic}")

    def _on_odom(self, msg: Odometry) -> None:
        out = VehicleOdometry()
        out.timestamp = int(msg.header.stamp.sec * 1_000_000 + msg.header.stamp.nanosec / 1000)
        out.timestamp_sample = out.timestamp

        out.pose_frame = VehicleOdometry.POSE_FRAME_NED
        out.position = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
        out.q = [
            msg.pose.pose.orientation.w,
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
        ]

        out.velocity_frame = VehicleOdometry.VELOCITY_FRAME_NED
        out.velocity = [
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z,
        ]
        out.angular_velocity = [
            msg.twist.twist.angular.x,
            msg.twist.twist.angular.y,
            msg.twist.twist.angular.z,
        ]

        out.position_variance = [float("nan")] * 3
        out.orientation_variance = [float("nan")] * 3
        out.velocity_variance = [float("nan")] * 3

        self._publisher.publish(out)


def main() -> None:
    rclpy.init()
    node = VinsToUxrceBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()