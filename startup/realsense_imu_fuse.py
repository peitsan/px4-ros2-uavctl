#!/usr/bin/env python3

import os
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


class RealsenseImuFuse(Node):
    def __init__(self) -> None:
        super().__init__("realsense_imu_fuse")
        self._accel_topic = os.environ.get("RS_ACCEL_TOPIC", "/camera/camera/accel/sample")
        self._gyro_topic = os.environ.get("RS_GYRO_TOPIC", "/camera/camera/gyro/sample")
        self._output_topic = os.environ.get("RS_IMU_TOPIC", "/camera/imu")
        self._max_dt = float(os.environ.get("RS_IMU_MAX_DT", "0.02"))

        self._latest_accel: Optional[Imu] = None
        self._latest_gyro: Optional[Imu] = None

        imu_pub_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._publisher = self.create_publisher(Imu, self._output_topic, imu_pub_qos)
        self.create_subscription(Imu, self._accel_topic, self._on_accel, qos_profile_sensor_data)
        self.create_subscription(Imu, self._gyro_topic, self._on_gyro, qos_profile_sensor_data)

        self.get_logger().info(
            f"Fuse IMU: accel={self._accel_topic}, gyro={self._gyro_topic}, out={self._output_topic}"
        )

    def _on_accel(self, msg: Imu) -> None:
        self._latest_accel = msg
        self._maybe_publish()

    def _on_gyro(self, msg: Imu) -> None:
        self._latest_gyro = msg
        self._maybe_publish()

    def _maybe_publish(self) -> None:
        if self._latest_accel is None or self._latest_gyro is None:
            return

        accel_ts = self._latest_accel.header.stamp.sec + self._latest_accel.header.stamp.nanosec * 1e-9
        gyro_ts = self._latest_gyro.header.stamp.sec + self._latest_gyro.header.stamp.nanosec * 1e-9
        if abs(accel_ts - gyro_ts) > self._max_dt:
            return

        out = Imu()
        out.header.stamp = self._latest_gyro.header.stamp
        out.header.frame_id = self._latest_gyro.header.frame_id

        out.angular_velocity = self._latest_gyro.angular_velocity
        out.linear_acceleration = self._latest_accel.linear_acceleration

        out.angular_velocity_covariance = self._latest_gyro.angular_velocity_covariance
        out.linear_acceleration_covariance = self._latest_accel.linear_acceleration_covariance
        out.orientation_covariance = self._latest_gyro.orientation_covariance

        self._publisher.publish(out)


def main() -> None:
    rclpy.init()
    node = RealsenseImuFuse()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()