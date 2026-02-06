#!/usr/bin/env python3

import rclpy
from offboard_control_lib import Vehicle
import time

def main():
   

    # 创建 Vehicle 实例（现在它是 rclpy.node.Node 的子类）
    vehicle = Vehicle()

    try:
        vehicle.drone.arm()
        if vehicle.drone.takeoff(2.0):
            vehicle.drone.fly_to_trajectory_setpoint(5.0, 0.0, 2.0, 0.0)  # x, y, z, yaw
            vehicle.drone.fly_to_trajectory_setpoint(5.0, 0.0, 2.0, 1.57)  # x, y, z, yaw
            vehicle.drone.fly_to_trajectory_setpoint(5.0, 5.0, 2.0, 1.57)  # x, y, z, yaw
            vehicle.drone.fly_to_trajectory_setpoint(5.0, 5.0, 2.0, 3.14)  # x, y, z, yaw
            vehicle.drone.fly_to_trajectory_setpoint(0.0, 5.0, 2.0, 3.14)  # x, y, z, yaw
            vehicle.drone.fly_to_trajectory_setpoint(0.0, 5.0, 2.0, 4.71)  # x, y, z, yaw
            vehicle.drone.fly_to_trajectory_setpoint(0.0, 0.0, 2.0, 4.71)  # x, y, z, yaw
            vehicle.drone.fly_to_trajectory_setpoint(0.0, 0.0, 2.0, 0.0)  # x, y, z, yaw
        #vehicle.drone.simulated_land()
        vehicle.drone.land()
        time.sleep(1)
        vehicle.drone.disarm()
    except KeyboardInterrupt:
        pass
    finally:
        # 清理资源
        vehicle.close()

if __name__ == '__main__':
    main()