#!/bin/bash

set -e

WS_PATH="/home/ubuntu/Desktop/px4-ros2-uavctl"

source /opt/ros/humble/setup.bash
if [ -f "$WS_PATH/install/setup.bash" ]; then
    source "$WS_PATH/install/setup.bash"
fi

export VINS_ODOM_TOPIC=${VINS_ODOM_TOPIC:-/odometry}
export PX4_ODOM_TOPIC=${PX4_ODOM_TOPIC:-/fmu/in/vehicle_visual_odometry}

python3 "$WS_PATH/startup/vins_to_uxrce_dds.py"