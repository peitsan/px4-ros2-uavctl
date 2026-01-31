#!/bin/bash
# =========================================================
# QR Code Tracking Demo (Docker/Headless) - Optimized
# =========================================================

PX4_PATH="/home/ubuntu/PX4-Autopilot"
WORLD_NAME="tracking_qr_sitl"
WS_PATH="/home/ubuntu/Desktop/px4-ros2-uavctl"
UAV_PX4_WS="/root/UAV-PX4"

# Set Gazebo Resource Path
export GZ_SIM_RESOURCE_PATH="$GZ_SIM_RESOURCE_PATH:$PX4_PATH/Tools/simulation/gz/models"

# Source ROS2 and specific workspaces
source /opt/ros/humble/setup.bash
if [ -f "$UAV_PX4_WS/install/setup.bash" ]; then
    source "$UAV_PX4_WS/install/setup.bash"
fi
if [ -f "$WS_PATH/install/setup.bash" ]; then
    source "$WS_PATH/install/setup.bash"
fi

# Cleanup
echo "🧹 Cleaning up old processes..."
pkill -9 -f MicroXRCEAgent || true
pkill -9 -f px4 || true
pkill -9 -f ruby || true 
pkill -9 -f parameter_bridge || true
pkill -9 -f qr_tracker || true
pkill -9 -f gz || true
pkill -9 -f ninja || true
pkill -9 -f tail || true

# Robust cleanup of lock files
rm -rf /tmp/px4* /tmp/gz* /tmp/xrce.log /tmp/bridge.log /tmp/px4.log
rm -f /dev/shm/gz_* 2>/dev/null || true

# Wait longer to ensure ports and shared memory are released
sleep 5

echo "📡 Starting MicroXRCEAgent..."
MicroXRCEAgent udp4 -p 8888 > /tmp/xrce.log 2>&1 &
sleep 2

echo "🛫 Starting PX4 SITL (x500_mono_cam) with world: $WORLD_NAME..."
cd "$PX4_PATH"
# Ensure the model path is included and no trailing colons
export GZ_SIM_RESOURCE_PATH="$PX4_PATH/Tools/simulation/gz/models"
export PX4_GZ_WORLD="$WORLD_NAME"

# Clean up any potential leftover lock files from previous runs
rm -f /tmp/px4_instance_*

# Run make with exact environment variables
# Note: we use 'make' but it eventually calls 'px4'. 
# If it fails, we check if px4 is already running or if paths are wrong.
PX4_GZ_WORLD="$WORLD_NAME" GZ_SIM_RESOURCE_PATH="$GZ_SIM_RESOURCE_PATH" make px4_sitl gz_x500_mono_cam > /tmp/px4.log 2>&1 &

echo "⏳ Waiting for PX4 and Gazebo to initialize..."
current_wait=0
while [ $current_wait -lt 40 ]; do
    if grep -q "Ready for takeoff" /tmp/px4.log 2>/dev/null; then
        echo "✅ PX4 is ready."
        break
    fi
    if grep -q "Service call timed out" /tmp/px4.log 2>/dev/null; then
        echo "❌ PX4 failed to spawn model (Timeout). Retrying with more logs..."
        tail -n 20 /tmp/px4.log
        exit 1
    fi
    sleep 2
    current_wait=$((current_wait+2))
    echo "... ($current_wait/40s)"
done

echo "🌉 Starting ROS-GZ Bridge..."
bash -c "source /opt/ros/humble/setup.bash && ros2 run ros_gz_bridge parameter_bridge /camera@sensor_msgs/msg/Image@gz.msgs.Image" > /tmp/bridge.log 2>&1 &
sleep 5

echo "🎯 Starting QR Tracker..."
bash -c "source /opt/ros/humble/setup.bash && source /root/UAV-PX4/install/setup.bash && source $WS_PATH/install/setup.bash && ros2 run px4_hexctl qr_tracker_alt --ros-args -p takeoff_thrust:=0.75"
