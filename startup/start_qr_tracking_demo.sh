#!/bin/bash
# =========================================================
# QR Code Tracking Demo Startup Script (ROS2 Humble)
# Refactored from Prometheus ROS1 Script
# =========================================================

# === Paths ===
PX4_PATH="/home/ubuntu/PX4-Autopilot"
# Use the world provided by Prometheus as per user request
WORLD_PATH="/home/ubuntu/Prometheus/Simulator/gazebo_simulator/gazebo_worlds/detection_worlds/Tracking_QR/Tracking_QR.world"
WS_PATH="/home/ubuntu/Desktop/px4-ros2-uavctl"
GZ_SIM_RESOURCE_PATH=/home/ubuntu/PX4-Autopilot/Tools/simulation/gz/models

# Check dependencies
echo "🔍 Checking environment..."
if [ ! -d "$PX4_PATH" ]; then
    echo "❌ Error: PX4-Autopilot not found at $PX4_PATH"
    exit 1
fi

# Source ROS2 and Workspace
source /opt/ros/humble/setup.bash
if [ -f "$WS_PATH/install/setup.bash" ]; then
    source "$WS_PATH/install/setup.bash"
else
    echo "⚠️  Warning: Workspace install/setup.bash not found. Build the project first."
fi

# 1. Start MicroXRCEAgent
echo "📡 Starting MicroXRCEAgent..."
gnome-terminal --tab --title="📡 MicroXRCEAgent" -- bash -c "
    MicroXRCEAgent udp4 -p 8888;
    exec bash
" &

sleep 2

# 2. Start PX4 SITL + Gazebo
echo "🛫 Starting PX4 SITL + Gazebo..."
gnome-terminal --tab --title="🛫 PX4 SITL (x500)" -- bash -c "
    cd '$PX4_PATH';
    # Use gz-x500 and the specified world
    # Note: Modern PX4 uses 'gz' as the simulation backend (formerly Ignition)
    PX4_GZ_WORLD='tracking_qr_sitl' make px4_sitl gz_x500_mono_cam;
    exec bash
" &

# Wait for Gazebo and PX4 to initialize
sleep 15

# 3. Start ROS-GZ Bridge (Camera Image)
echo "🌉 Starting ROS-GZ Bridge..."
gnome-terminal --tab --title="🌉 ROS-GZ Bridge" -- bash -c "
    source /opt/ros/humble/setup.bash;
    # Bridge camera image from Gazebo to ROS2
    # Verify the topic name in your Gazebo environment
    ros2 run ros_gz_bridge parameter_bridge /camera@sensor_msgs/msg/Image@gz.msgs.Image;
    exec bash
" &

sleep 3

# 4. Start native C++ QR Tracker Node
echo "🎯 Starting QR Tracker..."
gnome-terminal --tab --title="🎯 QR Tracker" -- bash -c "
    source /opt/ros/humble/setup.bash;
    source '$WS_PATH/install/setup.bash';
    echo '🚀 Launching qr_tracker...';
    # Parameters: target_id (ArUco ID), altitude (Track altitude)
    ros2 run px4_hexctl qr_tracker --ros-args -p target_id:=100 -p altitude:=1.5;
    exec bash
" &

echo "✅ QR Tracking Demo successfully started!"
echo "📡 Tracker is listening to /camera topic"
