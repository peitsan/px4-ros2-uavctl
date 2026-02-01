#!/bin/bash
# =========================================================
# QR Code Tracking Demo Startup Script (ROS2 Humble)
# Refactored from Prometheus ROS1 Script
# =========================================================

# === Paths ===
PX4_PATH="/home/ubuntu/PX4-Autopilot"
WORLD_NAME="aruco_6X6_250"
WORLD_SOURCE="/home/ubuntu/Desktop/px4-ros2-uavctl/world/aruco_6X6_250.world"
PROMETHEUS_MODELS="/home/ubuntu/Prometheus/Simulator/gazebo_simulator/gazebo_models"
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
export PATH="/opt/ros/humble/bin:$PATH"
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
    cp '$WORLD_SOURCE' '$PX4_PATH/Tools/simulation/gz/worlds/aruco_6X6_250.sdf'
    
    cd '$PX4_PATH';
    export GZ_SIM_RESOURCE_PATH=\$GZ_SIM_RESOURCE_PATH:\$PX4_PATH/Tools/simulation/gz/models:\$PROMETHEUS_MODELS:\$PROMETHEUS_MODELS/texture:/usr/share/gz/gz-sim7/models:/usr/share/ignition/ignition-gazebo6/models
    PX4_GZ_WORLD='$WORLD_NAME' make px4_sitl gz_x500_mono_cam;
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

# 4. Start RQT Image View
echo "🖼️ Starting RQT Image View..."
if [ ! -d "/opt/ros/humble/lib/rqt_image_view" ]; then
    echo "❌ rqt_image_view not found. Install with:"
    echo "   sudo apt update && sudo apt install -y ros-humble-rqt-image-view ros-humble-image-view"
    echo "⚠️  Click-to-track requires rqt_image_view (image_view_msgs)."
else
    gnome-terminal --tab --title="🖼️ RQT Image View" -- bash -c "
        source /opt/ros/humble/setup.bash;
        export PATH=/opt/ros/humble/bin:\$PATH;
        rqt --standalone rqt_image_view --force-discover;
        exec bash
    " &
fi

sleep 2

# 5. Start native C++ QR Tracker Node
echo "🎯 Starting QR Tracker..."
gnome-terminal --tab --title="🎯 QR Tracker" -- bash -c "
    source /opt/ros/humble/setup.bash;
    source '$WS_PATH/install/setup.bash';
    echo '🚀 Launching qr_tracker...';
    # Parameters: target_id (ArUco ID), altitude (Track altitude)
    ros2 run px4_hexctl qr_tracker --ros-args -p target_id:=100 -p altitude:=1.5 -p auto_track:=false -p rqt_mouse_topic:=/image_view/mouse_event -p qgc_mouse_topic:=/qgc/mouse_event -p rqt_click_point_topic:=/image_view/click_point -p qgc_click_point_topic:=/qgc/click_point -p tracking_delta_x:=2.5 -p tracking_delta_y:=0.0 -p tracking_delta_z:=0.0 -p marker_size:=0.2;
    exec bash
" &

echo "✅ QR Tracking Demo successfully started!"
echo "📡 Tracker is listening to /camera topic"
