#!/bin/bash
# =========================================================
# SMS Click Tracking SITL Startup Script (ROS2)
# Starts: MicroXRCEAgent, PX4 SITL (Gazebo), SMS Pipeline
# =========================================================

set -e

# === Configuration ===
PX4_PATH="/home/ubuntu/PX4-Autopilot"
WS_PATH="/home/ubuntu/Desktop/px4-ros2-uavctl"
WORLD_SOURCE="$WS_PATH/world/aruco_6X6_250.world"
WORLD_NAME="aruco_6X6_250"
PROMETHEUS_MODELS="$HOME/Prometheus/Simulator/gazebo_simulator/gazebo_models"

# Ensure Environment
source /opt/ros/humble/setup.bash
if [ -f "$WS_PATH/install/setup.bash" ]; then
    source "$WS_PATH/install/setup.bash"
else
    echo "❌ Workspace not built. Please build first."
    exit 1
fi

# 1. Start MicroXRCEAgent
echo "📡 Starting MicroXRCEAgent..."
gnome-terminal --tab --title="MicroXRCEAgent" -- bash -c "
    MicroXRCEAgent udp4 -p 8888;
    exec bash
" &

sleep 2

# 2. Start PX4 SITL + Gazebo
echo "🛫 Starting PX4 SITL + Gazebo..."
if [ ! -d "$PX4_PATH" ]; then
    echo "❌ PX4 not found at $PX4_PATH"
    exit 1
fi

gnome-terminal --tab --title="PX4 SITL" -- bash -c "
    # Copy world file to PX4 worlds
    mkdir -p $PX4_PATH/Tools/simulation/gz/worlds
    cp '$WORLD_SOURCE' '$PX4_PATH/Tools/simulation/gz/worlds/${WORLD_NAME}.sdf'
    
    cd '$PX4_PATH'
    # Set model path to include Prometheus models if available
    export GZ_SIM_RESOURCE_PATH=\$GZ_SIM_RESOURCE_PATH:\$PX4_PATH/Tools/simulation/gz/models:\$PROMETHEUS_MODELS:\$PROMETHEUS_MODELS/texture:/usr/share/gz/gz-sim7/models
    
    # Start SITL with x500 and mono camera
    PX4_GZ_WORLD='$WORLD_NAME' make px4_sitl gz_x500_mono_cam
    exec bash
" &

sleep 10

# 3. Start SMS Click Tracking Setup
echo "🎯 Starting SMS Click Tracking..."
# Note: SMS pvid loads a video file by default. 
# For true SITL loop, SMS tools would need to ingest the ROS topic /camera 
# or the video file needs to be replaced by a stream.
# Assuming current instructions are to run the SMS demo controlling the SITL drone.

gnome-terminal --tab --title="SMS Tracking" -- bash -c "
    source /opt/ros/humble/setup.bash
    source '$WS_PATH/install/setup.bash'
    
    # Launch the SMS pipeline (pvid -> detect -> track -> bridge -> core)
    ros2 launch px4_hexctl qr_click_tracking_sms.launch.py
    exec bash
" &

# 4. Bridge GZ camera to ROS (Optional, for visualization/comparison)
gnome-terminal --tab --title="ROS-GZ Bridge" -- bash -c "
    source /opt/ros/humble/setup.bash
    ros2 run ros_gz_bridge parameter_bridge /camera@sensor_msgs/msg/Image@gz.msgs.Image
    exec bash
" &

echo "✅ SITL SMS Click Tracking Loop started."
