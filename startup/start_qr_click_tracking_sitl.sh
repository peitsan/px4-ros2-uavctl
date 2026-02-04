#!/bin/bash
# =========================================================
# SMS Click Tracking SITL Startup Script (ROS2)
# Starts: MicroXRCEAgent, PX4 SITL (Gazebo gz-sim7), SMS Pipeline
# Updated for gz-sim7 compatibility (replaces ign-gazebo)
# =========================================================

set -e

# === Configuration ===
PX4_PATH="/home/ubuntu/PX4-Autopilot"
WS_PATH="/home/ubuntu/Desktop/px4-ros2-uavctl"
WORLD_SOURCE="$WS_PATH/world/aruco_6X6_250.world"
WORLD_NAME="aruco_6X6_250"
PROMETHEUS_MODELS="$HOME/Prometheus/Simulator/gazebo_simulator/gazebo_models"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
export GZ_PARTITION=px4

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

# Cleanup previous instances
pkill -x px4 || true
pkill -f gz-sim || true
pkill -f ruby || true
pkill -f ros_gz_bridge || true
pkill -f ros_gz_image || true
pkill -f image_transport || true
pkill -f gz_camera_bridge || true

gnome-terminal --tab --title="PX4 SITL" -- bash -c "
    # Copy world file to PX4 worlds
    mkdir -p \"$PX4_PATH/Tools/simulation/gz/worlds\"
    cp \"$WORLD_SOURCE\" \"$PX4_PATH/Tools/simulation/gz/worlds/\${WORLD_NAME}.sdf\"
    
    cd \"$PX4_PATH\"
    # Set model path for gz-sim7 (replaces ign-gazebo paths)
    export GZ_SIM_RESOURCE_PATH=\$GZ_SIM_RESOURCE_PATH:\"$PX4_PATH/Tools/simulation/gz/models\":\"$PX4_PATH/Tools/simulation/gz/worlds\":\"$PROMETHEUS_MODELS\":\"$PROMETHEUS_MODELS/texture\":/usr/share/gz/gz-sim7/models
    export GZ_PARTITION=$GZ_PARTITION
    export ROS_DOMAIN_ID=$ROS_DOMAIN_ID
    
    # Start SITL with monocular camera model (gz-sim7 compatible)
    export PX4_SYS_AUTOSTART=4001
    export PX4_GZ_WORLD=\"$WORLD_NAME\"
    make px4_sitl gz_x500_mono_cam
    exec bash
" &

sleep 10

# 3. Start SMS Click Tracking Setup
echo "🎯 Starting SMS Click Tracking..."
# Note: SMS pvid loads a video file by default. 
# For true SITL loop, SMS tools would need to ingest the ROS topic /camera 
# or the video file needs to be replaced by a stream.
# Assuming current instructions are to run the SMS demo controlling the SITL drone.

# gnome-terminal --tab --title="SMS Tracking" -- bash -c "
#     source /opt/ros/humble/setup.bash
#     source '$WS_PATH/install/setup.bash'
    
#     # Launch the SMS pipeline (pvid -> detect -> track -> bridge -> core)
#     ros2 launch px4_hexctl qr_click_tracking_sms.launch.py
#     exec bash
# " &

# 4. Direct GZ->ROS camera bridge (no ros_gz dependency)
gnome-terminal --tab --title="GZ Camera Bridge" -- bash -c "
    source /opt/ros/humble/setup.bash
    export GZ_PARTITION=$GZ_PARTITION
    export ROS_DOMAIN_ID=$ROS_DOMAIN_ID
    python3 '$WS_PATH/startup/gz_camera_bridge.py'
    exec bash
" &

# 5. RQT Image View (Optional)
gnome-terminal --tab --title="RQT Image View" -- bash -c "
    source /opt/ros/humble/setup.bash
    export ROS_DOMAIN_ID=$ROS_DOMAIN_ID
    rqt --standalone rqt_image_view
    exec bash
" &

echo "✅ SITL SMS Click Tracking Loop started."
