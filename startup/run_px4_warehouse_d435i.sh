#!/bin/bash
# =========================================================
# SMS Click Tracking SITL Startup Script (ROS2)
# Starts: MicroXRCEAgent, PX4 SITL (Gazebo gz-sim7), SMS Pipeline
# Updated for gz-sim7 compatibility (replaces ign-gazebo)
# =========================================================


set -e  # 遇到错误时退出

# === 本机配置 ===
LOCAL_WORKSPACE_PATH="$HOME/Desktop/px4-ros2-uavctl"
QGC_PATH="$HOME/bin/QGroundControl-x86_64.AppImage"
LOCAL_ROS_DISTRO="humble"

# === Configuration ===
PX4_PATH="/home/ubuntu/PX4-Autopilot"
PX4_SIM_MODEL="x500_realsense_d435i"
PX4_GZ_WORLD="warehouse_d435i"  
WS_PATH="/home/ubuntu/Desktop/px4-ros2-uavctl"
WORLD_NAME="aruco_6X6_250"
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
    cd \"$PX4_PATH\"
    # Start SITL with monocular camera model (gz-sim7 compatible)
    export PX4_SYS_AUTOSTART=4001
    export PX4_GZ_WORLD=\"$PX4_GZ_WORLD\"
    make px4_sitl gz_x500_realsense_d435i
    exec bash
" &

sleep 10


# 4. Direct GZ->ROS camera bridge (local D435i topics)
gnome-terminal --tab --title="GZ Camera Bridge" -- bash -c "
    source /opt/ros/humble/setup.bash
    source ~/ros_gz_ws/install/setup.bash
    ros2 run ros_gz_bridge parameter_bridge \
        /camera/color/image_raw@sensor_msgs/msg/Image[gz.msgs.Image \
        /camera/depth/image_rect_raw@sensor_msgs/msg/Image[gz.msgs.Image \
        /camera/infra1/image_rect_raw@sensor_msgs/msg/Image[gz.msgs.Image \
        /camera/infra2/image_rect_raw@sensor_msgs/msg/Image[gz.msgs.Image \
        /camera/imu@sensor_msgs/msg/Imu[gz.msgs.IMU
" &

# 5. RQT Image View (Optional)
gnome-terminal --tab --title="RQT Image View" -- bash -c "
    source /opt/ros/humble/setup.bash
    export ROS_DOMAIN_ID=$ROS_DOMAIN_ID
    rqt --standalone rqt_image_view
    exec bash
" &

echo "✅ SITL SMS Click Tracking Loop started."
