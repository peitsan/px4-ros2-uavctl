#!/bin/bash
# =========================================================
# Real Robot QR Tracking (SpireCV + PX4 Offboard)
# =========================================================

set -e

WS_PATH="$HOME/Desktop/px4-ros2-uavctl"
ROS_DISTRO="humble"

RGB_TOPIC="/camera/d435i/color/image_raw"
DEPTH_TOPIC="/camera/d435i/depth/image_rect_raw"

TARGET_ID="1"
TAKEOFF_ALT="1.5"
TRACKING_DELTA_X="2.5"
TRACKING_DELTA_Y="0.0"
TRACKING_DELTA_Z="0.0"
TRACK_Z="false"

source /opt/ros/$ROS_DISTRO/setup.bash
if [ -f "$WS_PATH/install/setup.bash" ]; then
  source "$WS_PATH/install/setup.bash"
else
  echo "❌ workspace not built: $WS_PATH/install/setup.bash not found"
  exit 1
fi

command -v gnome-terminal >/dev/null 2>&1 || { echo "❌ gnome-terminal not found"; exit 1; }

# 1) Launch pipeline (RealSense + Offboard + SpireCV)
gnome-terminal --tab --title="🚀 QR Tracking Launch" -- bash -c "
  source /opt/ros/$ROS_DISTRO/setup.bash;
  source $WS_PATH/install/setup.bash;
  ros2 launch px4_hexctl qr_tracking_realrobot.launch.py;
  exec bash
" &

echo "✅ Real robot QR tracking launch started"
