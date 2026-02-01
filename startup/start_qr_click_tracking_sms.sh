#!/bin/bash
# =========================================================
# SMS Click Tracking + PX4 Offboard (ROS2)
# =========================================================

set -e

WS_PATH="$HOME/Desktop/px4-ros2-uavctl"
ROS_DISTRO="humble"

source /opt/ros/$ROS_DISTRO/setup.bash
if [ -f "$WS_PATH/install/setup.bash" ]; then
  source "$WS_PATH/install/setup.bash"
else
  echo "❌ workspace not built: $WS_PATH/install/setup.bash not found"
  exit 1
fi

# Ensure python command exists for smsrun
if ! command -v python >/dev/null 2>&1; then
  if command -v python3 >/dev/null 2>&1; then
    sudo ln -s "$(command -v python3)" /usr/local/bin/python || true
  fi
fi

command -v gnome-terminal >/dev/null 2>&1 || { echo "❌ gnome-terminal not found"; exit 1; }

# 1) Start SMS pipeline + ROS2 nodes (single launch)
gnome-terminal --tab --title="🚀 SMS Click Tracking" -- bash -c "
  source /opt/ros/$ROS_DISTRO/setup.bash;
  source install/setup.bash;
  ros2 launch px4_hexctl qr_click_tracking_sms.launch.py;
  exec bash
" &

echo "✅ SMS click tracking launch started"
