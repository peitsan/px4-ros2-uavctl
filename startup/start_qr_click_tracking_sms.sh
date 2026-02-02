#!/bin/bash
# =========================================================
# SMS Click Tracking + PX4 Offboard (ROS2)
# =========================================================

set -e

WS_PATH="$HOME/uav_ws"
ROS_DISTRO="humble"
LOCAL_ROS_DISTRO="$ROS_DISTRO"
# 本机可视化
START_LOCAL_RQT=true


# MicroXRCEAgent 配置
AGENT_TRANSPORT="serial"  # serial 或 udp4
AGENT_PORT="/dev/ttyUSB0"
AGENT_BAUDRATE="921600"


# === 颜色输出 ===
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

if command -v xfce4-terminal >/dev/null 2>&1; then
  TERMINAL_CMD="xfce4-terminal"
  TERMINAL_KIND="xfce"
elif command -v gnome-terminal >/dev/null 2>&1; then
  TERMINAL_CMD="gnome-terminal"
  TERMINAL_KIND="gnome"
else
  echo "❌ xfce4-terminal/gnome-terminal not found"
  exit 1
fi

open_terminal() {
    local title="$1"
    local body="$2"
    if [ "$TERMINAL_KIND" = "xfce" ]; then
        $TERMINAL_CMD --title="$title" --command="bash -c '$body'" &
    else
        $TERMINAL_CMD --tab --title="$title" -- bash -c "$body" &
    fi
}

start_micro_agent() {
    echo -e "${YELLOW}=== 1️⃣ 启动 MicroXRCEAgent ===${NC}"
    CMD="source /opt/ros/$ROS_DISTRO/setup.bash && MicroXRCEAgent $AGENT_TRANSPORT -D $AGENT_PORT -b $AGENT_BAUDRATE"
    if [[ "$AGENT_TRANSPORT" == "udp4" ]]; then
        CMD="source /opt/ros/$ROS_DISTRO/setup.bash && MicroXRCEAgent udp4 -p $AGENT_PORT"
    fi

    open_terminal "📡 MicroXRCEAgent" "
      echo '📡 启动 MicroXRCEAgent...';
      $CMD;
      echo 'Agent 已停止，按 Enter 关闭...'; read;
    "
    AGENT_PID=$!
    sleep 3
}

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

start_sms_tracking() {
    # 1) Start Realsense + SMS pipeline + ROS2 nodes
    # NOTE: rqt_image_view（GUI 在本机启动）
    open_terminal "🚀 SMS Click Tracking" "
      source /opt/ros/$ROS_DISTRO/setup.bash;
      source $WS_PATH/install/setup.bash;
      ros2 launch realsense_camera rs_lt_launch.py &
      ros2 launch px4_hexctl qr_click_tracking_sms.launch.py start_image_view:=true;
      exec bash
    "
    TRACK_PID=$!
    sleep 2
}

start_local_rqt_image_view() {
    if [ "$START_LOCAL_RQT" != true ]; then
        return
    fi
    open_terminal "🖼️ RQT Image View" "
        source /opt/ros/$LOCAL_ROS_DISTRO/setup.bash;
        rqt --standalone rqt_image_view --force-discover;
        exec bash
    "
    RQT_PID=$!
    sleep 1
}
    

echo "✅ SMS click tracking launch started"


cleanup() {
    echo -e "\n${RED}=== 停止远程任务 ===${NC}"
  kill $AGENT_PID $TRACK_PID $RQT_PID 2>/dev/null || true
    exit 0
}
trap cleanup SIGINT SIGTERM

# === 执行流程 ===

start_micro_agent
start_local_rqt_image_view
start_sms_tracking