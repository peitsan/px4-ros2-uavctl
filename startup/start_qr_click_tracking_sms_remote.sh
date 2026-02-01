#!/bin/bash
# =========================================================
# SMS Click Tracking + PX4 Offboard (Remote/OrangePi)
# 启动顺序：
#   1. 远端 MicroXRCEAgent (香橙派)
#   2. 远端 SMS Click Tracking + Offboard (香橙派)
#   3. 本机 QGroundControl (可选)
# =========================================================

set -e

# === 本机配置 ===
LOCAL_WORKSPACE_PATH="$HOME/Desktop/px4-ros2-uavctl"
QGC_PATH="$HOME/bin/QGroundControl-x86_64.AppImage"
LOCAL_ROS_DISTRO="humble"

# === 远端香橙派配置 ===
REMOTE_USER="orangepi"
REMOTE_IP="192.168.3.17"
REMOTE_HOST="${REMOTE_USER}@${REMOTE_IP}"
REMOTE_WORKSPACE_PATH="/home/orangepi/uav_ws"
REMOTE_ROS_DISTRO="humble"

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

echo -e "${GREEN}🚁 SMS Click Tracking 远程启动脚本${NC}"
echo -e "${BLUE}远端地址: $REMOTE_HOST${NC}"

# === 依赖检查 ===
command -v ssh >/dev/null 2>&1 || { echo "❌ 错误: 本机需安装 SSH"; exit 1; }
command -v gnome-terminal >/dev/null 2>&1 || { echo "❌ 错误: 本机需安装 gnome-terminal"; exit 1; }

# === 连接检查 ===
if ! ssh -o ConnectTimeout=5 -o BatchMode=yes "${REMOTE_HOST}" "echo 'SSH连接成功'" > /dev/null 2>&1; then
    echo -e "${RED}❌ 无法连接到香橙派 ${REMOTE_HOST}${NC}"
    exit 1
fi

start_micro_agent() {
    echo -e "${YELLOW}=== 1️⃣ 启动远端 MicroXRCEAgent ===${NC}"
    CMD="source /opt/ros/$REMOTE_ROS_DISTRO/setup.bash && MicroXRCEAgent $AGENT_TRANSPORT -D $AGENT_PORT -b $AGENT_BAUDRATE"
    if [[ "$AGENT_TRANSPORT" == "udp4" ]]; then
        CMD="source /opt/ros/$REMOTE_ROS_DISTRO/setup.bash && MicroXRCEAgent udp4 -p $AGENT_PORT"
    fi

    gnome-terminal --tab --title="📡 MicroXRCEAgent (Remote)" -- bash -c "
        echo '📡 启动 MicroXRCEAgent...';
        ssh -t '${REMOTE_HOST}' '$CMD';
        echo 'Agent 已停止，按 Enter 关闭...'; read;
    " &
    AGENT_PID=$!
    sleep 3
}

start_sms_tracking() {
    echo -e "${YELLOW}=== 2️⃣ 启动远端 SMS Click Tracking + Offboard ===${NC}"
    
    # 远程启动命令：先 source 环境，再启动 launch 文件
    REMOTE_CMD="source /opt/ros/$REMOTE_ROS_DISTRO/setup.bash; \
                source $REMOTE_WORKSPACE_PATH/install/setup.bash; \
                export DISPLAY=:0; \
                ros2 launch px4_hexctl qr_click_tracking_sms.launch.py"

    gnome-terminal --tab --title="🎯 SMS Tracking (Remote)" -- bash -c "
        echo '🎯 连接香橙派并启动 SMS 跟随...';
        ssh -t '${REMOTE_HOST}' '$REMOTE_CMD';
        echo 'SMS 节点已停止，按 Enter 关闭...'; read;
    " &
    TRACK_PID=$!
    sleep 2
}

start_qgroundcontrol() {
    echo -e "${YELLOW}=== 3️⃣ 启动本机 QGroundControl ===${NC}"
    QGC_SCRIPT="/home/ubuntu/Desktop/QGroundControl.sh"
    if [[ -f "$QGC_SCRIPT" ]]; then
        gnome-terminal --tab --title="🛰️ QGroundControl" -- bash -c "bash '$QGC_SCRIPT'" &
        QGC_PID=$!
    fi
}

cleanup() {
    echo -e "\n${RED}=== 停止远程任务 ===${NC}"
    kill $AGENT_PID $TRACK_PID $QGC_PID 2>/dev/null || true
    ssh "${REMOTE_HOST}" "pkill -f 'MicroXRCEAgent\|qr_click_tracking_sms\|pvid\|pyolo\|pocvsot' 2>/dev/null || true" &
    exit 0
}
trap cleanup SIGINT SIGTERM

# === 执行流程 ===
start_micro_agent
start_sms_tracking

read -p "启动本机 QGroundControl? (y/n): " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    start_qgroundcontrol
fi

echo -e "\n${GREEN}✅ 远程任务已启动。按 Ctrl+C 停止所有进程。${NC}"
wait
