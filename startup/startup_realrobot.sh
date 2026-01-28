#!/bin/bash
# =========================================================
# PX4 实机无人机控制启动脚本（本机运行，远程控制香橙派）
# 启动顺序：
#   1. 远端 MicroXRCEAgent (香橙派)
#   2. 远端 Offboard Control (香橙派)
#   3. 本机 QGroundControl (地面站)
# =========================================================

set -e  # 遇到错误时退出

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

# MicroXRCEAgent 配置（在香橙派上运行）
AGENT_TRANSPORT="serial"  # serial 或 udp4
AGENT_PORT="/dev/ttyUSB0"  # 串口设备，或 UDP 端口号
AGENT_BAUDRATE="115200"    # 串口波特率

# === 颜色输出 ===
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# === 依赖检查 ===
echo "🔍 检查依赖..."
command -v ssh >/dev/null 2>&1 || { echo "错误: 本机需要安装 SSH"; exit 1; }
command -v gnome-terminal >/dev/null 2>&1 || { echo "错误: 本机需要安装 gnome-terminal"; exit 1; }

# 检查与香橙派的连接
echo "🔌 检查与香橙派的连接..."
if ! ssh -o ConnectTimeout=5 -o BatchMode=yes "${REMOTE_HOST}" "echo 'SSH连接成功'" > /dev/null 2>&1; then
    echo "错误: 无法连接到香橙派 ${REMOTE_HOST}"
    echo "请检查网络连接和SSH配置"
    exit 1
fi
echo "✅ SSH连接正常"

# === 启动函数 ===
start_micro_agent() {
    echo -e "${YELLOW}=== 1️⃣ 启动远端 MicroXRCEAgent (香橙派) ===${NC}"
    
    if [[ "$AGENT_TRANSPORT" == "serial" ]]; then
        echo -e "${BLUE}通过 SSH 在香橙派上启动 MicroXRCEAgent (Serial $AGENT_PORT @ $AGENT_BAUDRATE)${NC}"
        
        gnome-terminal --tab --title="📡 MicroXRCEAgent (Remote Serial)" -- bash -c "
            echo '📡 连接香橙派并启动 MicroXRCEAgent...';
            echo '地址: $REMOTE_HOST';
            echo '波特率: $AGENT_BAUDRATE';
            ssh -t '${REMOTE_HOST}' 'source /opt/ros/$REMOTE_ROS_DISTRO/setup.bash && MicroXRCEAgent serial -D $AGENT_PORT -b $AGENT_BAUDRATE';
            echo 'Agent 已停止，按 Enter 关闭窗口...';
            read;
        " &
        AGENT_PID=$!
        
    elif [[ "$AGENT_TRANSPORT" == "udp4" ]]; then
        echo -e "${BLUE}通过 SSH 在香橙派上启动 MicroXRCEAgent (UDP $AGENT_PORT)${NC}"
        
        gnome-terminal --tab --title="📡 MicroXRCEAgent (Remote UDP)" -- bash -c "
            echo '📡 连接香橙派并启动 MicroXRCEAgent...';
            echo '地址: $REMOTE_HOST';
            echo 'UDP 端口: $AGENT_PORT';
            ssh -t '${REMOTE_HOST}' 'source /opt/ros/$REMOTE_ROS_DISTRO/setup.bash && MicroXRCEAgent udp4 -p $AGENT_PORT';
            echo 'Agent 已停止，按 Enter 关闭窗口...';
            read;
        " &
        AGENT_PID=$!
        
    else
        echo -e "${RED}错误: 不支持的传输方式: $AGENT_TRANSPORT${NC}"
        echo "支持的方式: serial, udp4"
        exit 1
    fi
    
    echo -e "${GREEN}MicroXRCEAgent 启动中 (PID: $AGENT_PID)${NC}"
    echo -e "${BLUE}等待 Agent 启动完成 (5秒)...${NC}"
    sleep 5
}

start_offboard_control() {
    echo -e "${YELLOW}=== 2️⃣ 启动远端 Offboard Control (香橙派) ===${NC}"
    
    echo -e "${BLUE}通过 SSH 在香橙派上启动 Offboard Control 节点${NC}"
    
    gnome-terminal --tab --title="🛸 Offboard Control (Remote)" -- bash -c "
        echo '🛸 连接香橙派并启动 Offboard Control 节点...';
        echo '地址: $REMOTE_HOST';
        echo '等待 PX4 话题数据...';
        sleep 2;
        ssh -t '${REMOTE_HOST}' 'source /opt/ros/$REMOTE_ROS_DISTRO/setup.bash && source $REMOTE_WORKSPACE_PATH/install/setup.bash && sleep 3 && ros2 run offboard_control_cpp offboard_control_main';
        echo 'Offboard Control 已停止，按 Enter 关闭窗口...';
        read;
    " &
    OFFBOARD_PID=$!
    echo -e "${GREEN}Offboard Control 启动中 (PID: $OFFBOARD_PID)${NC}"
    sleep 2
}

start_qgroundcontrol() {
    echo -e "${YELLOW}=== 3️⃣ 启动本机 QGroundControl 地面站 ===${NC}"
    
    QGC_SCRIPT="/home/ubuntu/Desktop/QGroundControl.sh"
    
    if [[ -f "$QGC_SCRIPT" ]]; then
        echo -e "${BLUE}使用脚本启动 QGroundControl 地面站（本机）${NC}"
        gnome-terminal --tab --title="🛰️ QGroundControl" -- bash -c "
            echo '🛰️ 启动 QGroundControl 地面站';
            echo '连接到 PX4 实机...';
            bash '$QGC_SCRIPT';
            echo 'QGC 已关闭';
        " &
        QGC_PID=$!
        echo -e "${GREEN}QGroundControl 启动中 (PID: $QGC_PID)${NC}"
    else
        echo -e "${YELLOW}⚠️ 未找到 QGroundControl 启动脚本: $QGC_SCRIPT${NC}"
        echo -e "${YELLOW}跳过 QGC 启动${NC}"
        QGC_PID=""
    fi
    sleep 2
}

# === 清理函数 ===
cleanup() {
    echo -e "\n${RED}=== 停止实机控制环境 ===${NC}"
    echo "终止进程: Agent($AGENT_PID), Offboard($OFFBOARD_PID), QGC($QGC_PID)"
    
    # 终止本地进程
    kill $AGENT_PID $OFFBOARD_PID $QGC_PID 2>/dev/null || true
    
    # 在远端停止进程
    echo "在香橙派上停止进程..."
    ssh "${REMOTE_HOST}" "pkill -f 'MicroXRCEAgent\|offboard_control_main' 2>/dev/null || true" &
    
    pkill -f "QGroundControl\|gnome-terminal" 2>/dev/null || true
    echo -e "${GREEN}清理完成${NC}"
    exit 0
}

# 捕获 Ctrl+C
trap cleanup SIGINT SIGTERM

# === 主启动流程 ===
echo -e "${GREEN}🚁 PX4 实机无人机控制启动（本机运行）${NC}"
echo -e "${BLUE}本机 ROS 版本: $LOCAL_ROS_DISTRO${NC}"
echo -e "${BLUE}远端地址: $REMOTE_HOST | ROS 版本: $REMOTE_ROS_DISTRO${NC}"
echo -e "${BLUE}传输方式: $AGENT_TRANSPORT${NC}"
if [[ "$AGENT_TRANSPORT" == "serial" ]]; then
    echo -e "${BLUE}串口设备: $AGENT_PORT @ $AGENT_BAUDRATE bps${NC}"
else
    echo -e "${BLUE}UDP 端口: $AGENT_PORT${NC}"
fi
echo "================================================================"

# 1. 启动远端 MicroXRCEAgent
start_micro_agent

# 2. 启动远端 Offboard Control
read -p "是否启动 Offboard Control 节点? (y/n): " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    start_offboard_control
fi

# 3. 启动本机 QGroundControl 地面站
read -p "是否启动 QGroundControl 地面站? (y/n): " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    start_qgroundcontrol
fi

# === 验证和提示 ===
echo -e "\n${GREEN}✅ 所有组件启动完成！${NC}"
echo -e "${YELLOW}=== 验证检查 ===${NC}"
echo "1. 在香橙派上检查 PX4 话题:"
echo "   ssh $REMOTE_HOST 'source /opt/ros/$REMOTE_ROS_DISTRO/setup.bash && ros2 topic list | grep fmu'"
echo "2. 在香橙派上检查话题数据:"
echo "   ssh $REMOTE_HOST 'source /opt/ros/$REMOTE_ROS_DISTRO/setup.bash && source $REMOTE_WORKSPACE_PATH/install/setup.bash && ros2 topic echo /fmu/out/vehicle_status'"
echo "3. 在香橙派上检查 Offboard 节点:"
echo "   ssh $REMOTE_HOST 'source /opt/ros/$REMOTE_ROS_DISTRO/setup.bash && ros2 node list | grep offboard'"

echo -e "\n${BLUE}=== 重要提示 ===${NC}"
echo "⚠️  确保飞控已通电并正确连接到香橙派的 $AGENT_PORT"
echo "⚠️  起飞前检查电池电量、GPS 信号、传感器状态"
echo "⚠️  测试时保持安全距离，准备紧急停止"
echo "⚠️  首次飞行建议在 QGC 中手动解锁并测试"

echo -e "\n${RED}停止脚本: Ctrl+C 或关闭所有终端${NC}"
echo -e "${YELLOW}远端日志查看: ssh $REMOTE_HOST 'tail -f ~/.ros/log/latest/*.log'${NC}"
echo -e "${YELLOW}本机日志查看: tail -f ~/.ros/log/latest/*.log${NC}"

# 保持脚本运行，监控进程
echo -e "\n${BLUE}监控进程状态 (按 Ctrl+C 停止)...${NC}"
wait
