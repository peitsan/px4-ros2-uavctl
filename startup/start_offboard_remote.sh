#!/bin/bash
# ================================================================
# 直接启动 Offboard Control 节点
# 在远端(香橙派)上运行此脚本来启动 Offboard Control
# ================================================================

REMOTE_HOST="orangepi@192.168.5.163"
WORKSPACE_PATH="/home/orangepi/uav_ws"

echo "🚀 正在远端启动 Offboard Control 节点..."
echo "   主机: $REMOTE_HOST"
echo "   工作空间: $WORKSPACE_PATH"
echo ""

# SSH 到远端并启动节点
ssh -t "${REMOTE_HOST}" "
    echo '🔄 设置 ROS2 环境...'
    source /opt/ros/humble/setup.bash
    source ~/uav_ws/install/setup.bash
    
    echo '📍 启动 Offboard Control 节点...'
    ros2 run px4_hexctl offboard_circle
"
