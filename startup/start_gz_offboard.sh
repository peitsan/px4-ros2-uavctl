#!/bin/bash
# ================================================================
# 直接启动 Offboard Control 节点
# 在本地上运行此脚本来启动 Offboard Control
# ================================================================

WORKSPACE_PATH="/home/ubuntu/Desktop/px4-ros2-uavctl"

echo "🚀 正在本地启动 Offboard Control 节点..."
echo "   工作空间: $WORKSPACE_PATH"
echo ""

# SSH 到远端并启动节点
echo '🔄 设置 ROS2 环境...'
source /opt/ros/humble/setup.bash
source ${WORKSPACE_PATH}/install/setup.bash
    
echo '📍 启动 Offboard Control 节点...'
ros2 run px4_hexctl offboard_control_main