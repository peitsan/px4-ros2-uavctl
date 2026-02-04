#!/bin/bash
# =========================================================
# VINS-Fusion + PX4 Offboard Hover Control
# 启动完整的VINS+PX4自动悬停系统
# =========================================================

set -e

# === Configuration ===
WS_PATH="/home/ubuntu/Desktop/px4-ros2-uavctl"
VINS_CONFIG="$WS_PATH/src/VINS-Fusion-ROS2/config/realsense_d435i/realsense_stereo_imu_config.yaml"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}

# === Colors ===
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m'

# === Display Header ===
echo ""
echo "╔═══════════════════════════════════════════════════════════╗"
echo "║   VINS-Fusion + PX4 Offboard Hover Control System        ║"
echo "║   D435i SLAM + Autonomous Flight Control                ║"
echo "╚═══════════════════════════════════════════════════════════╝"
echo ""

# === Cleanup ===
# echo -e "${YELLOW}🧹 Cleaning up previous instances...${NC}"
# pkill -f "vins_node" 2>/dev/null || true
# pkill -f "offboard_hover" 2>/dev/null || true
# sleep 2

# === Setup ROS2 Environment ===
cd "$WS_PATH"
source /opt/ros/humble/setup.bash
source ./src/VINS-Fusion-ROS2/install/setup.bash
source ./install/setup.bash  # Source local workspace

echo -e "${GREEN}✅ ROS2 environment sourced${NC}"

# === Start VINS-Fusion Node ===
echo ""
echo -e "${BLUE}🚀 Starting VINS-Fusion node...${NC}"
echo "   📹 Sensor: Intel RealSense D435i"
echo "   🎯 Input topics:"
echo "      - /camera/color/image_raw"
echo "      - /camera/depth/image_rect_raw"
echo "      - /camera/infra1/image_rect_raw"
echo "      - /camera/infra2/image_rect_raw"
echo "      - /camera/imu"
echo "   📊 Output: /vins_estimator/odometry (VIO pose)"
echo ""

# Launch VINS in background
cd "$WS_PATH"
ros2 run vins vins_node "$VINS_CONFIG" &
VINS_PID=$!
sleep 5

# === Start Offboard Hover Controller ===
echo ""
echo -e "${BLUE}🚀 Starting Offboard Hover Controller...${NC}"
echo "   ✈️  Control: PX4 SITL via MicroXRCEAgent"
echo "   📍 Pose source: VINS odometry"
echo "   🎯 Behavior: Auto-hover from VINS odometry"
echo ""

# Launch offboard controller in background
ros2 run px4_hexctl vins_offboard_hover &
OFFBOARD_PID=$!
sleep 2

echo ""
echo -e "${GREEN}✅ System Started Successfully${NC}"
echo ""
echo "Active Topics:"
ros2 topic list | grep -E "(camera|vins|fmu)" | sed 's/^/  📡 /'
echo ""
echo -e "${YELLOW}Monitoring system (Press Ctrl+C to stop)...${NC}"
echo ""

# === Monitor both processes ===
wait $VINS_PID $OFFBOARD_PID
