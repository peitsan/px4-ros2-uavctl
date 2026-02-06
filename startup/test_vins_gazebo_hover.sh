#!/bin/bash
# =========================================================
# VINS-Gazebo集成测试: 完整的D435i定点飞行系统
# 启动顺序:
#  1. PX4 SITL + Gazebo (warehouse_d435i world)
#  2. MicroXRCEAgent (PX4<->ROS2 bridge)
#  3. VINS-Fusion node (视觉里程计)
#  4. Offboard hover controller (定点控制)
# =========================================================

set -e

# === Configuration ===
WS_PATH="/home/ubuntu/Desktop/px4-ros2-uavctl"
PX4_PATH="/home/ubuntu/PX4-Autopilot"
VINS_CONFIG="$WS_PATH/third_party/VINS-Fusion-ROS2/config/realsense_d435i/realsense_stereo_imu_config.yaml"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
AGENT_PORT=${AGENT_PORT:-4560}

# === Colors ===
RED='\033[0;31m'
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

# === Utility Functions ===
log_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

log_step() {
    echo ""
    echo -e "${CYAN}╔════════════════════════════════════════════════════════╗${NC}"
    echo -e "${CYAN}║${NC} $1"
    echo -e "${CYAN}╚════════════════════════════════════════════════════════╝${NC}"
    echo ""
}

cleanup() {
    log_warn "🛑 Shutting down system..."
    
    # Kill all processes in order
    pkill -f "MicroXRCEAgent" 2>/dev/null || true
    pkill -f "vins_node" 2>/dev/null || true
    pkill -f "offboard_hover" 2>/dev/null || true
    pkill -f "gz sim" 2>/dev/null || true
    pkill -f "px4" 2>/dev/null || true
    
    sleep 2
    log_info "System shutdown complete"
}

trap cleanup EXIT

# =========================================================
# STEP 1: Clean up previous instances
# =========================================================
log_step "STEP 1/5: Cleanup Previous Instances"

pkill -f "MicroXRCEAgent" 2>/dev/null || true
pkill -f "vins_node" 2>/dev/null || true
pkill -f "offboard_hover" 2>/dev/null || true
pkill -f "gz sim" 2>/dev/null || true
pkill -f "px4" 2>/dev/null || true

sleep 2
log_info "✅ Cleanup complete"

# =========================================================
# STEP 2: Setup ROS2 Environment
# =========================================================
log_step "STEP 2/5: Setup ROS2 Environment"

cd "$WS_PATH"
source /opt/ros/humble/setup.bash
source ./third_party/VINS-Fusion-ROS2/install/setup.bash
source ./install/setup.bash

log_info "✅ ROS2 Humble environment sourced"
log_info "✅ VINS-Fusion environment sourced"
log_info "✅ PX4-ROS2-UAVCTL environment sourced"

# =========================================================
# STEP 3: Start PX4 SITL with Gazebo
# =========================================================
log_step "STEP 3/5: Start PX4 SITL with Gazebo"

log_info "Starting PX4 SITL with warehouse_d435i world..."
log_info "🚁 Platform: X500 with RealSense D435i"
log_info "🌍 Environment: warehouse_d435i.sdf (40×40m)"
log_info ""

cd "$PX4_PATH"
export PX4_SIM_MODEL=x500_realsense_d435i
export PX4_GZ_WORLD=warehouse_d435i

# Start PX4 SITL in background
make px4_sitl gz_x500_realsense_d435i > /tmp/px4_sitl.log 2>&1 &
PX4_PID=$!

log_info "PX4 PID: $PX4_PID"
log_info "⏳ Waiting for PX4 initialization (15 seconds)..."

# Wait for PX4 to initialize
for i in {1..15}; do
    echo -ne "\r   [$(printf '%2d' $i)/15] Initializing..."
    sleep 1
done
echo ""

if ps -p $PX4_PID > /dev/null; then
    log_info "✅ PX4 SITL running"
else
    log_error "PX4 SITL failed to start!"
    cat /tmp/px4_sitl.log
    exit 1
fi

# =========================================================
# STEP 4: Start MicroXRCEAgent (PX4 <-> ROS2 Bridge)
# =========================================================
log_step "STEP 4/5: Start MicroXRCEAgent (PX4<->ROS2 Bridge)"

log_info "🌉 Starting MicroXRCEAgent on port $AGENT_PORT..."
log_info "   Bridge: PX4 SITL ←→ ROS2 Humble"
log_info ""

# Start MicroXRCEAgent in background
MicroXRCEAgent udp4 -p $AGENT_PORT > /tmp/agent.log 2>&1 &
AGENT_PID=$!

log_info "Agent PID: $AGENT_PID"
sleep 3

if ps -p $AGENT_PID > /dev/null; then
    log_info "✅ MicroXRCEAgent running"
else
    log_error "MicroXRCEAgent failed to start!"
    cat /tmp/agent.log
    exit 1
fi

# Wait for agent to establish connection
log_info "⏳ Waiting for agent connection (5 seconds)..."
sleep 5
log_info "✅ Agent bridge established"

# =========================================================
# STEP 5: Start VINS-Fusion + Offboard Hover
# =========================================================
log_step "STEP 5/5: Start VINS-Fusion & Offboard Hover"

echo ""
log_info "📹 Starting VINS-Fusion node..."
log_info "   Camera: Intel RealSense D435i"
log_info "   Config: $VINS_CONFIG"
log_info ""

cd "$WS_PATH"
ros2 run vins vins_node "$VINS_CONFIG" > /tmp/vins.log 2>&1 &
VINS_PID=$!

log_info "VINS PID: $VINS_PID"
log_info "⏳ Waiting for VINS initialization (8 seconds)..."
sleep 8

if ps -p $VINS_PID > /dev/null; then
    log_info "✅ VINS-Fusion running"
else
    log_error "VINS-Fusion failed to start!"
    tail -20 /tmp/vins.log
fi

echo ""
log_info "🚁 Starting Offboard Hover Controller..."
log_info "   Input: /vins_estimator/odometry (VINS VIO)"
log_info "   Output: PX4 trajectory setpoints"
log_info ""

ros2 run px4_hexctl vins_offboard_hover > /tmp/offboard.log 2>&1 &
OFFBOARD_PID=$!

log_info "Offboard PID: $OFFBOARD_PID"
sleep 3

if ps -p $OFFBOARD_PID > /dev/null; then
    log_info "✅ Offboard Hover Controller running"
else
    log_error "Offboard Hover Controller failed to start!"
    tail -20 /tmp/offboard.log
fi

# =========================================================
# System Status
# =========================================================
log_step "✅ COMPLETE: VINS-Gazebo Hover System Online"

echo -e "${GREEN}Process Status:${NC}"
echo "  PID $PX4_PID     : PX4 SITL"
echo "  PID $AGENT_PID   : MicroXRCEAgent"
echo "  PID $VINS_PID    : VINS-Fusion"
echo "  PID $OFFBOARD_PID: Offboard Hover"
echo ""

echo -e "${GREEN}Active ROS2 Topics:${NC}"
ros2 topic list | grep -E "(camera|vins|fmu|vehicle)" | sed 's/^/  📡 /' | head -20
echo ""

echo -e "${GREEN}Next Steps:${NC}"
echo "  1. Monitor VINS odometry:      ros2 topic echo /vins_estimator/odometry"
echo "  2. Monitor offboard setpoints: ros2 topic echo /fmu/in/trajectory_setpoint"
echo "  3. Check vehicle status:       ros2 topic echo /fmu/out/vehicle_status"
echo "  4. View Gazebo simulation:     (already open)"
echo ""

echo -e "${YELLOW}Control sequence:${NC}"
echo "  - Vehicle auto-arms in 5 seconds"
echo "  - Offboard mode enabled"
echo "  - Automatic hover at 1.5m altitude"
echo "  - Hover duration: 30 seconds"
echo "  - Auto-landing after timeout"
echo ""

echo -e "${CYAN}Press Ctrl+C to shutdown all processes${NC}"
echo ""

# =========================================================
# Monitor System
# =========================================================
# Wait for all processes
wait $PX4_PID $AGENT_PID $VINS_PID $OFFBOARD_PID 2>/dev/null || true
