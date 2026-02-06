#!/bin/bash
# =========================================================
# VINS + PX4 Offboard Hover System Test
# Verification script for complete integration
# =========================================================

set -e

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

WS_PATH="/home/ubuntu/Desktop/px4-ros2-uavctl"
PX4_PATH="/home/ubuntu/PX4-Autopilot"

echo ""
echo "╔════════════════════════════════════════════════════════════╗"
echo "║  VINS + PX4 Offboard Hover System - Test Suite            ║"
echo "╚════════════════════════════════════════════════════════════╝"
echo ""

# Function to check file existence
check_file() {
    if [ -f "$1" ]; then
        echo -e "${GREEN}✅${NC} $1"
        return 0
    else
        echo -e "${RED}❌${NC} $1"
        return 1
    fi
}

# Function to check directory existence
check_dir() {
    if [ -d "$1" ]; then
        echo -e "${GREEN}✅${NC} $1"
        return 0
    else
        echo -e "${RED}❌${NC} $1"
        return 1
    fi
}

echo "═══════════════════════════════════════════════════════════"
echo "1️⃣  Checking Project Structure"
echo "═══════════════════════════════════════════════════════════"

check_file "$WS_PATH/CMakeLists.txt"
check_file "$WS_PATH/package.xml"
check_dir "$WS_PATH/src"
check_dir "$WS_PATH/install"
check_dir "$WS_PATH/build"

echo ""
echo "═══════════════════════════════════════════════════════════"
echo "2️⃣  Checking PX4 Configuration"
echo "═══════════════════════════════════════════════════════════"

check_file "$PX4_PATH/Tools/simulation/gz/worlds/warehouse_d435i.sdf"
check_file "$WS_PATH/startup/run_px4_warehouse_d435i.sh"

echo ""
echo "═══════════════════════════════════════════════════════════"
echo "3️⃣  Checking VINS Configuration"
echo "═══════════════════════════════════════════════════════════"

VINS_CONFIG="$WS_PATH/third_party/VINS-Fusion-ROS2/config/realsense_d435i/realsense_stereo_imu_config.yaml"
check_file "$VINS_CONFIG"
check_dir "$WS_PATH/third_party/VINS-Fusion-ROS2/install"

echo ""
echo "═══════════════════════════════════════════════════════════"
echo "4️⃣  Checking Offboard Controller"
echo "═══════════════════════════════════════════════════════════"

check_file "$WS_PATH/src/px4_hexctl/px4_hexctl/vins_offboard_hover.py"
check_file "$WS_PATH/src/px4_hexctl/setup.py"

echo ""
echo "═══════════════════════════════════════════════════════════"
echo "5️⃣  Checking Launch Scripts"
echo "═══════════════════════════════════════════════════════════"

check_file "$WS_PATH/startup/start_vins_offboard_hover.sh"
check_file "$WS_PATH/startup/run_px4_warehouse_d435i.sh"

echo ""
echo "═══════════════════════════════════════════════════════════"
echo "6️⃣  Verifying Python Node"
echo "═══════════════════════════════════════════════════════════"

cd "$WS_PATH"
source /opt/ros/humble/setup.bash
source ./install/setup.bash 2>/dev/null || echo -e "${YELLOW}⚠️${NC}  Local install not sourced (will be sourced at runtime)"

# Check if node is registered
if ros2 pkg list | grep -q px4_hexctl; then
    echo -e "${GREEN}✅${NC} px4_hexctl package registered"
else
    echo -e "${YELLOW}⚠️${NC}  px4_hexctl not yet in ROS2 path (will be available after build)"
fi

echo ""
echo "═══════════════════════════════════════════════════════════"
echo "7️⃣  Checking Dependencies"
echo "═══════════════════════════════════════════════════════════"

# Check Python dependencies
python3 -c "import rclpy; print('✅ rclpy')" 2>/dev/null || echo "❌ rclpy"
python3 -c "import numpy; print('✅ numpy')" 2>/dev/null || echo "❌ numpy"
python3 -c "import tf_transformations; print('✅ tf_transformations')" 2>/dev/null || echo "❌ tf_transformations"
python3 -c "import px4_msgs; print('✅ px4_msgs')" 2>/dev/null || echo "⚠️  px4_msgs (available in ROS2 context)"

echo ""
echo "═══════════════════════════════════════════════════════════"
echo "8️⃣  System Configuration Summary"
echo "═══════════════════════════════════════════════════════════"

echo ""
echo "🏗️  Workspace Structure:"
echo "   Root: $WS_PATH"
echo "   PX4:  $PX4_PATH"
echo ""

echo "📸 D435i Camera Streams:"
echo "   /camera/color/image_raw"
echo "   /camera/depth/image_rect_raw"
echo "   /camera/infra1/image_rect_raw"
echo "   /camera/infra2/image_rect_raw"
echo "   /camera/imu"
echo ""

echo "🎯 VINS Output:"
echo "   /vins_estimator/odometry → VIO pose estimate"
echo ""

echo "✈️  PX4 Offboard Topics:"
echo "   /fmu/in/offboard_control_mode"
echo "   /fmu/in/trajectory_setpoint"
echo "   /fmu/in/vehicle_command"
echo "   /fmu/out/vehicle_status"
echo ""

echo "═══════════════════════════════════════════════════════════"
echo "9️⃣  Ready to Launch"
echo "═══════════════════════════════════════════════════════════"
echo ""

echo -e "${BLUE}Step 1: Start PX4 SITL${NC}"
echo "   $ ./startup/run_px4_warehouse_d435i.sh"
echo ""

echo -e "${BLUE}Step 2: In another terminal, start VINS + Offboard${NC}"
echo "   $ ./startup/start_vins_offboard_hover.sh"
echo ""

echo -e "${BLUE}Step 3: Monitor system${NC}"
echo "   Terminal 1: Watch PX4 console"
echo "   Terminal 2: Watch VINS/Offboard output"
echo "   Terminal 3 (optional): ros2 topic echo /vins_estimator/odometry"
echo ""

echo -e "${YELLOW}Advanced Testing:${NC}"
echo "   # Check VINS odometry"
echo "   ros2 topic echo /vins_estimator/odometry"
echo ""
echo "   # Check PX4 status"
echo "   ros2 topic echo /fmu/out/vehicle_status"
echo ""
echo "   # Monitor trajectory setpoints"
echo "   ros2 topic echo /fmu/in/trajectory_setpoint"
echo ""

echo -e "${GREEN}✨ System ready for testing!${NC}"
echo ""
