#!/bin/bash
# ================================================================
# PX4 Offboard Control 诊断脚本
# 用于排查 "no offboard signal" 等问题
# ================================================================

set -e

REMOTE_HOST="orangepi@192.168.3.17"
REMOTE_WORKSPACE="/home/orangepi/uav_ws"

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}================================================================${NC}"
echo -e "${BLUE}PX4 Offboard Control 诊断工具${NC}"
echo -e "${BLUE}================================================================${NC}"

# 检查 SSH 连接
echo -e "\n${YELLOW}1. 检查 SSH 连接...${NC}"
if ssh -o ConnectTimeout=5 -o BatchMode=yes "${REMOTE_HOST}" "echo 'SSH连接正常'" > /dev/null 2>&1; then
    echo -e "${GREEN}✅ SSH连接正常${NC}"
else
    echo -e "${RED}❌ SSH连接失败${NC}"
    exit 1
fi

# 检查 MicroXRCEAgent
echo -e "\n${YELLOW}2. 检查 MicroXRCEAgent 状态...${NC}"
AGENT_STATUS=$(ssh "${REMOTE_HOST}" "ps aux | grep MicroXRCEAgent | grep -v grep | wc -l")
if [ "$AGENT_STATUS" -gt 0 ]; then
    echo -e "${GREEN}✅ MicroXRCEAgent 正在运行${NC}"
    ssh "${REMOTE_HOST}" "ps aux | grep MicroXRCEAgent | grep -v grep"
else
    echo -e "${RED}❌ MicroXRCEAgent 未运行${NC}"
    echo -e "${YELLOW}   建议：启动 MicroXRCEAgent（通过 startup_realrobot.sh 或手动启动）${NC}"
fi

# 检查串口连接
echo -e "\n${YELLOW}3. 检查串口设备...${NC}"
SERIAL_COUNT=$(ssh "${REMOTE_HOST}" "ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null | wc -l")
if [ "$SERIAL_COUNT" -gt 0 ]; then
    echo -e "${GREEN}✅ 检测到串口设备${NC}"
    ssh "${REMOTE_HOST}" "ls -la /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || true"
else
    echo -e "${RED}❌ 未检测到串口设备${NC}"
    echo -e "${YELLOW}   建议：检查飞控和香橙派的连接${NC}"
fi

# 检查 ROS2 话题
echo -e "\n${YELLOW}4. 检查 ROS2 话题...${NC}"
echo -e "${BLUE}   发布话题（飞控→ROS2）:${NC}"
ssh "${REMOTE_HOST}" "export ROS_DOMAIN_ID=0 && source ~/uav_ws/install/setup.bash && ros2 topic list 2>/dev/null | grep 'fmu/out' | head -10" || echo "   无输出"

echo -e "\n${BLUE}   订阅话题（ROS2→飞控）:${NC}"
ssh "${REMOTE_HOST}" "export ROS_DOMAIN_ID=0 && source ~/uav_ws/install/setup.bash && ros2 topic list 2>/dev/null | grep 'fmu/in' | head -10" || echo "   无输出"

# 检查位置话题数据
echo -e "\n${YELLOW}5. 检查位置话题是否有数据...${NC}"
echo -e "${BLUE}   尝试读取 /fmu/out/vehicle_local_position...${NC}"
POSITION_DATA=$(ssh "${REMOTE_HOST}" "export ROS_DOMAIN_ID=0 && source ~/uav_ws/install/setup.bash && timeout 2 ros2 topic echo --once /fmu/out/vehicle_local_position 2>&1" || true)

if [ -z "$POSITION_DATA" ] || echo "$POSITION_DATA" | grep -q "No messages"; then
    echo -e "${BLUE}   尝试读取 /fmu/out/vehicle_local_position_v1...${NC}"
    POSITION_DATA=$(ssh "${REMOTE_HOST}" "source ~/uav_ws/install/setup.bash && timeout 2 ros2 topic echo --once /fmu/out/vehicle_local_position_v1 2>&1" || true)
fi

if [ -z "$POSITION_DATA" ] || echo "$POSITION_DATA" | grep -q "No messages"; then
    echo -e "${RED}❌ 没有接收到任何位置数据 (Local Position)${NC}"
    echo -e "${YELLOW}   💡 这通常意味着：${NC}"
    echo -e "   1. 无人机在室内，没有 GPS 且没有定位设备 (VIO/Flow)"
    echo -e "   2. EKF2 尚未初始化完成"
    echo -e "   3. ${RED}警告：没有位置数据，无法使用 'position' 定点控制模式！${NC}"
    echo -e "   4. 建议尝试使用 'attitude' 模式进行测试。"
else
    echo -e "${GREEN}✅ 接收到位置数据${NC}"
    echo "$POSITION_DATA" | head -n 15
fi

# 检查 Offboard Control Mode 话题
echo -e "\n${YELLOW}6. 检查 Offboard Control 话题...${NC}"
echo -e "${BLUE}   尝试读取 /fmu/in/offboard_control_mode（3秒超时）${NC}"
OFFBOARD_DATA=$(ssh "${REMOTE_HOST}" "export ROS_DOMAIN_ID=0 && source ~/uav_ws/install/setup.bash && timeout 3 ros2 topic info /fmu/in/offboard_control_mode 2>&1" || true)
echo "$OFFBOARD_DATA"

# 检查飞控状态
echo -e "\n${YELLOW}7. 检查飞控状态...${NC}"
echo -e "${BLUE}   尝试读取 /fmu/out/vehicle_status（3秒超时）${NC}"
STATUS_DATA=$(ssh "${REMOTE_HOST}" "export ROS_DOMAIN_ID=0 && source ~/uav_ws/install/setup.bash && timeout 3 ros2 topic echo --once /fmu/out/vehicle_status 2>&1" || true)
if [ -z "$STATUS_DATA" ] || echo "$STATUS_DATA" | grep -q "No messages"; then
    echo -e "${RED}❌ 没有接收到飞控状态${NC}"
else
    echo -e "${GREEN}✅ 接收到飞控状态${NC}"
    echo "$STATUS_DATA" | head -10
fi

# 诊断总结
echo -e "\n${YELLOW}8. 诊断总结${NC}"
echo -e "${BLUE}================================================================${NC}"

if [ "$AGENT_STATUS" -eq 0 ]; then
    echo -e "${RED}❌ 主要问题：MicroXRCEAgent 未运行${NC}"
    echo -e "${YELLOW}   解决步骤：${NC}"
    echo -e "   1. 启动本机的 startup_realrobot.sh（会远程启动 MicroXRCEAgent）"
    echo -e "   2. 或手动在香橙派上启动: ssh orangepi@192.168.3.17 'MicroXRCEAgent serial -D /dev/ttyUSB0 -b 921600'"
    echo -e "   3. 确认飞控已通电并通过串口连接到香橙派"
elif [ "$POSITION_DATA" = "" ]; then
    echo -e "${RED}❌ 主要问题：未收到位置数据${NC}"
    echo -e "${YELLOW}   可能原因和解决方案：${NC}"
    echo -e "   1. MicroXRCEAgent 连接失败"
    echo -e "   2. 飞控固件版本不兼容"
    echo -e "   3. 检查飞控 QGC 日志获取更多信息"
else
    echo -e "${GREEN}✅ 基本状态良好，应该可以进行 Offboard 控制${NC}"
fi

echo -e "\n${BLUE}================================================================${NC}"
echo -e "${YELLOW}更多命令帮助：${NC}"
echo -e "   # 查看实时位置数据"
echo -e "   ssh orangepi@192.168.3.17 'source ~/uav_ws/install/setup.bash && ros2 topic echo /fmu/out/vehicle_local_position_v1'"
echo -e "\n   # 查看飞控状态"
echo -e "   ssh orangepi@192.168.3.17 'source ~/uav_ws/install/setup.bash && ros2 topic echo /fmu/out/vehicle_status'"
echo -e "\n   # 手动启动 Offboard Control 节点"
echo -e "   ssh orangepi@192.168.3.17 'source ~/uav_ws/install/setup.bash && ros2 run px4_hexctl offboard_control_main'"
echo -e "\n${BLUE}================================================================${NC}"
