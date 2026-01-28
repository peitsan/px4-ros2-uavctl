# Offboard Control 启动指南

## 快速启动方式

### 方式 1: 使用启动脚本（推荐）

```bash
# 从本机运行此脚本,会自动启动:
# 1. 远端 MicroXRCEAgent (飞控通信)
# 2. 远端 Offboard Control 节点
# 3. 本机 QGroundControl (可选)

./startup/startup_realrobot.sh
```

然后在提示时:
- Offboard Control: 自动启动（无需确认）
- QGroundControl: 输入 `y` 启动地面站（可选）

### 方式 2: 手动启动各个组件

#### 第 1 步: 启动 MicroXRCEAgent

```bash
# 在本机运行此脚本启动远端 MicroXRCEAgent
gnome-terminal --tab --title="MicroXRCEAgent" -- bash -c "
    ssh -t orangepi@192.168.3.17 \
    'source /opt/ros/humble/setup.bash && MicroXRCEAgent serial -D /dev/ttyUSB0 -b 115200'
"
```

或直接在远端香橙派上:
```bash
source /opt/ros/humble/setup.bash
MicroXRCEAgent serial -D /dev/ttyUSB0 -b 115200
```

#### 第 2 步: 启动 Offboard Control

```bash
# 方式 A: 使用便捷脚本
./startup/start_offboard_remote.sh

# 方式 B: 手动启动
ssh orangepi@192.168.3.17 "
    source /opt/ros/humble/setup.bash
    source ~/uav_ws/install/setup.bash
    ros2 run offboard_control_cpp offboard_control_main
"
```

#### 第 3 步: 启动 QGroundControl

```bash
# 本机运行
~/Desktop/QGroundControl.sh
```

或在启动脚本中选择 `y`

## 验证系统状态

### 检查 MicroXRCEAgent 连接

```bash
ssh orangepi@192.168.3.17 "ps aux | grep MicroXRCEAgent | grep -v grep"
```

应该看到类似的输出:
```
orangepi 3574 MicroXRCEAgent serial -D /dev/ttyUSB0 -b 115200
```

### 检查 ROS2 话题

```bash
# 检查位置数据
ssh orangepi@192.168.3.17 "
    source ~/uav_ws/install/setup.bash
    ros2 topic echo /fmu/out/vehicle_local_position_v1 --once
"

# 检查飞控状态
ssh orangepi@192.168.3.17 "
    source ~/uav_ws/install/setup.bash
    ros2 topic echo /fmu/out/vehicle_status --once
"

# 检查 Offboard 信号
ssh orangepi@192.168.3.17 "
    source ~/uav_ws/install/setup.bash
    ros2 topic hz /fmu/in/offboard_control_mode
"
```

### 运行完整诊断

```bash
bash ./deploy/diagnose_offboard.sh
```

## 常见问题

### Q: Offboard 节点启动后仍然报 "no offboard signal"

**A:** 这说明飞控没有收到位置数据。检查:
1. MicroXRCEAgent 是否正常运行
2. 飞控是否通过 USB 连接到香橙派
3. 位置话题是否有数据: `ros2 topic echo /fmu/out/vehicle_local_position_v1`

### Q: 无法连接到远端香橙派

**A:** 检查 SSH 配置:
```bash
# 测试 SSH 连接
ssh -v orangepi@192.168.3.17 "echo 'SSH OK'"

# 确保免密登录已配置
# 如未配置,运行:
ssh-copy-id -i ~/.ssh/org_ed26619.pub orangepi@192.168.3.17
```

### Q: Offboard Control 节点启动失败

**A:** 检查包是否编译成功:
```bash
ssh orangepi@192.168.3.17 "
    cd ~/uav_ws
    source install/setup.bash
    ros2 pkg list | grep offboard_control
"
```

如果未列出,需要重新编译:
```bash
cd ~/Desktop/px4-ros2-uavctl
bash deploy/deploy_offboard.sh -n
```

## 启动脚本工作流

```
startup_realrobot.sh
    ├─ 检查依赖和网络连接
    ├─ 启动 MicroXRCEAgent (远端)
    │   └─ 等待 5 秒建立连接
    ├─ 自动启动 Offboard Control (远端)
    │   └─ 初始化 ROS2 节点
    │   └─ 启动心跳线程
    │   └─ 切换到 OFFBOARD 模式
    ├─ 询问是否启动 QGroundControl
    └─ 监控所有进程状态
```

## 通用启动顺序

对于任何启动方式,建议的顺序是:

```
1. MicroXRCEAgent ───→ 建立飞控通信
2. Offboard Control ──→ 建立无人机控制
3. QGroundControl ────→ 监控飞行状态 (可选)
```

**重要**: 必须先启动 MicroXRCEAgent,否则 Offboard Control 无法获取飞控数据。

## 快速命令参考

```bash
# 一键启动
./startup/startup_realrobot.sh

# 直接启动 offboard 节点
./startup/start_offboard_remote.sh

# 诊断系统
bash ./deploy/diagnose_offboard.sh

# 查看实时日志
ssh orangepi@192.168.3.17 "tail -f ~/.ros/log/latest/offboard_control_center/0/stdout"

# 停止远端进程
ssh orangepi@192.168.3.17 "pkill -f 'offboard_control_main|MicroXRCEAgent'"
```
