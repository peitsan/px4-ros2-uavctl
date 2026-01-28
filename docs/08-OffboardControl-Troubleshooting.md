# PX4 Offboard Control "no offboard signal" 诊断和解决方案

## 症状
1. **Manual 上锁状态**: 运行程序后,QGC 显示切换到 Offboard 模式,但无法解锁,报错 "no offboard signal"
2. **Manual 飞行状态**: 运行程序后,QGC 一直保持 Manual 模式,无法切换到 Offboard

## 根本原因

从诊断脚本结果可以看出:

```
❌ 没有接收到位置数据 (/fmu/out/vehicle_local_position_v1)
❌ 没有接收到飞控状态数据 (/fmu/out/vehicle_status)
```

**虽然 MicroXRCEAgent 在运行,但 PX4 飞控没有向它发送位置和状态数据。**

这通常意味着:
- **飞控未能识别 MicroXRCEAgent 的连接**
- **通信参数不匹配（波特率等）**
- **飞控固件问题**

## 排查步骤

### 1️⃣ 验证飞控物理连接

```bash
# 在香橙派上检查串口设备
ssh orangepi@192.168.3.17 "ls -la /dev/ttyUSB* /dev/ttyACM*"

# 应该看到至少一个设备，例如:
# crw-rw---- 1 root dialout 188, 0  /dev/ttyUSB0
```

### 2️⃣ 检查飞控固件

在 QGC 中:
1. 打开菜单 > Vehicle Setup > Summary
2. 确认固件版本（应该是 PX4 4.x 以上）
3. 检查是否有任何警告或错误

### 3️⃣ 验证 MicroXRCEAgent 配置

当前启动的 MicroXRCEAgent:
```bash
MicroXRCEAgent serial -D /dev/ttyUSB0 -b 115200
```

**波特率可能需要调整。** 常见的 PX4 波特率:
- `115200` - 标准配置
- `921600` - 某些飞控使用
- `57600` - 旧版本

### 4️⃣ 检查 PX4 日志

在飞控连接到 QGC 时:
1. 点击 Toolbox > Analyze
2. 查看 Log Files
3. 查找与 MicroXRCE 或 XRCE 相关的错误信息

### 5️⃣ 测试基本通信

```bash
# 在香橙派上启动 MicroXRCEAgent，查看连接信息
ssh orangepi@192.168.3.17 
MicroXRCEAgent serial -D /dev/ttyUSB0 -b 115200 -v5
```

输出应该包含连接成功的日志。

## 常见问题和解决方案

### 问题 A: 串口权限不足

```bash
# 错误：Permission denied: /dev/ttyUSB0
# 解决：
ssh orangepi@192.168.3.17 "sudo usermod -a -G dialout orangepi"
# 然后重新登录 SSH
```

### 问题 B: MicroXRCEAgent 连接失败

症状: MicroXRCEAgent 运行但没有输出,或频繁断开

解决方案:
```bash
# 尝试不同的波特率
MicroXRCEAgent serial -D /dev/ttyUSB0 -b 921600

# 或者检查串口是否正确
ls -la /dev/ttyUSB*

# 如果有多个串口,尝试另一个
# /dev/ttyUSB0 vs /dev/ttyUSB1 vs /dev/ttyACM0
```

### 问题 C: 话题有发布者但没有数据

症状: 
- Offboard Control Mode 话题有发布者
- 但位置话题没有数据

原因: **飞控没有启动或 MicroXRCEAgent 连接失败**

解决:
1. 确保飞控已通电
2. 确保飞控通过 USB 连接到香橙派
3. 检查 QGC 中飞控的状态

### 问题 D: 无法切换 Offboard 模式

症状: QGC 中无法选择 Offboard 模式,或选后报 "no offboard signal"

原因:
1. 位置数据丢失（最常见）
2. Offboard 信号断开
3. 飞控固件不支持

解决:
```bash
# 验证位置数据
ssh orangepi@192.168.3.17 'source ~/uav_ws/install/setup.bash && ros2 topic echo /fmu/out/vehicle_local_position_v1' &

# 在另一个终端,启动 Offboard Control
ssh orangepi@192.168.3.17 'source ~/uav_ws/install/setup.bash && ros2 run offboard_control_cpp offboard_control_main'
```

位置数据应该继续输出。

## 快速诊断命令

```bash
# 一键诊断（在本机运行）
bash ~/Desktop/px4-ros2-uavctl/deploy/diagnose_offboard.sh

# 实时监控位置数据
ssh orangepi@192.168.3.17 'source ~/uav_ws/install/setup.bash && ros2 topic hz /fmu/out/vehicle_local_position_v1'

# 实时监控飞控状态
ssh orangepi@192.168.3.17 'source ~/uav_ws/install/setup.bash && ros2 topic hz /fmu/out/vehicle_status'

# 检查 Offboard 信号发送
ssh orangepi@192.168.3.17 'source ~/uav_ws/install/setup.bash && ros2 topic hz /fmu/in/offboard_control_mode'
```

## 最后的检查清单

- [ ] 飞控已通电
- [ ] 飞控通过 USB 连接到香橙派
- [ ] MicroXRCEAgent 正在运行（`ps aux | grep MicroXRCEAgent`）
- [ ] 串口权限正确（`ls -la /dev/ttyUSB*`）
- [ ] 位置话题有数据（`ros2 topic echo /fmu/out/vehicle_local_position_v1`）
- [ ] 飞控状态话题有数据（`ros2 topic echo /fmu/out/vehicle_status`）
- [ ] QGC 显示飞控已连接
- [ ] Offboard Control 节点正常运行

## 如果仍有问题

1. 收集日志:
   ```bash
   # PX4 日志（在 QGC 中下载）
   # ROS2 日志
   ssh orangepi@192.168.3.17 'cat ~/.ros/log/latest/offboard_control_center/0/stderr.log'
   ```

2. 检查飞控固件版本和配置
3. 确保使用了正确的波特率（通常 115200 或 921600）
4. 考虑升级或重新刷写飞控固件

## 相关文档

- [PX4 Offboard Mode](https://docs.px4.io/main/en/flight_modes/offboard.html)
- [MicroXRCEAgent 文档](https://micro-xrce-dds-docs.docs.eprosima.com/)
- [px4_msgs ROS2 包](https://github.com/PX4/px4_msgs)
