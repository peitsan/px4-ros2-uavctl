# 🚁 PX4-ROS2 无人机 Offboard 控制系统 - 完整配置总结

## 项目概述

这是一个完整的 PX4 无人机 ROS2 Offboard 控制系统,支持:
- ✅ 远程 SSH 免密登录到香橙派机载电脑
- ✅ 自动编译和部署到机载电脑
- ✅ 完整的 Offboard 控制节点(C++)
- ✅ 自动化启动脚本
- ✅ 诊断和故障排除工具

## 系统架构

```
┌─────────────────────────────────────────────────────────┐
│ 本机 (Development PC)                                   │
│                                                         │
│  startup_realrobot.sh ────→ 启动 QGroundControl        │
│  deploy_offboard.sh ──────→ 编译和部署代码             │
│  diagnose_offboard.sh ────→ 诊断系统问题               │
└─────────────────────────────────────────────────────────┘
              ↓ SSH 免密连接 (192.168.3.17)
┌─────────────────────────────────────────────────────────┐
│ 香橙派 (Onboard Computer)                               │
│                                                         │
│  ├─ MicroXRCEAgent ──────→ PX4 飞控通信                 │
│  └─ ROS2 Humble                                         │
│      └─ px4_hexctl                            │
│          ├─ 发送心跳信号 (20Hz)                         │
│          ├─ 接收飞控反馈                                │
│          └─ 执行飞行命令                                │
└─────────────────────────────────────────────────────────┘
              ↓ USB/UART 串口 (115200 bps)
┌─────────────────────────────────────────────────────────┐
│ PX4 飞控                                                │
│                                                         │
│  ├─ 接收 Offboard 控制信号                              │
│  ├─ 发送位置/速度/状态数据                              │
│  └─ 执行解锁/起飞/降落等命令                            │
└─────────────────────────────────────────────────────────┘
```

## 快速开始

### 前置条件

1. **本机环境**:
   - Linux (Ubuntu 22.04 推荐)
   - SSH 客户端
   - gnome-terminal
   - git

2. **香橙派环境** (远端):
   - ROS2 Humble
   - px4_msgs
   - px4_ros_com
   - MicroXRCEAgent
   - px4_hexctl 包

3. **硬件连接**:
   - PX4 飞控 → USB 连接到香橙派
   - 香橙派 ↔ 网络连接到本机

### 1️⃣ 配置 SSH 免密登录

```bash
# 生成 SSH 密钥(如未生成)
ssh-keygen -t ed25519 -f ~/.ssh/org_ed26619

# 添加公钥到远端
ssh-copy-id -i ~/.ssh/org_ed26619.pub orangepi@192.168.3.17

# 验证连接
ssh orangepi@192.168.3.17 "echo 'SSH OK'"
```

### 2️⃣ 部署代码到香橙派

```bash
cd ~/Desktop/px4-ros2-uavctl

# 快速部署(推送 + 编译)
bash deploy/quick_deploy.sh

# 或详细部署
bash deploy/deploy_offboard.sh
```

### 3️⃣ 启动无人机控制系统

```bash
# 一键启动(自动启动 MicroXRCEAgent + Offboard Control + QGC)
./startup/startup_realrobot.sh

# 或分步启动
./startup/start_offboard_remote.sh
```

## 核心文件说明

### 部署脚本

| 脚本 | 功能 | 用途 |
|------|------|------|
| `deploy/deploy_offboard.sh` | 完整部署流程 | git 提交 + 推送 + 编译 |
| `deploy/quick_deploy.sh` | 快速部署菜单 | 场景化快速选择 |
| `deploy/diagnose_offboard.sh` | 系统诊断 | 检查连接和话题状态 |
| `deploy/clean_remote_build.sh` | 清理编译缓存 | 远端清理工作区 |

### 启动脚本

| 脚本 | 功能 | 用途 |
|------|------|------|
| `startup/startup_realrobot.sh` | 主启动脚本 | 启动 Agent/Offboard/QGC |
| `startup/start_offboard_remote.sh` | 直接启动节点 | 远端快速启动 Offboard |
| `startup/startup.sh` | 仿真启动脚本 | Gazebo 仿真环境 |

### 源代码

| 文件 | 功能 |
|------|------|
| `src/px4_hexctl/src/main.cpp` | 主程序入口 |
| `src/px4_hexctl/src/offboard_control_lib/offboard_control.cpp` | 控制核心库 |
| `src/px4_hexctl/src/offboard_control_lib/vehicle.cpp` | 无人机对象 |

### 文档

| 文档 | 内容 |
|------|------|
| `docs/01-QuickStart.md` | 项目快速开始 |
| `docs/02-QuickReference.md` | 快速参考 |
| `docs/03-DeploymentGuide.md` | 部署指南 |
| `docs/08-OffboardControl-Troubleshooting.md` | 故障排除 |
| `docs/09-QuickStart-Offboard.md` | Offboard 启动指南 |

## 常见问题

### Q: 无法连接到香橙派

```bash
# 检查网络连接
ping 192.168.3.17

# 检查 SSH 密钥
ssh -v orangepi@192.168.3.17 "echo 'test'"

# 生成密钥并配置
ssh-keygen -t ed25519 -f ~/.ssh/org_ed26619
ssh-copy-id -i ~/.ssh/org_ed26619.pub orangepi@192.168.3.17
```

### Q: 编译失败

```bash
# 检查依赖
ssh orangepi@192.168.3.17 "cd ~/uav_ws && rosdep install --from-paths src --ignore-src -r -y"

# 清理后重新编译
bash deploy/clean_remote_build.sh
bash deploy/deploy_offboard.sh -n
```

### Q: Offboard 节点启动失败

```bash
# 诊断系统
bash deploy/diagnose_offboard.sh

# 检查 MicroXRCEAgent
ssh orangepi@192.168.3.17 "ps aux | grep MicroXRCEAgent"

# 查看 ROS2 话题
ssh orangepi@192.168.3.17 "source ~/uav_ws/install/setup.bash && ros2 topic list"
```

### Q: 飞控报 "no offboard signal"

确保:
1. ✅ MicroXRCEAgent 运行中
2. ✅ 位置话题有数据
3. ✅ Offboard Control 节点运行中
4. ✅ 心跳信号正常发送 (20Hz)

```bash
# 检查位置数据
ssh orangepi@192.168.3.17 "
    source ~/uav_ws/install/setup.bash
    ros2 topic echo /fmu/out/vehicle_local_position_v1 --once
"
```

## 工作流总结

### 日常开发流程

```bash
# 1. 编辑本地代码
vim src/px4_hexctl/src/main.cpp

# 2. 部署到远端
bash deploy/quick_deploy.sh
# 选择 "build" 场景

# 3. 启动系统
./startup/startup_realrobot.sh

# 4. 监控和调试
ssh orangepi@192.168.3.17 "
    tail -f ~/.ros/log/latest/offboard_control_center/0/stdout
"
```

### 完整飞行流程

```bash
# 1. 启动系统
./startup/startup_realrobot.sh

# 2. 确认 MicroXRCEAgent 连接
# 3. QGC 中检查飞控状态
# 4. 观察 Offboard Control 日志
# 5. 无人机自动解锁和起飞
# 6. 执行预定飞行任务
# 7. 自动降落和上锁
# 8. Ctrl+C 停止所有进程
```

## 关键参数配置

### MicroXRCEAgent

```bash
# 串口参数
-D /dev/ttyUSB0    # 串口设备
-b 115200          # 波特率
```

### Offboard Control

```cpp
// 心跳信号频率
heartbeat_hz_ = 20;  // 20 Hz

// 预热消息数
engage_offboard_mode(10, 2.0);  // 10条消息或2秒超时

// 起飞高度
takeoff_command_global(1.5, ...);  // 1.5 米
```

## 系统检查清单

启动前确保:

- [ ] SSH 免密登录已配置
- [ ] 网络连接正常 (`ping 192.168.3.17`)
- [ ] 飞控已通电并通过 USB 连接
- [ ] 香橙派已启动
- [ ] 代码已部署到远端 (`bash deploy/quick_deploy.sh`)
- [ ] MicroXRCEAgent 能启动 (检查串口权限)
- [ ] ROS2 话题正常 (`ros2 topic list`)
- [ ] QGC 识别飞控

## 技术支持

### 查看日志

```bash
# 本机日志
tail -f ~/.ros/log/latest/*/stdout

# 远端日志
ssh orangepi@192.168.3.17 "tail -f ~/.ros/log/latest/*/stdout"

# PX4 日志 (在 QGC 中下载)
```

### 运行诊断

```bash
# 完整诊断
bash ./deploy/diagnose_offboard.sh

# 检查特定话题
ros2 topic echo /fmu/out/vehicle_local_position_v1
ros2 topic echo /fmu/out/vehicle_status
ros2 topic hz /fmu/in/offboard_control_mode
```

### 获取帮助

参考以下文档:
- `docs/04-FAQ.md` - 常见问题
- `docs/08-OffboardControl-Troubleshooting.md` - 故障排除
- `docs/09-QuickStart-Offboard.md` - 快速启动

---

**最后更新**: 2026-01-28  
**版本**: 1.0  
**维护者**: px4-ros2-uavctl 团队
