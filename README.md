
# � PX4 ROS2 Vehicle Offboard Control

一键部署系统 | One-Click Deployment System

---

## 📚 部署文档导航 / Documentation Navigation

欢迎使用本项目的部署系统！请根据您的需要选择相应的文档。  
Welcome to our deployment system! Please choose the documentation that matches your needs.

### 🌟 快速导航 / Quick Links

| 您的需求 / Your Need | 推荐文档 / Recommended Doc | 说明 / Description |
|---|---|---|
| **新手用户** / New Users | [📖 快速开始指南](docs/01-QuickStart.md) | 3步快速上手 / 3-step quick start |
| **日常使用** / Daily Usage | [⚡ 快速参考](docs/02-QuickReference.md) | 常用命令速查表 / Command cheat sheet |
| **完整学习** / Deep Learning | [📘 完整部署指南](docs/03-DeploymentGuide.md) | 详细功能说明 / Detailed guide |
| **遇到问题** / Troubleshooting | [🆘 常见问题](docs/04-FAQ.md) | Q&A 和解决方案 / Q&A with solutions |
| **查看示例** / See Examples | [📝 使用示例](docs/05-Examples.md) | 13个实际场景 / 13 real scenarios |
| **了解文件结构** / Understand Files | [📁 文件结构](docs/06-FileStructure.md) | 项目组织说明 / Project organization |
| **功能总结** / Feature Summary | [📊 部署总结](docs/07-DeploymentSummary.md) | 功能清单和统计 / Feature checklist |

### 🎯 常见入口 / Common Entry Points

**👤 如果您是新手 / If you're new:**
```bash
# 1. 阅读快速开始指南
cat docs/01-QuickStart.md

# 2. 运行快速菜单
./deploy/quick_deploy.sh

# 3. 查看实际示例
cat docs/05-Examples.md
```

**🔧 如果您是高级用户 / If you're advanced:**
```bash
# 1. 查看完整部署指南
cat docs/03-DeploymentGuide.md

# 2. 编辑配置文件
vim deploy/deploy_config.sh

# 3. 运行主脚本
./deploy/deploy_offboard.sh -h
```

**🪟 如果您使用Windows / If you're on Windows:**
```cmd
# 运行启动器
deploy\deploy_offboard.bat
```

### 📂 项目结构 / Project Structure

```
├── 📁 deploy/                    ⭐ 部署脚本目录 / Deployment scripts
│   ├── deploy_offboard.sh        主脚本 / Main script
│   ├── quick_deploy.sh           快速菜单 / Quick menu
│   ├── deploy_offboard.bat       Windows启动器 / Windows launcher
│   └── deploy_config.sh          配置文件 / Configuration
│
├── 📁 docs/                      📚 文档目录 / Documentation
│   ├── README.md                 📖 文档导航 / Documentation index
│   ├── 01-QuickStart.md          快速开始 / Quick start
│   ├── 02-QuickReference.md      快速参考 / Quick reference
│   ├── 03-DeploymentGuide.md     完整指南 / Complete guide
│   ├── 04-FAQ.md                 常见问题 / FAQ
│   ├── 05-Examples.md            使用示例 / Usage examples
│   ├── 06-FileStructure.md       文件结构 / File structure
│   └── 07-DeploymentSummary.md   功能总结 / Feature summary
│
└── 📁 src/                       💻 源代码 / Source code
    ├── px4_hexctl/     C++ 实现
    ├── py_script/                Python 脚本
    └── startup/                  启动脚本
```

### 🚀 快速开始 / Get Started Now

```bash
# 方式 1: 快速菜单（推荐新手）/ Method 1: Quick menu (recommended for beginners)
./deploy/quick_deploy.sh

# 方式 2: 完整脚本控制 / Method 2: Full script control
./deploy/deploy_offboard.sh

# 方式 3: 自定义部署 / Method 3: Custom deployment
./deploy/deploy_offboard.sh -m "Your custom message"

# 获取帮助 / Get help
./deploy/deploy_offboard.sh -h
```

---

## 🛠️ offboard_control_lib 使用说明（详细版）  
## 🛠️ Detailed Usage Guide for offboard_control_lib  

---

## 📑 目录 / Table of Contents  

- [1. 概述 / Overview](#1-概述--overview)
- [2. 安装依赖 / Installation Dependencies](#2-安装依赖--installation-dependencies)
- [3. 核心类：Vehicle / Core Class: Vehicle](#3-核心类vehicle--core-class-vehicle)
- [4. 主要 API 方法（通过 vehicledrone 调用） / Key API Methods (via vehicledrone)](#4-主要-api-方法通过-vehicledrone-调用--key-api-methods-via-vehicledrone)
  - [4.1 arm() → bool](#41-arm--bool)
  - [4.2 takeoff(target_altitude: float) → bool](#42-takeofftarget_altitude-float--bool)
  - [4.3 fly_to_trajectory_setpoint(x, y, z, yaw) → None](#43-fly_to_trajectory_setpointx-y-z-yaw--none)
  - [4.4 disarm() → bool](#44-disarm--bool)
  - [4.5 close()](#45-close)
- [5. 使用示例（完整 main 文件） / Usage Example (Complete main File)](#5-使用示例完整-main-文件--usage-example-complete-main-file)
- [6. 注意事项 / Important Notes](#6-注意事项--important-notes)
  - [6.1 坐标系 / Coordinate System](#61-坐标系--coordinate-system)
  - [6.2 Offboard 模式要求 / Offboard Mode Requirements](#62-offboard-模式要求--offboard-mode-requirements)
  - [提示 / Tip](#提示--tip)

---

## 1. 概述 / Overview  

**offboard_control_lib** 是一个基于 **ROS 2（rclpy）** 开发的无人机离线控制库，封装了 PX4 飞控系统与 ROS 2 接口之间的通信逻辑。  
该库提供简洁、高层的 API，用于实现无人机的起飞、飞行控制、悬停、降落及安全退出等操作。  

**offboard_control_lib** is a **ROS 2 (rclpy)**-based library for unmanned aerial vehicle (UAV) offboard control.  
It encapsulates the communication logic between the PX4 flight stack and ROS 2 interfaces, providing a clean, high-level API for essential operations such as arming, takeoff, trajectory control, hovering, landing, and safe shutdown.

---

## 2. 安装依赖 / Installation Dependencies  

确保已安装以下组件：  
Ensure the following components are installed:

- **ROS 2**（推荐 *Humble* 或 *Iron*）  
  **ROS 2** (*Humble* or *Iron* recommended)  
- **PX4 SITL**（软件在环仿真）或真实飞控硬件  
  **PX4 SITL** (*Software-in-the-Loop simulation*) or real flight controller hardware  
- **px4_msgs**（需与 PX4 版本匹配）  
  **px4_msgs** (must match your PX4 version)  
- **rclpy**  
- **offboard_control_lib**（需正确放置于 Python 路径中或通过 `setup.py` 安装）  
  **offboard_control_lib** (must be in your Python path or installed via `setup.py`)  

---

## 3. 核心类：Vehicle / Core Class: `Vehicle`  

`Vehicle` 类继承自 `rclpy.node.Node`，自动初始化 ROS 2 节点，并内部管理以下子模块：  

The `Vehicle` class inherits from `rclpy.node.Node`, automatically initializing a ROS 2 node and managing the following internal components:

- **drone**：实际控制接口对象，提供飞行指令方法  
  **drone:** The actual control interface object that provides flight command methods  
- **订阅器**：接收飞控状态（如位置、姿态、是否就绪等）  
  **Subscribers:** Receive flight controller status (e.g., position, attitude, readiness)  
- **发布器**：发送轨迹设定点（TrajectorySetpoint）、飞控模式命令（OffboardControlMode）等  
  **Publishers:** Send trajectory setpoints (`TrajectorySetpoint`) and mode commands (`OffboardControlMode`)  
- **定时器**：周期性发送控制指令以维持 Offboard 模式  
  **Timers:** Periodically publish control commands to maintain Offboard mode  

### 初始化 / Initialization  

```python
vehicle = Vehicle()
````

此调用会：
This call will:

* 创建名为 `offboard_control_node` 的 ROS 2 节点
* 初始化所有必要的发布者/订阅者
* 启动后台定时器（通常为 20 Hz）以维持 Offboard 控制链路

> ⚠️ **注意 / Note**：`Vehicle` 实例必须在 `rclpy.init()` 之后创建（通常由 `main()` 函数隐式处理）。

---

## 4. 主要 API 方法（通过 `vehicle.drone` 调用） / Key API Methods (via `vehicle.drone`)

### 4.1 `arm() → bool`

**功能 / Function:** 解锁电机（Arming）
**返回值 / Returns:** 成功返回 `True`，否则 `False`
**前提条件 / Prerequisites:** 飞控处于就绪状态（如 GPS 锁定、IMU 校准完成等）

---

### 4.2 `takeoff(target_altitude: float) → bool`

**功能 / Function:** 执行自动起飞至指定高度（单位：米）
**参数 / Parameters:**

* `target_altitude`: 目标高度（相对于起飞点），建议 ≥ 1.5 米
  **行为 / Behavior:** 自动切换至 Offboard 模式，垂直上升至目标高度并悬停
  **返回值 / Returns:** 成功到达目标高度并稳定后返回 `True`

---

### 4.3 `fly_to_trajectory_setpoint(x: float, y: float, z: float, yaw: float) → None`

**功能 / Function:** 飞往指定的 NED 坐标系下的位置和偏航角
**参数 / Parameters:**

* `x`: 北向位置（m）
* `y`: 东向位置（m）
* `z`: 高度（m）— 输入正值表示高度（例如 `2.0` 表示 2 米）
* `yaw`: 偏航角（弧度），0 表示机头朝北

**说明 / Notes:**

* 非阻塞调用，仅发送一次设定点
* 若需持续控制，应在循环中定期调用

---

### 4.4 `disarm() → bool`

**功能 / Function:** 锁定电机（Disarming）
**返回值 / Returns:** 成功返回 `True`

---

### 4.5 `close()`

**功能 / Function:** 释放资源，停止定时器，关闭节点
**使用场景 / Usage:** 程序退出前必须调用（建议放在 `finally` 块中）

---

## 5. 使用示例（完整 main 文件） / Usage Example (Complete main File)

```python
#!/usr/bin/env python3

import rclpy
from offboard_control_lib import Vehicle

def main():
  

    # 创建 Vehicle 实例
    vehicle = Vehicle()

    try:
        # 解锁无人机
        if not vehicle.drone.arm():
            print("Arming failed!")
            return

        # 起飞至 2.0 米高度
        if vehicle.drone.takeoff(2.0):
            print("Takeoff successful. Flying to target position...")
            vehicle.drone.fly_to_trajectory_setpoint(5.0, 0.0, 2.0, 0.0)
        else:
            print("Takeoff failed!")

        # 安全降落并锁定电机
        vehicle.drone.disarm()

    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received. Disarming...")
        vehicle.drone.disarm()

    finally:
        vehicle.close()
        

if __name__ == '__main__':
    main()
```

> 🔔 **注意 / Note:**
> 原始代码中缺少 `rclpy.init()` 和 `rclpy.shutdown()`，已在示例中补充。

---

## 6. 注意事项 / Important Notes

### 6.1 坐标系 / Coordinate System

本库对用户隐藏了 PX4 的 **NED（北-东-下）** 坐标系细节。
用户传入的 `z` 为正数表示高度（如 `2.0` 表示 2 米高），内部自动转换为 `-2.0` 发送给 PX4。

This library abstracts PX4’s **NED (North-East-Down)** coordinate system.
Users provide positive `z` values (e.g., `2.0 = 2 meters high`); internally converted to `-2.0`.

---

### 6.2 Offboard 模式要求 / Offboard Mode Requirements

* 必须以 ≥ 2 Hz 的频率发送控制指令，否则 PX4 会自动退出 Offboard 模式
* 本库通过内部定时器自动维持此频率



