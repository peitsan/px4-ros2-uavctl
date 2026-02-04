# D435i 相机与 gz-sim7 集成指南

## 概述

本项目已更新以支持在PX4 SITL仿真中使用 Intel RealSense D435i 深度相机。该集成使用 **Gazebo gz-sim7** 替代了旧的 **ign-gazebo** 版本，确保无人机仿真能够正确加载和使用 D435i 相机。

## 更新内容

### 1. 深度相机 XACRO 配置更新
**文件**: `src/realsense_ros_gazebo/xacro/depthcam.xacro`

- ✅ 移除了旧的 `librealsense_gazebo_plugin.so` 插件依赖
- ✅ 转换为 gz-sim7 原生的传感器定义格式
- ✅ 更新了相机传感器配置以支持以下传感器：
  - **彩色相机** (RGB): 640×480 @ 30Hz
  - **深度相机** (RGBD): 1280×720 @ 30Hz
  - **红外摄像头1和2**: 1280×720 @ 30Hz

### 2. 无人机 URDF 配置
**文件**: `urdf/x500_d435i.urdf.xacro`

- ✅ 为 X500 四旋翼无人机创建了新的 URDF 配置
- ✅ 集成了 D435i 相机宏定义
- ✅ 支持 gz-sim7 物理引擎和传感器系统

### 3. 仿真世界文件
**文件**: `world/aruco_d435i_gz_sim7.sdf`

- ✅ 创建了专用的 SDF 世界配置
- ✅ 加载 PX4 原生的 `x500_realsense_d435i` 模型
- ✅ 配置了所有必要的 gz-sim7 系统插件

### 4. 启动脚本
**文件**: `startup/start_qr_click_tracking_d435i.sh` (新增)

- ✅ 完全的 D435i 支持启动脚本
- ✅ 自动配置 gz-sim7 资源路径
- ✅ ROS2-Gazebo 桥接配置用于相机主题
- ✅ 详细的主题和调试信息

## 使用方法

### 前置需求

```bash
# 确保已安装 gz-sim7 和 ROS2 Humble
sudo apt install gz-sim7 ros-humble-gazebo-ros-pkgs ros-humble-ros-gz
```

### 启动 D435i 支持的仿真

```bash
# 方式1: 使用新的 D435i 启动脚本
cd /home/ubuntu/Desktop/px4-ros2-uavctl
bash startup/start_qr_click_tracking_d435i.sh

# 方式2: 使用更新的默认启动脚本
bash startup/start_qr_click_tracking_sitl.sh
```

### 查看可用的相机主题

一旦仿真启动，您可以访问以下ROS2主题：

```bash
# 查看所有可用的相机主题
ros2 topic list | grep camera

# 查看彩色图像
ros2 topic echo /camera/color/image_raw

# 查看深度图像
ros2 topic echo /camera/depth/image_rect_raw

# 查看红外图像
ros2 topic echo /camera/infra1/image_rect_raw
ros2 topic echo /camera/infra2/image_rect_raw

# 查看D435i IMU数据
ros2 topic echo /camera/imu
```

### 使用RQT查看图像

```bash
# 启动 RQT 图像查看器
ros2 run rqt_image_view rqt_image_view

# 在 GUI 中选择相应的主题
# 例如: /camera/color/image_raw
```

## 技术细节

### gz-sim7 与 ign-gazebo 的区别

| 特性 | ign-gazebo | gz-sim7 |
|------|-----------|---------|
| 插件系统 | ignition-gazebo-* | gz-sim-* |
| 传感器定义 | `<gazebo><sensor>` | `<gazebo><sensor>` (改进) |
| ROS2 集成 | ros_ign_* | ros_gz_* |
| 物理引擎 | ODE/Bullet | ODE/Bullet (改进) |
| 模型加载 | model:// | model:// (相同) |

### 相机主题映射

```
D435i 物理相机               →   ROS2 主题
─────────────────────────────────────────────
Color Camera (640×480)       →   /camera/color/image_raw
Depth Camera (1280×720)      →   /camera/depth/image_rect_raw
Left IR (1280×720)           →   /camera/infra1/image_rect_raw
Right IR (1280×720)          →   /camera/infra2/image_rect_raw
D435i IMU (200Hz)            →   /camera/imu
```

### 相机位置

D435i 相机相对于无人机基座的位置：

```
X (前向): +0.12 m
Y (右向): +0.03 m
Z (上向): +0.242 m
```

## 故障排除

### 问题1: 无法找到 gz-sim7 模型

**症状**: `Error: Cannot find model://x500_realsense_d435i`

**解决方案**:
```bash
# 检查 GZ_SIM_RESOURCE_PATH
echo $GZ_SIM_RESOURCE_PATH

# 确保 PX4 模型路径已包含
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:/home/ubuntu/PX4-Autopilot/Tools/simulation/gz/models

# 重新启动仿真
```

### 问题2: 相机主题未发布

**症状**: `ros2 topic list` 中没有相机主题

**解决方案**:
```bash
# 确认 Gazebo 已启动
ps aux | grep gz-sim

# 检查 ros_gz_bridge 是否运行
ros2 node list | grep bridge

# 手动启动桥接
ros2 run ros_gz_bridge parameter_bridge /camera/color/image_raw@sensor_msgs/msg/Image[gz.msgs.Image
```

### 问题3: 仿真性能问题

**症状**: 仿真运行缓慢或不稳定

**解决方案**:
```bash
# 1. 检查 gz-sim7 是否使用正确的渲染引擎
export GZ_RENDERING_ENGINE=ogre2

# 2. 降低相机更新频率（编辑 depthcam.xacro）
<update_rate>10</update_rate>  # 改为 10Hz

# 3. 禁用相机可视化
<visualize>0</visualize>

# 4. 使用高性能模式
gz-sim aruco_d435i_gz_sim7.sdf -v 4
```

## 后续开发

### 与现有QR追踪集成

```bash
# 使用 D435i 深度数据进行距离估计
ros2 launch px4_hexctl qr_click_tracking_sms.launch.py use_depth:=true

# 启用深度基础的目标检测
ros2 run qr_tracker qr_tracker_depth_node
```

### 自定义相机参数

编辑 `src/realsense_ros_gazebo/xacro/depthcam.xacro`:

```xml
<!-- 修改相机内参 -->
<horizontal_fov>1.3962634</horizontal_fov>  <!-- FOV (弧度) -->

<!-- 修改分辨率 -->
<width>1280</width>
<height>720</height>

<!-- 修改更新频率 -->
<update_rate>30</update_rate>
```

## 参考资源

- [PX4 仿真文档](https://docs.px4.io/v1.13/en/simulation/)
- [Gazebo gz-sim 文档](https://gazebosim.org/docs)
- [RealSense D435i 规格](https://www.intel.com/content/www/us/en/architecture-and-technology/realsense-overview.html)
- [ROS2 - Gazebo 集成](https://github.com/gazebosim/ros_gz)

## 许可证

遵循项目原有许可证（Apache 2.0）

## 贡献

如遇到问题或有改进建议，请提交 issue 或 pull request。
