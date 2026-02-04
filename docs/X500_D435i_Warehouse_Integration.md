# X500 + D435i 深度相机集成指南

## 概述

本集成将Intel RealSense D435i深度相机与PX4 X500四旋翼无人机集成到Gazebo Warehouse仓库模拟环境中。

### 主要特性

- ✅ **X500无人机模型**：基于PX4官方四旋翼设计
- ✅ **D435i深度相机**：5路传感器流（RGB、深度、IR左、IR右、IMU）
- ✅ **Warehouse仓库环境**：从depth_d435包复用的仓库场景
- ✅ **ROS2完全集成**：使用ros_gz_bridge进行话题桥接
- ✅ **RViz可视化**：支持机器人模型、坐标系和相机图像显示
- ✅ **GZ-Sim7支持**：使用现代Gazebo gz-sim7引擎

---

## 文件结构

```
/home/ubuntu/Desktop/px4-ros2-uavctl/
├── urdf/
│   ├── x500_d435i_full.xacro          # X500+D435i完整URDF定义
│   └── x500_d435i.urdf.xacro          # 原始X500配置
├── world/
│   ├── warehouse_x500_d435i.sdf       # 仓库环境配置（X500优化）
│   └── aruco_d435i_gz_sim7.sdf        # 原始ArUco标记环境
├── rviz/
│   └── x500_d435i.rviz                # RViz2可视化配置
├── launch/
│   └── x500_d435i_warehouse.launch.py # 完整启动脚本（Python）
├── startup/
│   ├── start_warehouse_x500_d435i.sh  # 快速启动脚本（Bash）
│   ├── test_x500_d435i_integration.sh # 集成验证脚本
│   └── start_px4_sitl_direct.sh       # PX4 SITL启动脚本
└── src/depth_d435/
    ├── urdf/sensors_diffbot.xacro     # 原始D435i传感器配置
    ├── rviz/robot_display.rviz        # 原始RViz配置
    └── worlds/warehouse.sdf           # 原始仓库环境
```

---

## 快速开始

### 方式1：Bash启动脚本（推荐）

```bash
cd /home/ubuntu/Desktop/px4-ros2-uavctl
bash startup/start_warehouse_x500_d435i.sh
```

这将启动：
1. Gazebo仓库环境
2. X500+D435i机器人
3. ROS2话题桥接
4. RViz2可视化

### 方式2：ROS2启动脚本

```bash
cd /home/ubuntu/Desktop/px4-ros2-uavctl
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch px4_hexctl x500_d435i_warehouse.launch.py
```

### 方式3：验证集成

运行验证脚本以检查所有组件是否就位：

```bash
bash /home/ubuntu/Desktop/px4-ros2-uavctl/startup/test_x500_d435i_integration.sh
```

---

## 传感器配置

### RGB相机（彩色）

- **分辨率**: 640 × 480 像素
- **帧率**: 30 Hz
- **话题**: `/camera/color/image_raw`
- **视场角 (FOV)**: 69.4°
- **消息类型**: `sensor_msgs/Image`

### 深度相机

- **分辨率**: 1280 × 720 像素
- **帧率**: 30 Hz
- **话题**: `/camera/depth/image_rect_raw`
- **焦距**: 277.1 像素
- **消息类型**: `sensor_msgs/Image`

### 红外相机左 (Infra1)

- **分辨率**: 1280 × 720 像素
- **帧率**: 30 Hz
- **话题**: `/camera/infra1/image_rect_raw`
- **消息类型**: `sensor_msgs/Image`

### 红外相机右 (Infra2)

- **分辨率**: 1280 × 720 像素
- **帧率**: 30 Hz
- **话题**: `/camera/infra2/image_rect_raw`
- **消息类型**: `sensor_msgs/Image`

### IMU传感器

- **采样率**: 200 Hz
- **话题**: `/imu`
- **消息类型**: `sensor_msgs/Imu`
- **测量**: 角速度、线性加速度

### 机器人里程计

- **话题**: `/x500/odometry`
- **消息类型**: `nav_msgs/Odometry`

---

## ROS2话题列表

启动后，以下话题应该可用：

```bash
# 查看所有话题
ros2 topic list

# 应该看到的相机话题
/camera/color/image_raw
/camera/color/camera_info
/camera/depth/image_rect_raw
/camera/infra1/image_rect_raw
/camera/infra2/image_rect_raw
/camera/imu

# 机器人状态话题
/x500/odometry
/joint_states
/tf
/tf_static
```

---

## 可视化和调试

### 查看RGB图像

```bash
# 方式1：使用RViz（已配置）
# 在启动脚本中自动启动的RViz窗口中查看

# 方式2：使用rqt_image_view
ros2 run rqt_image_view rqt_image_view
# 选择话题：/camera/color/image_raw

# 方式3：命令行查看话题信息
ros2 topic echo /camera/color/image_raw
```

### 查看IMU数据

```bash
ros2 topic echo /imu
```

### 查看机器人状态

```bash
ros2 topic echo /x500/odometry
```

### 可视化坐标系变换

```bash
# RViz中已包含TF可视化，或使用tf2工具
ros2 run tf2_tools view_frames
dot -Tsvg frames.pdf > frames.svg
```

---

## 关键配置详情

### XACRO参数

**文件**: `urdf/x500_d435i_full.xacro`

主要链接和关节：
- `base_link`: 无人机主体
- `d435_bottom_screw_frame`: D435i底部安装框架
- `d435_depth_frame`: 深度相机帧
- `d435_infra1_frame`: 左IR相机帧
- `d435_infra2_frame`: 右IR相机帧

相机安装位置（相对于base_link）：
```xml
<origin xyz="0.12 0.03 0.08" rpy="0 0 0"/>
```
- X: 12cm向前
- Y: 3cm向右
- Z: 8cm向上

### Gazebo插件

激活的GZ-Sim7系统插件：
- `gz-sim-physics-system`: 物理引擎
- `gz-sim-sensors-system`: 传感器管理
- `gz-sim-joint-state-publisher-system`: 关节状态发布
- `gz-sim-odometry-publisher-system`: 里程计发布
- `gz-sim-imu-system`: IMU处理

---

## 环境变量

确保设置以下环境变量以获得最佳性能：

```bash
# 添加模型和世界文件路径
export GZ_SIM_RESOURCE_PATH="/home/ubuntu/Desktop/px4-ros2-uavctl/urdf:\
/home/ubuntu/Desktop/px4-ros2-uavctl/world:\
/usr/share/gz/gz-sim7/models:\
/usr/share/ignition/ignition-gazebo6/models"

# 设置ROS2通信
export ROS_DOMAIN_ID=0

# 设置Gazebo分区
export GZ_PARTITION=default
```

---

## 常见问题与解决方案

### 问题1：Gazebo无法加载模型

**症状**: 启动时Gazebo崩溃或模型不显示

**解决方案**:
```bash
# 检查资源路径
echo $GZ_SIM_RESOURCE_PATH

# 重置资源路径
unset GZ_SIM_RESOURCE_PATH
export GZ_SIM_RESOURCE_PATH="/home/ubuntu/Desktop/px4-ros2-uavctl/urdf:/home/ubuntu/Desktop/px4-ros2-uavctl/world:/usr/share/gz/gz-sim7/models"
```

### 问题2：ROS2话题未发布

**症状**: 运行`ros2 topic list`时看不到相机话题

**解决方案**:
```bash
# 检查ros_gz_bridge是否正在运行
ps aux | grep parameter_bridge

# 手动启动桥接
ros2 run ros_gz_bridge parameter_bridge \
  '/camera/color/image_raw@sensor_msgs/msg/Image[gz.msgs.Image'
```

### 问题3：RViz显示不正确

**症状**: RViz中看不到机器人或传感器

**解决方案**:
```bash
# 重新加载RViz配置
rviz2 -d /home/ubuntu/Desktop/px4-ros2-uavctl/rviz/x500_d435i.rviz

# 在RViz中检查：
# - Fixed Frame应为: base_link
# - 显示面板中启用RobotModel和TF
```

---

## 性能调优

### 降低仿真速度（提高精度）

编辑 `world/warehouse_x500_d435i.sdf`:
```xml
<physics name="1ms" type="ignored">
  <max_step_size>0.001</max_step_size>    <!-- 减小此值 -->
  <real_time_factor>0.5</real_time_factor>  <!-- 设为 < 1.0 -->
</physics>
```

### 提高帧率（降低精度）

在XACRO中修改传感器频率：
```xml
<update_rate>60</update_rate>  <!-- 从30Hz增加到60Hz -->
```

### 禁用不需要的传感器

注释掉`x500_d435i_full.xacro`中的传感器块以禁用它们。

---

## 数据记录

### 录制rosbag

```bash
# 记录所有话题
ros2 bag record -a

# 记录仅相机话题
ros2 bag record /camera/color/image_raw /camera/depth/image_rect_raw /imu
```

### 回放rosbag

```bash
ros2 bag play <bag_file>
```

---

## 来源和参考

- **D435i配置**: 来自 `src/depth_d435` 包
- **Warehouse环境**: Fuel Gazebo Models (https://fuel.gazebosim.org/)
- **X500无人机**: PX4自动驾驶仪官方模型
- **GZ-Sim7**: Gazebo现代模拟引擎

---

## 许可证和属性

- 本集成基于depth_d435包的配置
- 遵守相应的开源许可证
- Warehouse模型由OpenRobotics提供

---

## 支持和反馈

如遇到问题，请检查：
1. 所有文件是否正确创建（运行test脚本）
2. ROS2环境变量是否正确设置
3. Gazebo和gz-sim7是否正确安装
4. 日志输出中的错误信息

