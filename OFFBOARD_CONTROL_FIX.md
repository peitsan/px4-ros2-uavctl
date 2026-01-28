# Offboard Control 修复文档

## 问题描述
无人机在启动 Offboard Control 节点后，尝试切换到 OFFBOARD 模式时报错 **"no offboard signal"**，无论无人机是否已经解锁都会出现此问题。

## 根本原因分析
1. **初始化顺序问题**: 原始代码中，心跳线程启动后立即尝试切换 OFFBOARD 模式，但没有给心跳线程足够的时间建立稳定的信号通道
2. **缺少预热阶段**: Offboard Control 需要先发送一定数量的心跳信号给 PX4，让 PX4 识别到有外部控制器在线，才能安全切换到 OFFBOARD 模式
3. **时序不当**: 在 `arm()` 命令发送前，必须确保 OFFBOARD 模式已正确建立

## 修复方案

### 1. 改进 `main.cpp` 初始化流程
- **添加 Vehicle 初始化后的延迟等待 (10秒)**：确保心跳线程已启动并稳定工作
- **在 `arm()` 前增加显式等待**：确保 Offboard 信号已被 PX4 接收
- **添加详细的日志输出**：便于诊断问题

```cpp
// 创建 Vehicle 实例（自动启动心跳线程和切换 OFFBOARD 模式）
auto vehicle = std::make_shared<Vehicle>();

// 等待系统初始化和心跳信号建立
std::cout << "⏳ Waiting for system initialization (10 seconds)..." << std::endl;
for (int i = 0; i < 10; i++) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
}

// 现在可以安全地解锁
vehicle->drone()->arm();
```

### 2. 改进 `Vehicle::Vehicle()` 构造函数
- **在启动心跳线程前等待 ROS2 spin 初始化**：确保 publisher/subscriber 已建立
- **启动心跳线程后等待 5 秒**：让心跳信号充分传播
- **然后调用 `engage_offboard_mode()`**

```cpp
// 等待 spin 开始工作
std::this_thread::sleep_for(std::chrono::milliseconds(500));

// 启动心跳线程
drone_->heartbeat_thread_start();

// 等待足够的时间让心跳信号建立
std::cout << "⏳ Waiting for heartbeat signals to establish (5 seconds)..." << std::endl;
std::this_thread::sleep_for(std::chrono::seconds(5));

// 切换到 OFFBOARD 模式
drone_->engage_offboard_mode(10, 2.0);
```

### 3. 增强 `engage_offboard_mode()` 日志
- 实时显示预热进度
- 显示当前的 setpoint 计数器值
- 提供更清晰的诊断信息

## 关键时序图

```
┌─────────────────────────────────────────────────────────────┐
│ Vehicle Constructor                                         │
├─────────────────────────────────────────────────────────────┤
│ 1. rclcpp::init()                                           │
│ 2. Create OffboardControl node                             │
│ 3. Create MultiThreadedExecutor & add node                │
│ 4. Start spin_thread (executes ROS2 callbacks)            │
│ 5. [WAIT 500ms] ← 让 spin 初始化完成                      │
│ 6. heartbeat_thread_start() ← 开始发送心跳信号             │
│ 7. [WAIT 5s] ← 让心跳信号充分建立                         │
│ 8. engage_offboard_mode(10, 2.0) ← 切换 OFFBOARD 模式   │
└─────────────────────────────────────────────────────────────┘
                        ↓
┌─────────────────────────────────────────────────────────────┐
│ main() 继续执行                                             │
├─────────────────────────────────────────────────────────────┤
│ 1. [WAIT 10s] ← 让 Offboard 信号充分建立                  │
│ 2. arm() ← 现在可以安全解锁                               │
│ 3. takeoff_command_global() ← 起飞                         │
│ 4. ... 其他飞行操作 ...                                   │
└─────────────────────────────────────────────────────────────┘
```

## 测试验证步骤

### 本机测试
1. 确保无人机已连接到香橙派
2. 运行启动脚本：`./startup/startup_realrobot.sh`
3. 观察日志输出，确保看到以下信息：
   - `⏳ Waiting for heartbeat signals to establish (5 seconds)...`
   - `✅ OFFBOARD mode command sent!`
   - `⏳ Waiting for system initialization (10 seconds)...`
   - `✅ ARM command sent`

### 在远端测试
```bash
ssh orangepi@192.168.3.17
source ~/uav_ws/install/setup.bash
ros2 run offboard_control_cpp offboard_control_main
```

预期输出：
- 不应该出现 "no offboard signal" 错误
- 应该成功起飞到 1.5 米高度
- 应该成功执行轨迹点飞行

## 修改的文件
1. `/src/offboard_control_cpp/src/main.cpp` - 添加初始化延迟和日志
2. `/src/offboard_control_cpp/src/offboard_control_lib/vehicle.cpp` - 改进构造函数
3. `/src/offboard_control_cpp/src/offboard_control_lib/offboard_control.cpp` - 增强 engage_offboard_mode 日志

## 相关 PX4 文档
- [PX4 Offboard Mode](https://docs.px4.io/main/en/flight_modes/offboard.html)
- [PX4 ROS 2 Offboard Control](https://docs.px4.io/main/en/ros/ros2_comm.html)

## 常见问题排查

### 仍然报 "no offboard signal" 错误
- 检查 MicroXRCEAgent 是否正常运行
- 检查飞控与香橙派的串口连接
- 查看 PX4 的详细日志

### Offboard 模式切换失败
- 确认无人机已解锁 (arm)
- 检查 Offboard Control 节点是否正常运行
- 查看 ROS2 话题是否有数据：
  ```bash
  ros2 topic echo /fmu/in/offboard_control_mode
  ros2 topic echo /fmu/in/trajectory_setpoint
  ```
