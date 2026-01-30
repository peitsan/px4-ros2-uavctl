# 🚁 PX4-ROS2 无人机 Offboard 控制系统

> 一个完善的、支持 PX4 飞控和 ROS2 的无人机 Offboard 开发与部署系统。

---

## 🚀 核心功能

✅ **自动化部署** - 一键将代码同步并编译至机载电脑 (OrangePi)。  
✅ **专业解锁序列** - 解决 "Arming denied" 问题，支持 EKF 状态自适应解锁。  
✅ **安全保障** - 支持 Ctrl+C 紧急降落并自动切回 Manual 模式。  
✅ **诊断系统** - 完整的链路与话题状态诊断工具。  
✅ **多语言支持** - 提供 C++ 和 Python 两套控制方式。

---

## 📂 项目结构

```
.
├── deploy/               # 🚀 部署脚本 (rsync + ssh + colcon)
├── docs/                 # 📚 详细文档与故障排除
├── src/                  # �� 源代码
│   ├── px4_hexctl/       # C++ 核心库与示例 (offboard_circle 等)
│   └── py_script/        # Python 参考脚本
├── startup/              # ⚡ 启动脚本 (远程启动 Agent 与 任务)
└── CMakeLists.txt        # 构建配置
```

---

## ⚡ 快速开始 (实机运行)

### 1. 本机配置
确保 `.ssh/config` 已配置 `orangepi` 的免密登录。

### 2. 一键启动
在项目根目录下执行：
```bash
./startup/startup_realrobot.sh
```
这会自动执行：
- 启动远端 `MicroXRCEAgent`。
- 启动远端 `Offboard Control` 核心节点。
- 本机启动 `QGroundControl` (如果已安装)。

### 3. 运行指定任务 (示例: 绕圆)
```bash
# 进入远端 SSH
# 执行绕圆飞行任务
ros2 run px4_hexctl offboard_circle
```

---

## 🔧 常用操作

### 部署与编译
```bash
# 快速部署菜单
./deploy/quick_deploy.sh

# 强制完全重新编译
./deploy/deploy_offboard.sh
```

### 诊断问题
```bash
# 运行综合诊断
./deploy/diagnose_offboard.sh
```

---

## 📚 文档指南

更多详细信息请参阅 `docs/` 目录：
- [📖 用户操作手册](docs/UserGuide.md)
- [🆘 常见问题与任务排除](docs/FAQ.md)
- [📝 飞行示例](docs/Examples.md)

---

## ⚠️ 开发注意事项

1. **Git 提交**：`build/`, `install/`, `log/` 已通过 `.gitignore` 排除，请勿手动提交。
2. **安全第一**：在执行 Offboard 任务时，请确保遥控器处于开启状态，随时准备切回 Manual 模式。
3. **频率要求**：Offboard 模式要求控制指令频率不低于 2Hz（本项目默认 20Hz - 50Hz）。

---
Created by px4-ros2-uavctl team | 2026-01