# 🚁 PX4-ROS2 无人机 Offboard 控制系统

> 一个完整的、生产级的无人机自动控制系统,支持 PX4 飞控和 ROS2

## �� 项目完成状态

✅ **部署系统** - 完整的自动化部署工具  
✅ **启动脚本** - 一键启动所有组件  
✅ **诊断工具** - 完整的系统诊断功能  
✅ **核心代码** - Offboard Control C++ 库  
✅ **文档** - 从快速开始到故障排除的完整文档  
⚠️ **飞控集成** - 等待位置数据（见诊断结果）

## 🚀 快速开始

### 最简单的方式

```bash
cd ~/Desktop/px4-ros2-uavctl
./startup/startup_realrobot.sh
```

这会自动启动:
- 远端 MicroXRCEAgent (飞控通信)
- 远端 Offboard Control 节点
- 本机 QGroundControl (可选)

### 更多信息

- 📖 **完整指南**: 查看 `SYSTEM_SETUP_GUIDE.md`
- 🚀 **快速启动**: 查看 `QUICK_START.txt`
- 🐛 **故障排除**: 查看 `docs/08-OffboardControl-Troubleshooting.md`
- 📋 **项目总结**: 查看 `PROJECT_COMPLETION_SUMMARY.md`

## 🛠️ 核心工具

### 部署

```bash
# 快速部署菜单
bash deploy/quick_deploy.sh

# 完整部署脚本
bash deploy/deploy_offboard.sh
```

### 诊断

```bash
# 检查系统状态
bash deploy/diagnose_offboard.sh
```

### 启动

```bash
# 一键启动
./startup/startup_realrobot.sh

# 或直接启动 Offboard 节点
./startup/start_offboard_remote.sh
```

## 📚 文档结构

```
文档/
├── QUICK_START.txt                         ← 快速启动指南 🌟
├── SYSTEM_SETUP_GUIDE.md                   ← 完整系统指南 🌟
├── PROJECT_COMPLETION_SUMMARY.md           ← 项目总结 🌟
├── OFFBOARD_CONTROL_FIX.md                 ← 技术细节
└── docs/
    ├── 01-QuickStart.md
    ├── 02-QuickReference.md
    ├── 03-DeploymentGuide.md
    ├── 04-FAQ.md
    ├── 05-Examples.md
    ├── 06-FileStructure.md
    ├── 07-DeploymentSummary.md
    ├── 08-OffboardControl-Troubleshooting.md ← 故障排除
    └── 09-QuickStart-Offboard.md             ← Offboard 启动
```

## 💻 系统要求

### 本机
- Linux (Ubuntu 22.04+)
- SSH 客户端
- gnome-terminal
- git

### 香橙派
- ROS2 Humble
- PX4 ROS2 消息包
- MicroXRCEAgent
- px4_hexctl 包

### 硬件
- PX4 飞控 (USB 连接)
- 香橙派 (网络连接)

## 📊 系统架构

```
本机 ←SSH→ 香橙派 ←USB→ PX4飞控

启动脚本        MicroXRCEAgent      飞控
Offboard Ctrl   ROS2 节点          状态数据
QGC 地面站      心跳信号 (20Hz)    控制命令
```

## 🔍 快速诊断

遇到问题?

```bash
# 1. 运行诊断
bash deploy/diagnose_offboard.sh

# 2. 查看日志
ssh orangepi@192.168.3.17 "tail -f ~/.ros/log/latest/*/stdout"

# 3. 查看话题
ssh orangepi@192.168.3.17 "source ~/uav_ws/install/setup.bash && ros2 topic list"
```

## 📝 提交历史

最近的主要改进:

```
d055882 - Add: Quick start guide for easy reference
5543a43 - Project completion: Full deployment system with diagnostics ready
dbb5c50 - Add: Complete system setup guide and documentation
70325eb - Add: Offboard control quick start scripts and documentation
f10a72f - Fix: Auto-start Offboard Control in startup script
9841d2d - Add: Comprehensive diagnostics for offboard control issues
6886f2e - Fix: Improve offboard control initialization timing
```

## 🎓 学习资源

- [PX4 官方文档](https://docs.px4.io)
- [ROS2 文档](https://docs.ros.org)
- [MicroXRCE DDS 文档](https://micro-xrce-dds-docs.docs.eprosima.com/)

## 📞 支持

### 常见问题

**Q: SSH 连接失败**  
A: 检查密钥配置 `ssh-copy-id -i ~/.ssh/org_ed26619.pub orangepi@192.168.3.17`

**Q: Offboard 报 "no offboard signal"**  
A: 运行诊断脚本 `bash deploy/diagnose_offboard.sh`

**Q: 编译失败**  
A: 清理并重新编译 `bash deploy/clean_remote_build.sh && bash deploy/deploy_offboard.sh`

更多问题? 查看 `docs/08-OffboardControl-Troubleshooting.md`

## 📄 许可证

Project License

## 👨‍💻 作者

px4-ros2-uavctl 团队  
Created: 2026-01-28

---

**准备就绪?** → 查看 `QUICK_START.txt`  
**想了解更多?** → 查看 `SYSTEM_SETUP_GUIDE.md`  
**有问题?** → 查看 `docs/08-OffboardControl-Troubleshooting.md`

祝你飞行愉快! 🚁
