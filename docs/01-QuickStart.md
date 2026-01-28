# 🚀 快速开始指南

## ⚡ 3步快速部署

### 步骤1：准备环境

**Windows用户：**
```bash
# 使用 Git Bash 或 WSL2
git --version
ssh -V
```

**Linux/macOS用户：**
```bash
sudo apt-get install git ssh rsync    # Ubuntu/Debian
brew install git openssh rsync        # macOS
```

### 步骤2：配置远端

```bash
ssh orangepi@192.168.3.17
mkdir -p /home/orangepi/px4-ros2-vehicle-offboardcontrol
exit
```

### 步骤3：运行部署

```bash
# 进入项目目录
cd ~/px4-ros2-vehicle-offboardcontrol

# 方法1：快速菜单
./deploy/quick_deploy.sh

# 方法2：完整部署
./deploy/deploy_offboard.sh

# 方法3：Windows批处理
deploy\deploy_offboard.bat
```

---

## 📋 常用命令

| 需求 | 命令 |
|------|------|
| 完整部署 | `./deploy/deploy_offboard.sh` |
| 快速菜单 | `./deploy/quick_deploy.sh` |
| 仅推送 | `./deploy/deploy_offboard.sh -n` |
| 仅提交 | `./deploy/deploy_offboard.sh -s` |
| 自定义信息 | `./deploy/deploy_offboard.sh -m "msg"` |
| 显示帮助 | `./deploy/deploy_offboard.sh -h` |

---

## 🔐 SSH认证

### 方案A：密钥认证（推荐）

```bash
ssh-keygen -t ed25519 -C "your_email@example.com"
ssh-copy-id orangepi@192.168.3.17
ssh orangepi@192.168.3.17 "echo OK"
```

### 方案B：密码认证

需要安装 sshpass，脚本会自动检测并使用

---

## 🎯 常见场景

### 日常开发
```bash
./deploy/deploy_offboard.sh
# 回答编译问题：y
```

### 仅推送代码
```bash
./deploy/deploy_offboard.sh -n
```

### 自定义提交信息
```bash
./deploy/deploy_offboard.sh -m "新增功能"
```

---

## ⚙️ 远端信息

- **IP**: 192.168.3.17
- **用户**: orangepi
- **密码**: orangepi
- **项目路径**: `/home/orangepi/px4-ros2-vehicle-offboardcontrol`

---

## 💡 提示

- ✅ 第一次推送会复制整个项目
- ✅ 之后只会同步修改的文件
- ✅ Git提交信息默认为 `upload-mmdd`
- ✅ 支持跳过任何步骤

---

**下一步：**
- 查看 [02-QuickReference.md](02-QuickReference.md) 了解完整命令
- 查看 [03-DeploymentGuide.md](03-DeploymentGuide.md) 深入学习
- 遇到问题？查看 [04-FAQ.md](04-FAQ.md)

---

**版本**: 1.0 | **更新**: 2026-01-28
