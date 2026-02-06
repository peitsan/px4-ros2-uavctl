# 📚 无人机系统用户手册

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
ssh orangepi@192.168.5.163
mkdir -p /home/orangepi/uav_ws/src/px4_hexctl
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
ssh-copy-id orangepi@192.168.5.163
ssh orangepi@192.168.5.163 "echo OK"
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

- **IP**: 192.168.5.163
- **用户**: orangepi
- **密码**: orangepi
- **项目路径**: `/home/orangepi/uav_ws/src/px4_hexctl`

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

---
# 📖 快速参考卡

## 🎯 命令速查表

### 基础部署
```bash
./deploy/deploy_offboard.sh          # 完整部署（推送+编译）
./deploy/quick_deploy.sh             # 快速菜单
./deploy/deploy_offboard.sh -h       # 显示帮助
```

### 高级选项
```bash
-n, --no-commit         跳过git提交，仅推送
-s, --skip-ssh          仅提交，不推送
-m, --message MSG       自定义提交信息
-d, --date MMDD         指定日期（如0128）
```

### 快速场景
```bash
./deploy/quick_deploy.sh simple      # 推送代码
./deploy/quick_deploy.sh build       # 推送+编译
./deploy/quick_deploy.sh status      # 检查状态
```

---

## 💻 常用命令示例

```bash
# 完整部署（日常使用）
./deploy/deploy_offboard.sh

# 仅推送代码
./deploy/deploy_offboard.sh -n

# 仅本地提交
./deploy/deploy_offboard.sh -s

# 自定义提交信息
./deploy/deploy_offboard.sh -m "新增功能"

# 指定日期
./deploy/deploy_offboard.sh -d 0128

# 组合使用
./deploy/deploy_offboard.sh -m "hotfix" -n
```

---

## 🔐 SSH设置

```bash
# 生成密钥
ssh-keygen -t ed25519

# 复制到远端
ssh-copy-id orangepi@192.168.5.163

# 测试连接
ssh orangepi@192.168.5.163 "echo OK"
```

---

## ⚙️ 远端信息

```
IP:       192.168.5.163
User:     orangepi
Pass:     orangepi
Port:     22
Path:     /home/orangepi/uav_ws/src/px4_hexctl
```

---

## 📁 文件位置

```
deploy/
├── deploy_offboard.sh    主脚本
├── quick_deploy.sh       快速脚本
├── deploy_offboard.bat   Windows启动
└── deploy_config.sh      配置文件

docs/
├── 01-QuickStart.md      快速开始
├── 02-QuickReference.md  本文件
├── 03-DeploymentGuide.md 详细指南
└── ...
```

---

## 🐛 常见问题速解

| 问题 | 解决方案 |
|------|---------|
| ssh: command not found | 安装OpenSSH或使用Git Bash/WSL2 |
| Permission denied | 检查用户名/密码或配置SSH密钥 |
| git commit failed | 配置：`git config user.email "x@x.com"` |
| rsync: command not found | 安装：`apt install rsync` 或 `brew install rsync` |

---

## 💡 提示

- 📝 Git提交格式：`upload-mmdd`（mmdd为日期）
- 🚀 第一次推送复制整个项目，之后只同步修改
- ⚡ 自动排除：build、install、.git等大目录
- 🛠️ 可跳过任何步骤：-n（跳过提交）或-s（不推送）

---

**需要更多信息？**
- [03-DeploymentGuide.md](03-DeploymentGuide.md) - 详细功能
- [04-FAQ.md](04-FAQ.md) - 常见问题
- [05-Examples.md](05-Examples.md) - 实际例子

---

**版本**: 1.0 | MIT License | 2026-01-28

---
# 📖 完整部署指南

本文档提供关于部署脚本的完整功能说明和使用方法。

## 🎯 核心功能

### Git自动管理
- ✅ 自动暂存修改（git add）
- ✅ 自动生成提交（格式：upload-mmdd）
- ✅ 支持自定义提交信息
- ✅ 自动检测git状态

### SSH连接和认证
- ✅ SSH密钥认证（推荐）
- ✅ SSH密码认证（需sshpass）
- ✅ 自动超时处理
- ✅ 详细的连接日志

### 文件增量同步
- ✅ rsync增量同步
- ✅ 自动排除大目录（build、install等）
- ✅ 支持删除远端不存在的文件
- ✅ 带宽优化

### 远端编译部署
- ✅ colcon编译支持
- ✅ ROS2 Humble环境配置
- ✅ 增量编译优化
- ✅ 编译结果验证

---

## 🚀 详细使用

### 基础部署流程

```bash
./deploy/deploy_offboard.sh
```

执行步骤：
1. 检查系统依赖（git、ssh）
2. 验证git仓库
3. 检查git工作状态
4. 自动提交更改（格式：upload-0128）
5. 连接远端香橙派（SSH认证）
6. 使用rsync同步文件
7. 询问是否编译和部署
8. 显示完成摘要

### 命令行参数

#### `-h, --help` 显示帮助
```bash
./deploy/deploy_offboard.sh -h
```
显示详细的帮助信息和使用示例。

#### `-d, --date MMDD` 指定日期
```bash
./deploy/deploy_offboard.sh -d 0128
# 提交信息：upload-0128
```
用于指定特定的日期格式。

#### `-m, --message MSG` 自定义提交信息
```bash
./deploy/deploy_offboard.sh -m "新增PID控制"
# 提交信息：新增PID控制
```
完全自定义git提交信息。

#### `-n, --no-commit` 跳过git提交
```bash
./deploy/deploy_offboard.sh -n
```
仅推送代码，不提交git。用于代码已提交的情况。

#### `-s, --skip-ssh` 仅本地操作
```bash
./deploy/deploy_offboard.sh -s
```
仅进行本地git提交，不推送到远端。

### 参数组合

```bash
# 跳过提交，仅推送
./deploy/deploy_offboard.sh -n

# 自定义信息，跳过提交
./deploy/deploy_offboard.sh -m "hotfix" -n

# 本地提交，不推送
./deploy/deploy_offboard.sh -s

# 指定日期和自定义信息
./deploy/deploy_offboard.sh -d 0128 -m "release"
```

---

## 🔐 认证方式

### SSH密钥认证（推荐）

优势：
- ✅ 安全性高
- ✅ 自动化友好
- ✅ 无需输入密码

设置步骤：
```bash
# 1. 生成密钥
ssh-keygen -t ed25519 -C "your_email@example.com"

# 2. 复制到远端
ssh-copy-id orangepi@192.168.5.163

# 3. 测试连接（应无需输入密码）
ssh orangepi@192.168.5.163 "echo OK"

# 4. 运行部署脚本
./deploy/deploy_offboard.sh
```

### SSH密码认证

需要安装sshpass：
```bash
# Ubuntu/Debian
sudo apt-get install sshpass

# macOS
brew install sshpass

# Windows (WSL2)
apt-get install sshpass
```

然后脚本会自动使用密码进行认证。

---

## 🔄 工作流程详解

```
┌─────────────────────────────┐
│  1. 检查系统依赖             │
│  (git, ssh, sshpass)        │
└────────────┬────────────────┘
             │
┌────────────▼────────────────┐
│  2. 验证Git仓库             │
│  检查.git目录               │
└────────────┬────────────────┘
             │
┌────────────▼────────────────┐
│  3. 检查工作区修改           │
│  (可跳过: -n)               │
└────────────┬────────────────┘
             │
┌────────────▼────────────────┐
│  4. 生成并执行提交           │
│  格式: upload-mmdd          │
│  (可跳过: -n)               │
└────────────┬────────────────┘
             │
┌────────────▼────────────────┐
│  5. 检查SSH连接             │
│  (可跳过: -s)               │
└────────────┬────────────────┘
             │
┌────────────▼────────────────┐
│  6. 增量同步文件             │
│  使用rsync                  │
└────────────┬────────────────┘
             │
┌────────────▼────────────────┐
│  7. 交互询问                │
│  是否编译和部署？            │
└────────┬───────────────┬────┘
         │               │
        YES             NO
         │               │
    ┌────▼──┐       ┌────▼──┐
    │ 编译  │       │ 完成  │
    └────┬──┘       └───────┘
         │
    ┌────▼──┐
    │ 部署  │
    └────┬──┘
         │
    ┌────▼────────┐
    │ 显示摘要    │
    └─────────────┘
```

---

## 📊 Git提交格式

### 默认格式

自动生成：`upload-mmdd`
- `mm` = 月份（01-12）
- `dd` = 日期（01-31）

**示例：**
- 2026年1月28日 → `upload-0128`
- 2026年12月31日 → `upload-1231`

### 自定义格式

```bash
./deploy/deploy_offboard.sh -m "自定义消息"
```

---

## 📁 rsync文件同步

### 自动排除的目录
```
.git                # git仓库
build               # cmake构建目录
install             # 安装目录
log                 # 日志目录
.vscode             # IDE配置
.DS_Store           # macOS元数据
deploy              # 部署目录
docs                # 文档目录
```

### 同步流程
- 第一次：复制整个项目
- 之后：仅同步修改的文件（增量更新）
- 自动删除远端不存在的本地文件

---

## 🔧 配置文件

编辑 `deploy/deploy_config.sh` 可自定义：

```bash
# 远端信息
DEPLOY_REMOTE_USER="orangepi"
DEPLOY_REMOTE_IP="192.168.5.163"
DEPLOY_REMOTE_PASSWORD="orangepi"
DEPLOY_REMOTE_PROJECT_PATH="/home/orangepi/..."

# Git配置
DEPLOY_DATE_FORMAT="mmdd"
DEPLOY_COMMIT_PREFIX="upload"

# 编译配置
DEPLOY_BUILD_COMMAND="colcon build --symlink-install"

# 网络配置
DEPLOY_SSH_TIMEOUT="10"
```

---

## 🎨 日志输出

脚本提供彩色、带时间戳的详细日志：

```
[INFO 2026-01-28 10:30:45] 检查系统依赖...
[INFO 2026-01-28 10:30:45] 依赖检查完成✓
[WARN 2026-01-28 10:30:46] 当前没有修改，继续执行...
[ERROR 2026-01-28 10:30:47] 无法连接到远端主机
```

---

## 💡 最佳实践

1. **首次使用配置SSH密钥** - 提高安全性和效率
2. **配置git用户信息** - 避免提交失败
3. **定期检查部署状态** - `./deploy/quick_deploy.sh status`
4. **使用有意义的提交信息** - 便于版本管理
5. **保持网络连接稳定** - 避免同步中断

---

**相关文档：**
- [01-QuickStart.md](01-QuickStart.md) - 快速开始
- [04-FAQ.md](04-FAQ.md) - 常见问题
- [05-Examples.md](05-Examples.md) - 实际例子

---

**版本**: 1.0 | MIT License | 2026-01-28
