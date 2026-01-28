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
ssh-copy-id orangepi@192.168.3.17

# 测试连接
ssh orangepi@192.168.3.17 "echo OK"
```

---

## ⚙️ 远端信息

```
IP:       192.168.3.17
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
