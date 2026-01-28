# 📚 部署文档索引

此目录包含关于项目部署的所有文档。

## 📖 快速导航

### 🚀 快速开始
- [快速开始指南](01-QuickStart.md) - 3分钟上手部署系统
- [快速参考卡](02-QuickReference.md) - 一页纸的命令速查表

### 📋 详细指南
- [完整部署指南](03-DeploymentGuide.md) - 详细的功能说明和使用方法
- [常见问题解答](04-FAQ.md) - 故障排除和问题解决
- [部署示例](05-Examples.md) - 13个实际使用场景

### 📁 参考文档
- [文件结构说明](06-FileStructure.md) - 项目文件组织说明
- [部署总结](07-DeploymentSummary.md) - 整体功能总结

## 🎯 按需求查找

| 需求 | 文档 |
|------|------|
| 快速上手 | [01-QuickStart.md](01-QuickStart.md) |
| 查询命令 | [02-QuickReference.md](02-QuickReference.md) |
| 学习详细用法 | [03-DeploymentGuide.md](03-DeploymentGuide.md) |
| 遇到问题 | [04-FAQ.md](04-FAQ.md) |
| 查看例子 | [05-Examples.md](05-Examples.md) |
| 了解结构 | [06-FileStructure.md](06-FileStructure.md) |
| 功能总结 | [07-DeploymentSummary.md](07-DeploymentSummary.md) |

## 💡 推荐阅读顺序

### 对于新用户
1. [01-QuickStart.md](01-QuickStart.md) - 了解基本概念
2. [02-QuickReference.md](02-QuickReference.md) - 学习常用命令
3. 运行脚本体验功能

### 对于进阶用户
1. [03-DeploymentGuide.md](03-DeploymentGuide.md) - 深入了解功能
2. [05-Examples.md](05-Examples.md) - 学习实际用法
3. [deploy_config.sh](../deploy/deploy_config.sh) - 自定义配置

### 遇到问题
1. [04-FAQ.md](04-FAQ.md) - 查找答案
2. 运行 `./deploy/quick_deploy.sh status` - 检查系统状态
3. 运行 `./deploy/deploy_offboard.sh -h` - 查看脚本帮助

## 🔗 相关资源

### 部署脚本位置
```
deploy/
├── deploy_offboard.sh      主部署脚本
├── quick_deploy.sh         快速部署脚本
├── deploy_offboard.bat     Windows启动器
└── deploy_config.sh        配置文件
```

### 运行方式
```bash
# Linux/macOS
cd deploy
./quick_deploy.sh              # 交互式菜单
./deploy_offboard.sh           # 完整部署

# Windows (Git Bash/WSL)
bash deploy/quick_deploy.sh
bash deploy/deploy_offboard.sh

# Windows (PowerShell)
deploy\deploy_offboard.bat
```

## 📞 快速帮助

**显示脚本帮助：**
```bash
./deploy/deploy_offboard.sh -h
```

**检查系统状态：**
```bash
./deploy/quick_deploy.sh status
```

**查看文档目录：**
```bash
ls -lah docs/
```

---

**版本**: 1.0 | **更新**: 2026-01-28 | **许可**: MIT
