# 📁 文件结构说明

## 项目目录结构

```
px4-ros2-vehicle-offboardcontrol/
│
├── 📁 deploy/                          ⭐ 部署脚本目录
│   ├── deploy_offboard.sh              主部署脚本
│   ├── quick_deploy.sh                 快速部署脚本
│   ├── deploy_offboard.bat             Windows启动器
│   └── deploy_config.sh                配置文件
│
├── 📁 docs/                            📚 文档目录
│   ├── README.md                       文档导航
│   ├── 01-QuickStart.md                快速开始指南
│   ├── 02-QuickReference.md            快速参考卡
│   ├── 03-DeploymentGuide.md           完整部署指南
│   ├── 04-FAQ.md                       常见问题
│   ├── 05-Examples.md                  使用示例
│   ├── 06-FileStructure.md             文件结构（本文件）
│   └── 07-DeploymentSummary.md         部署总结
│
├── 📁 src/                             源代码目录
│   ├── px4_hexctl/           C++控制实现
│   │   ├── include/
│   │   ├── src/
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   ├── py_script/                      Python脚本
│   │   ├── main.py
│   │   ├── main_srv.py
│   │   └── offboard_control_lib.py
│   └── startup/                        启动脚本
│       ├── simulation-gazebo
│       └── startup.sh
│
├── .git/                               Git仓库
├── .vscode/                            VSCode配置
├── README.md                           项目主文档
└── vehicle.hpp                         头文件（带中文注释）
```

---

## 🌟 核心文件说明

### deploy/ 目录

部署系统的核心脚本所在目录。

#### deploy_offboard.sh (11.7 KB)
**主部署脚本**

功能：
- ✅ 自动Git提交
- ✅ SSH连接和认证
- ✅ 增量文件同步（rsync）
- ✅ 远端编译（colcon）
- ✅ 详细日志输出

使用：
```bash
./deploy_offboard.sh              # 完整部署
./deploy_offboard.sh -h           # 显示帮助
./deploy_offboard.sh -m "msg"     # 自定义信息
./deploy_offboard.sh -n           # 跳过提交
```

#### quick_deploy.sh (4.1 KB)
**快速部署脚本**

预置场景：
- simple - 仅推送代码
- build - 推送+编译
- full - 推送+编译+部署
- test - 测试模式
- status - 检查状态

使用：
```bash
./quick_deploy.sh              # 交互菜单
./quick_deploy.sh simple       # 推送代码
./quick_deploy.sh status       # 检查状态
```

#### deploy_offboard.bat (2.5 KB)
**Windows启动器**

功能：
- 检查git和ssh安装
- 调用deploy_offboard.sh脚本
- UTF-8编码支持

使用：
```cmd
deploy\deploy_offboard.bat
```

#### deploy_config.sh (5.9 KB)
**配置文件**

可配置项：
- 远端设备信息（用户、IP、密码）
- Git配置
- rsync排除规则
- 编译参数
- 网络设置

编辑此文件自定义部署参数。

---

### docs/ 目录

完整的文档和使用指南。

#### README.md
**文档导航和索引**

功能：
- 快速导航到各个文档
- 按需求查找文档
- 推荐阅读顺序

#### 01-QuickStart.md
**快速开始指南（推荐首先阅读）**

内容：
- 3步快速部署
- 常用命令
- SSH认证设置
- 常见场景

#### 02-QuickReference.md
**快速参考卡**

内容：
- 命令速查表
- 常用命令示例
- 远端设备信息
- 常见问题速解

#### 03-DeploymentGuide.md
**完整部署指南**

内容：
- 核心功能说明
- 详细使用方法
- 工作流程图
- 配置说明
- 最佳实践

#### 04-FAQ.md
**常见问题与解答**

包含：
- 环境和依赖问题
- SSH和认证问题
- Git相关问题
- 网络和同步问题
- 编译和部署问题
- 故障排除流程

#### 05-Examples.md
**使用示例**

包含13个场景：
- 基础部署示例
- 实际工作流程
- 高级用法
- 故障排除示例

#### 06-FileStructure.md
**文件结构说明（本文件）**

内容：
- 项目目录结构
- 文件功能说明
- 使用指南
- 快速查询表

#### 07-DeploymentSummary.md
**部署总结**

内容：
- 项目统计
- 功能清单
- 快速参考
- 版本信息

---

## src/ 目录

项目源代码。

### px4_hexctl/
**C++ 控制实现**

```
px4_hexctl/
├── include/
│   └── px4_hexctl/
│       ├── offboard_control.hpp    (已添加中文注释)
│       └── vehicle.hpp              (已添加中文注释)
├── src/
│   ├── main.cpp
│   ├── offboard_control.cpp
│   └── vehicle.cpp
├── CMakeLists.txt
└── package.xml
```

### py_script/
**Python 脚本**

```
py_script/
├── main.py              主脚本
├── main2.py             备选脚本
├── main_srv.py          服务脚本
├── offboard_control_lib.py   控制库
└── test_rect.py         测试脚本
```

### startup/
**启动脚本**

```
startup/
├── simulation-gazebo    Gazebo仿真
└── startup.sh           启动脚本
```

---

## 📖 文档导航指南

### 按用户类型选择

**👤 新手用户**
1. [docs/01-QuickStart.md](docs/01-QuickStart.md)
2. 运行 `./deploy/quick_deploy.sh`
3. [docs/05-Examples.md](docs/05-Examples.md)

**🔧 高级用户**
1. [docs/03-DeploymentGuide.md](docs/03-DeploymentGuide.md)
2. [deploy/deploy_config.sh](deploy/deploy_config.sh)
3. 修改脚本以适应特殊需求

**🪟 Windows用户**
1. [docs/01-QuickStart.md](docs/01-QuickStart.md)
2. 运行 `deploy\deploy_offboard.bat`
3. 或在WSL2中使用bash脚本

### 按需求查找

| 需求 | 文件 |
|------|------|
| 快速上手 | docs/01-QuickStart.md |
| 查询命令 | docs/02-QuickReference.md |
| 学习详细用法 | docs/03-DeploymentGuide.md |
| 遇到问题 | docs/04-FAQ.md |
| 查看例子 | docs/05-Examples.md |
| 自定义参数 | deploy/deploy_config.sh |
| 查看脚本帮助 | `./deploy/deploy_offboard.sh -h` |

---

## 💾 文件大小统计

### 脚本文件
```
deploy_offboard.sh      11.7 KB
deploy_config.sh         5.9 KB
quick_deploy.sh          4.1 KB
deploy_offboard.bat      2.5 KB
────────────────────────────────
总计：24.2 KB
```

### 文档文件
```
01-QuickStart.md         ~5 KB
02-QuickReference.md     ~4 KB
03-DeploymentGuide.md    ~10 KB
04-FAQ.md                ~8 KB
05-Examples.md          ~10 KB
06-FileStructure.md      本文件
07-DeploymentSummary.md  ~8 KB
README.md                ~3 KB
────────────────────────────────
总计：~48 KB
```

---

## 🔍 快速查询

### 我需要...

| 需求 | 查看 |
|------|------|
| 快速部署 | `./deploy/quick_deploy.sh` |
| 完整控制 | `./deploy/deploy_offboard.sh` |
| 自定义参数 | `deploy/deploy_config.sh` |
| 快速参考 | `docs/02-QuickReference.md` |
| 详细说明 | `docs/03-DeploymentGuide.md` |
| 解决问题 | `docs/04-FAQ.md` |
| 看示例 | `docs/05-Examples.md` |
| 帮助信息 | `./deploy/deploy_offboard.sh -h` |

---

## 🚀 快速开始

1. **首次阅读**
   ```bash
   cat docs/01-QuickStart.md
   ```

2. **首次运行**
   ```bash
   ./deploy/quick_deploy.sh
   ```

3. **查询命令**
   ```bash
   cat docs/02-QuickReference.md
   ```

4. **遇到问题**
   ```bash
   cat docs/04-FAQ.md
   ```

5. **深入学习**
   ```bash
   cat docs/03-DeploymentGuide.md
   ```

---

**版本**: 1.0 | MIT License | 2026-01-28
