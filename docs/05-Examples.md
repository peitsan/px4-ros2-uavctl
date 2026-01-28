# 📖 使用示例

本文档提供13个常见场景的实际使用例子。

## 基础示例

### 例1：最简单的部署（一键式）

```bash
$ cd ~/px4-ros2-vehicle-offboardcontrol
$ ./deploy/deploy_offboard.sh
```

**执行流程：**
1. 自动生成提交信息 `upload-0128`（基于当前日期）
2. 提交本地更改
3. 连接到香橙派并同步代码
4. 询问是否编译

---

### 例2：使用快速部署菜单

```bash
$ ./deploy/quick_deploy.sh

===============================================================
  快速部署工具
===============================================================

可用场景：
  1. simple  - 简单推送（仅同步代码）
  2. build   - 编译推送（同步+编译）
  3. full    - 完整部署（同步+编译+运行）
  4. test    - 测试部署（干运行模式）
  5. status  - 检查状态
  0. 退出

请选择操作 (0-5): 2
```

选择 `2` 后开始编译部署。

---

### 例3：直接指定快速部署场景

```bash
# 仅推送代码
$ ./deploy/quick_deploy.sh simple

# 推送并编译
$ ./deploy/quick_deploy.sh build

# 检查部署状态
$ ./deploy/quick_deploy.sh status
```

---

## 命令行参数示例

### 例4：跳过Git提交，仅推送代码

场景：代码已经提交到git，只想推送到远端

```bash
$ ./deploy/deploy_offboard.sh -n
```

**效果：**
- 跳过 `git commit` 步骤
- 直接进行SSH连接和文件同步
- 询问是否进行远端编译

---

### 例5：仅进行本地Git提交，不推送到远端

场景：只想备份代码到git，不上传到香橙派

```bash
$ ./deploy/deploy_offboard.sh -s
```

**效果：**
- 执行git add和commit
- 跳过SSH和文件推送步骤
- 完成后退出

---

### 例6：自定义提交信息

场景：想使用特定的提交信息

```bash
# 自定义提交信息
$ ./deploy/deploy_offboard.sh -m "新增PID控制算法"

# 指定日期
$ ./deploy/deploy_offboard.sh -d 0201

# 组合使用
$ ./deploy/deploy_offboard.sh -m "修复陀螺仪校准" -n
```

---

## 实际工作流示例

### 例7：日常开发流程

**场景：** 完成一天的开发工作，需要部署到香橙派进行测试

```bash
# 步骤1：进入项目目录
$ cd ~/px4-ros2-vehicle-offboardcontrol

# 步骤2：查看修改（可选）
$ git status
On branch main
Changes not staged for commit:
  modified:   src/offboard_control_cpp/src/offboard_control.cpp
  modified:   src/py_script/main.py

# 步骤3：运行部署脚本
$ ./deploy/deploy_offboard.sh

[INFO] 检查系统依赖...
[INFO] 依赖检查完成✓
[INFO] 执行Git提交...
[INFO] 提交信息: upload-0128
[main 1a2b3c4] upload-0128
 2 files changed, 50 insertions(+), 5 deletions(-)

[INFO] 检查与远端香橙派的连接...
[INFO] 远端连接检查完成✓
[INFO] 同步代码到远端香橙派...
[INFO] 使用rsync进行增量文件同步...
sending incremental file list
src/offboard_control_cpp/src/offboard_control.cpp
src/py_script/main.py
sent 2,345 bytes  received 1,234 bytes

# 步骤4：脚本提示选择，输入 y 进行远端编译
是否在远端执行编译和部署？(y/n) y

# 步骤5：等待编译完成
[INFO] 在远端执行编译...
[100%] Built target offboard_control
[INFO] 远端编译成功✓

# 完成！
===============================================================
  部署任务完成！
===============================================================
```

---

### 例8：紧急hotfix部署

**场景：** 发现并修复了一个严重bug，需要快速部署

```bash
# 步骤1：修改并保存代码

# 步骤2：快速部署（使用自定义消息标记为hotfix）
$ ./deploy/deploy_offboard.sh -m "hotfix: 修复PID参数溢出"

[INFO] 执行Git提交...
[INFO] 提交信息: hotfix: 修复PID参数溢出
[main a1b2c3d] hotfix: 修复PID参数溢出
 1 file changed, 10 insertions(+), 3 deletions(-)

[INFO] 同步代码到远端香橙派...
...
[INFO] 代码同步成功✓

# 步骤3：应答编译询问
是否在远端执行编译和部署？(y/n) y

# 完成！代码已部署到香橙派并编译
```

---

### 例9：代码审查后的部署

**场景：** 代码审查通过，需要合并并部署到生产环境

```bash
# 步骤1：确保所有更改已保存
$ git add -A
$ git status

# 步骤2：运行部署脚本，使用release标签
$ ./deploy/deploy_offboard.sh -m "v1.2.0-release"

# 步骤3：远端编译
是否在远端执行编译和部署？(y/n) y

# 步骤4：验证部署（可选）
$ ssh orangepi@192.168.3.17
orangepi@192.168.3.17:~$ cd px4-ros2-vehicle-offboardcontrol
orangepi@192.168.3.17:~/px4-ros2-vehicle-offboardcontrol$ source install/setup.bash
orangepi@192.168.3.17:~/px4-ros2-vehicle-offboardcontrol$ ros2 launch ...
```

---

## 高级使用示例

### 例10：跳过编译，仅推送代码

**场景：** 只想上传源代码，计划稍后在香橙派上手动编译

```bash
$ ./deploy/deploy_offboard.sh
# 当提示时，回答 n
是否在远端执行编译和部署？(y/n) n

# 然后手动在香橙派上编译：
$ ssh orangepi@192.168.3.17
$ cd /home/orangepi/uav_ws/src/px4_hexctl
$ source /opt/ros/humble/setup.bash
$ colcon build --symlink-install
```

---

### 例11：批量部署多个分支

```bash
# 假设有多个git分支需要部署

# 部署feature1分支
$ git checkout feature1
$ ./deploy/deploy_offboard.sh -m "feature1分支"

# 部署feature2分支
$ git checkout feature2
$ ./deploy/deploy_offboard.sh -m "feature2分支"

# 返回main分支
$ git checkout main
```

---

### 例12：脚本化部署（用于CI/CD）

```bash
#!/bin/bash
# ci_deploy.sh - 自动化部署脚本

set -e

PROJECT_DIR="/path/to/project"
cd "$PROJECT_DIR"

# 获取git提交哈希作为版本标识
VERSION=$(git rev-parse --short HEAD)
MESSAGE="auto-deploy-${VERSION}"

# 执行部署
./deploy/deploy_offboard.sh -m "$MESSAGE" -n

echo "部署完成: $MESSAGE"
```

运行：
```bash
$ bash ci_deploy.sh
```

---

### 例13：检查部署状态

```bash
$ ./deploy/quick_deploy.sh status

检查部署状态...

1. 检查Git状态
On branch main
nothing to commit, working tree clean

2. 检查网络连接
✓ 香橙派网络连接正常

3. 检查SSH连接
✓ SSH连接正常
```

---

## 故障排除示例

### SSH连接问题诊断

```bash
# 检查SSH连接
$ ssh -v orangepi@192.168.3.17

# 如果显示 "Permission denied"，尝试手动输入密码
$ sshpass -p "orangepi" ssh orangepi@192.168.3.17 "echo OK"

# 配置SSH密钥（一次性）
$ ssh-keygen -t ed25519
$ ssh-copy-id orangepi@192.168.3.17

# 再次测试
$ ssh orangepi@192.168.3.17 "echo OK"
```

---

### Git配置问题

```bash
# 检查当前git配置
$ git config --list

# 如果未配置，设置用户信息
$ git config user.email "developer@example.com"
$ git config user.name "Developer Name"

# 重试部署
$ ./deploy/deploy_offboard.sh
```

---

## 总结

| 使用场景 | 推荐命令 |
|---------|---------|
| 日常开发 | `./deploy/deploy_offboard.sh` |
| 快速菜单 | `./deploy/quick_deploy.sh` |
| 仅推送代码 | `./deploy/deploy_offboard.sh -n` |
| 仅本地提交 | `./deploy/deploy_offboard.sh -s` |
| 自定义信息 | `./deploy/deploy_offboard.sh -m "msg"` |
| 检查状态 | `./deploy/quick_deploy.sh status` |

---

**相关文档：**
- [01-QuickStart.md](01-QuickStart.md) - 快速开始
- [03-DeploymentGuide.md](03-DeploymentGuide.md) - 详细功能
- [04-FAQ.md](04-FAQ.md) - 常见问题

---

**版本**: 1.0 | MIT License | 2026-01-28
