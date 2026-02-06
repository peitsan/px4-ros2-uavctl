# ❓ 常见问题与解答

## 🔧 环境和依赖

### Q1: ssh: command not found

**问题描述：** 在Windows上运行脚本时显示找不到ssh命令。

**解决方案：**
1. **使用Git Bash** - Git安装包中包含ssh
   ```bash
   # 使用Git Bash而不是cmd或PowerShell
   "C:\Program Files\Git\bin\bash.exe" deploy\deploy_offboard.sh
   ```

2. **安装WSL2** - Windows子系统for Linux
   ```bash
   wsl --install
   # 然后在WSL中运行脚本
   ./deploy/deploy_offboard.sh
   ```

3. **安装OpenSSH** - Windows官方SSH实现
   ```powershell
   Add-WindowsCapability -Online -Name OpenSSH.Client
   ```

---

### Q2: git: command not found

**问题描述：** 系统中未安装git。

**解决方案：**
- **Windows**: 下载 [Git for Windows](https://git-scm.com/download/win)
- **macOS**: `brew install git`
- **Ubuntu/Debian**: `sudo apt-get install git`

---

### Q3: rsync: command not found

**问题描述：** rsync命令不可用。

**解决方案：**
- **Ubuntu/Debian**: `sudo apt-get install rsync`
- **macOS**: `brew install rsync`
- **Windows**: 使用Git Bash或WSL2

---

## 🔐 SSH和认证

### Q4: Permission denied (publickey, password)

**问题描述：** SSH连接被拒绝，无法认证。

**原因可能：**
- 用户名或密码错误
- SSH服务未启动
- SSH端口被阻止

**解决方案：**

1. **检查用户名和密码**
   ```bash
   ssh orangepi@192.168.5.163
   # 输入密码：orangepi
   ```

2. **在香橙派上启动SSH服务**
   ```bash
   sudo systemctl start openssh-server
   sudo systemctl enable openssh-server
   ```

3. **配置SSH密钥认证**
   ```bash
   ssh-keygen -t ed25519
   ssh-copy-id orangepi@192.168.5.163
   ```

---

### Q5: SSH连接超时

**问题描述：** `ssh: connect to host 192.168.5.163 port 22: Connection timed out`

**原因可能：**
- 网络连接问题
- IP地址错误
- 防火墙阻止

**解决方案：**

1. **检查网络连接**
   ```bash
   ping 192.168.5.163
   ```

2. **检查IP地址**
   ```bash
   # 在香橙派上运行
   ip addr show
   ```

3. **检查防火墙**
   ```bash
   # 在香橙派上允许SSH
   sudo ufw allow 22/tcp
   ```

---

### Q6: sshpass找不到

**问题描述：** 脚本提示未找到sshpass，无法使用密码认证。

**解决方案：**

1. **安装sshpass**
   - Ubuntu/Debian: `sudo apt-get install sshpass`
   - macOS: `brew install sshpass`

2. **或配置SSH密钥** - 推荐方式
   ```bash
   ssh-keygen -t ed25519
   ssh-copy-id orangepi@192.168.5.163
   ```

---

## 📝 Git相关

### Q7: git commit failed: Please tell me who you are

**问题描述：** git提交失败，需要配置用户信息。

**解决方案：**
```bash
git config user.email "your_email@example.com"
git config user.name "Your Name"

# 查看配置
git config --list
```

---

### Q8: nothing to commit, working tree clean

**问题描述：** 脚本提示没有修改需要提交。

**原因：** 没有做任何代码修改。

**解决方案：**
- 修改代码后再运行脚本
- 或使用 `-n` 选项跳过提交

---

### Q9: merge conflict in git

**问题描述：** 提交时出现合并冲突。

**解决方案：**
```bash
# 1. 检查冲突
git status

# 2. 解决冲突（编辑冲突文件）
# vim <conflicted-file>

# 3. 暂存解决后的文件
git add <conflicted-file>

# 4. 完成合并
git commit -m "resolve merge conflict"
```

---

## 🌐 网络和同步

### Q10: rsync sync failed: connection refused

**问题描述：** rsync同步失败，连接被拒绝。

**原因可能：** SSH连接失败

**解决方案：**
- 参考 Q4 和 Q5 的SSH问题解决方案

---

### Q11: 文件同步很慢

**问题描述：** 每次同步需要很长时间。

**原因可能：**
- 网络带宽低
- 第一次同步（需要复制整个项目）
- 有大量修改

**解决方案：**
- 检查网络连接：`ping 192.168.5.163`
- 首次同步可能需要更长时间
- 配置带宽限制（如需要）

---

## 🏗️ 编译和部署

### Q12: 远端编译失败

**问题描述：** 远端colcon编译出错。

**解决方案：**

1. **检查ROS2是否安装**
   ```bash
   ssh orangepi@192.168.5.163
   source /opt/ros/humble/setup.bash
   ros2 --version
   ```

2. **手动编译测试**
   ```bash
   ssh orangepi@192.168.5.163
   cd /home/orangepi/uav_ws/src/px4_hexctl
   source /opt/ros/humble/setup.bash
   colcon build --symlink-install
   ```

3. **查看详细错误**
   ```bash
   cd /home/orangepi/uav_ws/src/px4_hexctl
   colcon build --symlink-install --log-level debug
   ```

---

### Q13: build目录过大，同步慢

**问题描述：** build和install目录被同步到远端，占用空间和时间。

**解决方案：**
脚本会自动排除这些目录，不需要手动操作。

---

## 💡 使用技巧

### Q14: 如何跳过提交步骤？

```bash
./deploy/deploy_offboard.sh -n
```

使用 `-n` 选项仅推送代码，不提交。

---

### Q15: 如何自定义提交信息？

```bash
./deploy/deploy_offboard.sh -m "自定义消息"
```

使用 `-m` 选项指定自定义的提交信息。

---

### Q16: 如何仅本地提交，不推送？

```bash
./deploy/deploy_offboard.sh -s
```

使用 `-s` 选项跳过SSH和推送步骤。

---

### Q17: 如何检查部署状态？

```bash
./deploy/quick_deploy.sh status
```

检查：
- Git状态
- 网络连接
- SSH连接

---

### Q18: 如何更改远端IP或用户名？

编辑 `deploy/deploy_config.sh`：
```bash
DEPLOY_REMOTE_USER="orangepi"
DEPLOY_REMOTE_IP="192.168.5.163"
DEPLOY_REMOTE_PASSWORD="orangepi"
DEPLOY_REMOTE_PROJECT_PATH="/home/orangepi/uav_ws/src/px4_hexctl"
```

---

## 🐛 故障排除流程

如遇到问题，按以下流程排查：

1. **检查网络**
   ```bash
   ping 192.168.5.163
   ```

2. **检查SSH**
   ```bash
   ssh orangepi@192.168.5.163 "echo OK"
   ```

3. **检查Git**
   ```bash
   git status
   git config --list
   ```

4. **检查脚本日志**
   ```bash
   ./deploy/deploy_offboard.sh -h
   ```

5. **查看详细状态**
   ```bash
   ./deploy/quick_deploy.sh status
   ```

6. **查看相关文档**
   - [03-DeploymentGuide.md](03-DeploymentGuide.md) - 详细功能
   - [05-Examples.md](05-Examples.md) - 实际例子

---

## 📞 需要更多帮助？

- 查看 [README.md](README.md) 文档导航
- 查看 [03-DeploymentGuide.md](03-DeploymentGuide.md) 详细说明
- 运行 `./deploy/deploy_offboard.sh -h` 显示帮助

---

**版本**: 1.0 | MIT License | 2026-01-28
# PX4 Offboard Control "no offboard signal" 诊断和解决方案

## 症状
1. **Manual 上锁状态**: 运行程序后,QGC 显示切换到 Offboard 模式,但无法解锁,报错 "no offboard signal"
2. **Manual 飞行状态**: 运行程序后,QGC 一直保持 Manual 模式,无法切换到 Offboard

## 根本原因

从诊断脚本结果可以看出:

```
❌ 没有接收到位置数据 (/fmu/out/vehicle_local_position_v1)
❌ 没有接收到飞控状态数据 (/fmu/out/vehicle_status)
```

**虽然 MicroXRCEAgent 在运行,但 PX4 飞控没有向它发送位置和状态数据。**

这通常意味着:
- **飞控未能识别 MicroXRCEAgent 的连接**
- **通信参数不匹配（波特率等）**
- **飞控固件问题**

## 排查步骤

### 1️⃣ 验证飞控物理连接

```bash
# 在香橙派上检查串口设备
ssh orangepi@192.168.5.163 "ls -la /dev/ttyUSB* /dev/ttyACM*"

# 应该看到至少一个设备，例如:
# crw-rw---- 1 root dialout 188, 0  /dev/ttyUSB0
```

### 2️⃣ 检查飞控固件

在 QGC 中:
1. 打开菜单 > Vehicle Setup > Summary
2. 确认固件版本（应该是 PX4 4.x 以上）
3. 检查是否有任何警告或错误

### 3️⃣ 验证 MicroXRCEAgent 配置

当前启动的 MicroXRCEAgent:
```bash
MicroXRCEAgent serial -D /dev/ttyUSB0 -b 921600
```

**波特率可能需要调整。** 常见的 PX4 波特率:
- `921600` - 标准配置
- `921600` - 某些飞控使用
- `57600` - 旧版本

### 4️⃣ 检查 PX4 日志

在飞控连接到 QGC 时:
1. 点击 Toolbox > Analyze
2. 查看 Log Files
3. 查找与 MicroXRCE 或 XRCE 相关的错误信息

### 5️⃣ 测试基本通信

```bash
# 在香橙派上启动 MicroXRCEAgent，查看连接信息
ssh orangepi@192.168.5.163 
MicroXRCEAgent serial -D /dev/ttyUSB0 -b 115200 -v5
```

输出应该包含连接成功的日志。

## 常见问题和解决方案

### 问题 A: 串口权限不足

```bash
# 错误：Permission denied: /dev/ttyUSB0
# 解决：
ssh orangepi@192.168.5.163 "sudo usermod -a -G dialout orangepi"
# 然后重新登录 SSH
```

### 问题 B: MicroXRCEAgent 连接失败

症状: MicroXRCEAgent 运行但没有输出,或频繁断开

解决方案:
```bash
# 尝试不同的波特率
MicroXRCEAgent serial -D /dev/ttyUSB0 -b 921600

# 或者检查串口是否正确
ls -la /dev/ttyUSB*

# 如果有多个串口,尝试另一个
# /dev/ttyUSB0 vs /dev/ttyUSB1 vs /dev/ttyACM0
```

### 问题 C: 话题有发布者但没有数据

症状: 
- Offboard Control Mode 话题有发布者
- 但位置话题没有数据

原因: **飞控没有启动或 MicroXRCEAgent 连接失败**

解决:
1. 确保飞控已通电
2. 确保飞控通过 USB 连接到香橙派
3. 检查 QGC 中飞控的状态

### 问题 D: 无法切换 Offboard 模式

症状: QGC 中无法选择 Offboard 模式,或选后报 "no offboard signal"

原因:
1. 位置数据丢失（最常见）
2. Offboard 信号断开
3. 飞控固件不支持

解决:
```bash
# 验证位置数据
ssh orangepi@192.168.5.163 'source ~/uav_ws/install/setup.bash && ros2 topic echo /fmu/out/vehicle_local_position_v1' &

# 在另一个终端,启动 Offboard Control
ssh orangepi@192.168.5.163 'source ~/uav_ws/install/setup.bash && ros2 run px4_hexctl offboard_control_main'
```

位置数据应该继续输出。

## 快速诊断命令

```bash
# 一键诊断（在本机运行）
bash ~/Desktop/px4-ros2-uavctl/deploy/diagnose_offboard.sh

# 实时监控位置数据
ssh orangepi@192.168.5.163 'source ~/uav_ws/install/setup.bash && ros2 topic hz /fmu/out/vehicle_local_position_v1'

# 实时监控飞控状态
ssh orangepi@192.168.5.163 'source ~/uav_ws/install/setup.bash && ros2 topic hz /fmu/out/vehicle_status'

# 检查 Offboard 信号发送
ssh orangepi@192.168.5.163 'source ~/uav_ws/install/setup.bash && ros2 topic hz /fmu/in/offboard_control_mode'
```

## 最后的检查清单

- [ ] 飞控已通电
- [ ] 飞控通过 USB 连接到香橙派
- [ ] MicroXRCEAgent 正在运行（`ps aux | grep MicroXRCEAgent`）
- [ ] 串口权限正确（`ls -la /dev/ttyUSB*`）
- [ ] 位置话题有数据（`ros2 topic echo /fmu/out/vehicle_local_position_v1`）
- [ ] 飞控状态话题有数据（`ros2 topic echo /fmu/out/vehicle_status`）
- [ ] QGC 显示飞控已连接
- [ ] Offboard Control 节点正常运行

## 如果仍有问题

1. 收集日志:
   ```bash
   # PX4 日志（在 QGC 中下载）
   # ROS2 日志
   ssh orangepi@192.168.5.163 'cat ~/.ros/log/latest/offboard_control_center/0/stderr.log'
   ```

2. 检查飞控固件版本和配置
3. 确保使用了正确的波特率（通常 115200 或 921600）
4. 考虑升级或重新刷写飞控固件

## 相关文档

- [PX4 Offboard Mode](https://docs.px4.io/main/en/flight_modes/offboard.html)
- [MicroXRCEAgent 文档](https://micro-xrce-dds-docs.docs.eprosima.com/)
- [px4_msgs ROS2 包](https://github.com/PX4/px4_msgs)
