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
   ssh orangepi@192.168.3.17
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
   ssh-copy-id orangepi@192.168.3.17
   ```

---

### Q5: SSH连接超时

**问题描述：** `ssh: connect to host 192.168.3.17 port 22: Connection timed out`

**原因可能：**
- 网络连接问题
- IP地址错误
- 防火墙阻止

**解决方案：**

1. **检查网络连接**
   ```bash
   ping 192.168.3.17
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
   ssh-copy-id orangepi@192.168.3.17
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
- 检查网络连接：`ping 192.168.3.17`
- 首次同步可能需要更长时间
- 配置带宽限制（如需要）

---

## 🏗️ 编译和部署

### Q12: 远端编译失败

**问题描述：** 远端colcon编译出错。

**解决方案：**

1. **检查ROS2是否安装**
   ```bash
   ssh orangepi@192.168.3.17
   source /opt/ros/humble/setup.bash
   ros2 --version
   ```

2. **手动编译测试**
   ```bash
   ssh orangepi@192.168.3.17
   cd /home/orangepi/px4-ros2-vehicle-offboardcontrol
   source /opt/ros/humble/setup.bash
   colcon build --symlink-install
   ```

3. **查看详细错误**
   ```bash
   cd /home/orangepi/px4-ros2-vehicle-offboardcontrol
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
DEPLOY_REMOTE_IP="192.168.3.17"
DEPLOY_REMOTE_PASSWORD="orangepi"
DEPLOY_REMOTE_PROJECT_PATH="/home/orangepi/px4-ros2-vehicle-offboardcontrol"
```

---

## 🐛 故障排除流程

如遇到问题，按以下流程排查：

1. **检查网络**
   ```bash
   ping 192.168.3.17
   ```

2. **检查SSH**
   ```bash
   ssh orangepi@192.168.3.17 "echo OK"
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
