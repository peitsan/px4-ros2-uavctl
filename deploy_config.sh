#!/bin/bash
################################################################################
# deploy_config.sh - 部署脚本配置文件
# 
# 说明：
#   本配置文件定义了部署脚本的参数。
#   使用方法：source deploy_config.sh
#   或在deploy_offboard.sh中自动加载
#
# 最后更新: 2026-01-28
################################################################################

# ============================================================================
# 远端设备配置（香橙派信息）
# ============================================================================

# 香橙派用户名
DEPLOY_REMOTE_USER="orangepi"

# 香橙派IP地址
DEPLOY_REMOTE_IP="192.168.3.17"

# 香橙派SSH端口（默认22）
DEPLOY_REMOTE_PORT="22"

# 香橙派SSH密码
DEPLOY_REMOTE_PASSWORD="orangepi"

# 香橙派项目目录
DEPLOY_REMOTE_PROJECT_PATH="/home/orangepi/uav_ws/src/ros2-offboardcontrol"

# ============================================================================
# 本地项目配置
# ============================================================================

# 本地项目名称
DEPLOY_PROJECT_NAME="px4-ros2-vehicle-offboardcontrol"

# 本地ROS2环境设置脚本路径（本地编译时使用）
DEPLOY_LOCAL_ROS2_SETUP="/opt/ros/humble/setup.bash"

# ============================================================================
# Git配置
# ============================================================================

# 自动提交日期格式
# 可选值: mmdd (0128), yyyy-mmdd (2026-0128), full (2026-01-28)
DEPLOY_DATE_FORMAT="mmdd"

# Git提交信息前缀
DEPLOY_COMMIT_PREFIX="upload"

# Git提交者邮箱（如果未配置git）
DEPLOY_GIT_EMAIL="deploy@vehicle.local"

# Git提交者名称（如果未配置git）
DEPLOY_GIT_NAME="Deploy Script"

# ============================================================================
# 文件同步配置
# ============================================================================

# rsync排除规则列表（多个规则用空格分隔）
DEPLOY_RSYNC_EXCLUDES=(
    ".git"
    ".gitignore"
    "build"
    "install"
    "log"
    ".vscode"
    ".DS_Store"
    "*.swp"
    "*.swo"
    "*~"
    ".idea"
    ".cmake"
)

# 是否删除远端不存在的文件（--delete）
DEPLOY_RSYNC_DELETE="yes"

# ============================================================================
# 远端编译配置
# ============================================================================

# 是否自动编译
DEPLOY_AUTO_BUILD="no"

# ROS2环境（远端）
DEPLOY_REMOTE_ROS2_SETUP="/opt/ros/humble/setup.bash"

# 编译命令
DEPLOY_BUILD_COMMAND="colcon build --symlink-install"

# 编译超时时间（秒）
DEPLOY_BUILD_TIMEOUT="3600"

# 是否在编译前清理
DEPLOY_BUILD_CLEAN="no"

# ============================================================================
# 远端部署配置
# ============================================================================

# 是否自动部署
DEPLOY_AUTO_DEPLOY="no"

# 部署命令前缀
DEPLOY_DEPLOY_COMMAND="source install/setup.bash"

# ============================================================================
# 日志配置
# ============================================================================

# 日志文件目录
DEPLOY_LOG_DIR="${HOME}/.deploy_logs"

# 日志文件名称模式
DEPLOY_LOG_FILE_PATTERN="deploy-${PROJECT_NAME}-%Y%m%d-%H%M%S.log"

# 保留日志天数
DEPLOY_LOG_RETENTION_DAYS="7"

# ============================================================================
# 网络配置
# ============================================================================

# SSH连接超时时间（秒）
DEPLOY_SSH_TIMEOUT="10"

# SSH保活间隔（秒）
DEPLOY_SSH_ALIVE_INTERVAL="60"

# SSH保活计数
DEPLOY_SSH_ALIVE_COUNT="3"

# rsync带宽限制（KB/s，0表示无限制）
DEPLOY_RSYNC_BANDWIDTH="0"

# ============================================================================
# 备份配置
# ============================================================================

# 是否备份远端现有代码
DEPLOY_BACKUP_REMOTE="no"

# 备份目录名称
DEPLOY_BACKUP_DIR="backup-$(date +%m%d)"

# ============================================================================
# 通知配置
# ============================================================================

# 部署完成后发送邮件（需配置邮件）
DEPLOY_SEND_EMAIL="no"

# 邮件地址
DEPLOY_EMAIL_TO="your-email@example.com"

# ============================================================================
# 调试配置
# ============================================================================

# 是否启用调试模式
DEPLOY_DEBUG="no"

# 详细输出
DEPLOY_VERBOSE="yes"

# 干运行模式（只显示会执行的命令，不实际执行）
DEPLOY_DRY_RUN="no"

# ============================================================================
# 高级配置
# ============================================================================

# 自定义SSH命令参数
DEPLOY_SSH_EXTRA_ARGS=""

# 自定义rsync参数
DEPLOY_RSYNC_EXTRA_ARGS=""

# 部署前执行的本地脚本
DEPLOY_PRE_DEPLOY_SCRIPT=""

# 部署后执行的本地脚本
DEPLOY_POST_DEPLOY_SCRIPT=""

# 远端部署前执行的脚本
DEPLOY_REMOTE_PRE_DEPLOY_SCRIPT=""

# 远端部署后执行的脚本
DEPLOY_REMOTE_POST_DEPLOY_SCRIPT=""

# ============================================================================
# 导出变量供脚本使用
# ============================================================================

export DEPLOY_REMOTE_USER
export DEPLOY_REMOTE_IP
export DEPLOY_REMOTE_PORT
export DEPLOY_REMOTE_PASSWORD
export DEPLOY_REMOTE_PROJECT_PATH
export DEPLOY_PROJECT_NAME
export DEPLOY_LOCAL_ROS2_SETUP
export DEPLOY_DATE_FORMAT
export DEPLOY_COMMIT_PREFIX
export DEPLOY_GIT_EMAIL
export DEPLOY_GIT_NAME
export DEPLOY_AUTO_BUILD
export DEPLOY_AUTO_DEPLOY
export DEPLOY_DEBUG
export DEPLOY_VERBOSE
export DEPLOY_DRY_RUN
