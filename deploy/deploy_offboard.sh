#!/bin/bash

################################################################################
# @file deploy_offboard.sh
# @brief 无人机离板控制项目一键部署脚本
# @details 
#   功能说明：
#   - 自动生成git提交信息（格式：upload-mmdd）
#   - 提交增量修改到本地git仓库
#   - 通过SSH将代码增量推送到香橙派
#   - 在远端执行编译和部署命令
#   - 支持交互式交叉编译和部署
#
#   使用方法：
#   ./deploy/deploy_offboard.sh [选项]
#
#   选项：
#   -h, --help          显示帮助信息
#   -d, --date MMDD     指定日期（格式MMDD，默认为系统当前日期）
#   -m, --message MSG   指定git提交信息（默认为upload-MMDD）
#   -n, --no-commit     跳过git提交步骤，仅推送现有代码
#   -s, --skip-ssh      仅进行本地git操作，不进行远端推送
#
# @author [Your Name]
# @date 2026-01-28
# @version 1.0
################################################################################

set -e  # 遇到错误立即退出

# ============================================================================
# 配置信息
# ============================================================================

# 远端香橙派信息
REMOTE_USER="orangepi"
REMOTE_IP="192.168.3.17"
REMOTE_HOST="${REMOTE_USER}@${REMOTE_IP}"
REMOTE_PASSWORD="orangepi"
REMOTE_PROJECT_PATH="/home/orangepi/uav_ws/src/px4_hexctl"

# 本地项目信息（获取deploy脚本所在目录的父目录）
LOCAL_PROJECT_PATH="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'  # No Color

# ============================================================================
# 函数定义
# ============================================================================

# 显示帮助信息
show_help() {
    cat << EOF
${BLUE}===============================================================${NC}
  无人机离板控制项目一键部署脚本
${BLUE}===============================================================${NC}

使用方法：
  ./deploy/deploy_offboard.sh [选项]

选项：
  -h, --help              显示此帮助信息
  -d, --date MMDD         指定日期（格式MMDD，如0128）
  -m, --message MSG       指定git提交信息
  -n, --no-commit         跳过git提交，仅推送代码
  -s, --skip-ssh          仅进行本地git操作，不推送到远端

示例：
  ./deploy/deploy_offboard.sh                    # 使用默认参数
  ./deploy/deploy_offboard.sh -d 0128            # 指定日期为01月28日
  ./deploy/deploy_offboard.sh -m "新增功能"      # 自定义提交信息
  ./deploy/deploy_offboard.sh -n                 # 跳过提交，仅推送
  ./deploy/deploy_offboard.sh -s                 # 仅提交，不推送

${BLUE}===============================================================${NC}
EOF
}

# 打印带时间戳的日志
log_info() {
    echo -e "${GREEN}[INFO $(date '+%Y-%m-%d %H:%M:%S')]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}[WARN $(date '+%Y-%m-%d %H:%M:%S')]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR $(date '+%Y-%m-%d %H:%M:%S')]${NC} $1"
}

# 检查依赖程序
check_dependencies() {
    log_info "检查系统依赖..."
    
    local missing_deps=()
    
    # 检查git
    if ! command -v git &> /dev/null; then
        missing_deps+=("git")
    fi
    
    # 检查ssh
    if ! command -v ssh &> /dev/null; then
        missing_deps+=("ssh")
    fi
    
    # 检查sshpass（用于非交互式密码认证）
    if ! command -v sshpass &> /dev/null; then
        log_warn "未找到sshpass，将使用ssh密钥认证或交互式输入"
    fi
    
    if [ ${#missing_deps[@]} -gt 0 ]; then
        log_error "缺少以下依赖程序: ${missing_deps[*]}"
        log_error "请先安装这些程序后再运行此脚本"
        exit 1
    fi
    
    log_info "依赖检查完成✓"
}

# 检查git仓库
check_git_repo() {
    log_info "检查Git仓库..."
    
    if [ ! -d "${LOCAL_PROJECT_PATH}/.git" ]; then
        log_error "未发现Git仓库，请在Git仓库中运行此脚本"
        exit 1
    fi
    
    log_info "Git仓库检查完成✓"
}

# 检查git状态
check_git_status() {
    log_info "检查Git状态..."
    
    cd "${LOCAL_PROJECT_PATH}"
    
    if git diff-index --quiet HEAD --; then
        log_warn "当前没有修改，继续执行..."
        return 0
    fi
    
    log_info "发现未提交的更改，将进行提交"
}

# 生成日期字符串（格式：mmdd）
generate_date_string() {
    if [ -n "$CUSTOM_DATE" ]; then
        echo "$CUSTOM_DATE"
    else
        date '+%m%d'
    fi
}

# 生成提交信息
generate_commit_message() {
    if [ -n "$CUSTOM_MESSAGE" ]; then
        echo "$CUSTOM_MESSAGE"
    else
        local date_str=$(generate_date_string)
        echo "upload-${date_str}"
    fi
}

# 执行git提交
do_git_commit() {
    log_info "执行Git提交..."
    
    cd "${LOCAL_PROJECT_PATH}"
    
    # 检查是否有未暂存的更改
    if ! git diff-index --quiet HEAD --; then
        log_info "暂存所有更改..."
        git add -A
    fi
    
    # 检查是否有暂存的更改
    if git diff-index --cached --quiet HEAD --; then
        log_warn "没有更改需要提交"
        return 0
    fi
    
    local commit_msg=$(generate_commit_message)
    log_info "提交信息: ${commit_msg}"
    
    git commit -m "${commit_msg}"
    
    log_info "Git提交成功✓"
}

# 检查远端连接
check_remote_connection() {
    log_info "检查与远端香橙派的连接..."
    
    if ! ssh -o ConnectTimeout=5 -o BatchMode=yes "${REMOTE_HOST}" "echo 'SSH连接成功'" > /dev/null 2>&1; then
        log_warn "SSH密钥认证失败，尝试使用密码认证..."
        
        # 尝试使用sshpass进行密码认证
        if command -v sshpass &> /dev/null; then
            if ! sshpass -p "${REMOTE_PASSWORD}" ssh -o ConnectTimeout=5 "${REMOTE_HOST}" "echo 'SSH连接成功'" > /dev/null 2>&1; then
                log_error "无法连接到远端主机 ${REMOTE_HOST}"
                log_error "请检查IP地址、用户名和密码是否正确"
                exit 1
            fi
            USE_SSHPASS=1
        else
            log_warn "请手动输入密码或配置SSH密钥认证"
            if ! ssh "${REMOTE_HOST}" "echo 'SSH连接成功'"; then
                log_error "无法连接到远端主机"
                exit 1
            fi
            USE_SSHPASS=0
        fi
    else
        log_info "使用SSH密钥认证"
        USE_SSHPASS=0
    fi
    
    log_info "远端连接检查完成✓"
}

# 同步代码到远端
sync_code_to_remote() {
    log_info "同步代码到远端香橙派..."
    
    # 获取最新提交的哈希值
    local latest_commit=$(git rev-parse HEAD)
    log_info "最新提交: ${latest_commit}"
    
    # 方法1：使用git push（如果远端已配置）
    # 方法2：使用rsync进行增量同步
    
    log_info "使用rsync进行增量文件同步..."
    
    local rsync_cmd="rsync -avz --delete \
        --exclude='.git' \
        --exclude='build' \
        --exclude='install' \
        --exclude='log' \
        --exclude='.vscode' \
        --exclude='.DS_Store' \
        --exclude='deploy' \
        --exclude='docs' \
        '${LOCAL_PROJECT_PATH}/' \
        '${REMOTE_HOST}:${REMOTE_PROJECT_PATH}/'"
    
    if [ $USE_SSHPASS -eq 1 ]; then
        sshpass -p "${REMOTE_PASSWORD}" bash -c "${rsync_cmd}"
    else
        bash -c "${rsync_cmd}"
    fi
    
    if [ $? -eq 0 ]; then
        log_info "代码同步成功✓"
    else
        log_error "代码同步失败"
        exit 1
    fi
}

# 在远端执行编译
build_on_remote() {
    log_info "在远端执行编译..."
    
    # 获取工作空间根目录（去掉 /src/px4_hexctl 部分）
    local workspace_root="/home/orangepi/uav_ws"
    
    # 按照依赖顺序编译包
    # 1. 先编译 px4_msgs (消息定义包，其他包依赖它)
    log_info "步骤 1/3: 编译 px4_msgs..."
    local build_px4_msgs="cd ${workspace_root} && \
        source /opt/ros/humble/setup.bash && \
        colcon build --packages-select px4_msgs --symlink-install 2>&1"
    
    if [ $USE_SSHPASS -eq 1 ]; then
        sshpass -p "${REMOTE_PASSWORD}" ssh -t "${REMOTE_HOST}" "${build_px4_msgs}"
    else
        ssh -t "${REMOTE_HOST}" "${build_px4_msgs}"
    fi
    
    if [ $? -ne 0 ]; then
        log_error "px4_msgs 编译失败"
        exit 1
    fi
    log_info "px4_msgs 编译成功✓"
    
    # 2. 编译 px4_ros_com (依赖 px4_msgs)
    log_info "步骤 2/3: 编译 px4_ros_com..."
    local build_px4_ros_com="cd ${workspace_root} && \
        source /opt/ros/humble/setup.bash && \
        source install/setup.bash && \
        colcon build --packages-select px4_ros_com --symlink-install 2>&1"
    
    if [ $USE_SSHPASS -eq 1 ]; then
        sshpass -p "${REMOTE_PASSWORD}" ssh -t "${REMOTE_HOST}" "${build_px4_ros_com}"
    else
        ssh -t "${REMOTE_HOST}" "${build_px4_ros_com}"
    fi
    
    if [ $? -ne 0 ]; then
        log_warn "px4_ros_com 编译失败（可能不存在，继续...）"
    else
        log_info "px4_ros_com 编译成功✓"
    fi
    
    # 3. 最后编译 px4_hexctl (依赖 px4_msgs)
    log_info "步骤 3/3: 编译 px4_hexctl..."
    local build_px4_hexctl="cd ${workspace_root} && \
        source /opt/ros/humble/setup.bash && \
        source install/setup.bash && \
        colcon build --packages-select px4_hexctl --symlink-install 2>&1"
    
    if [ $USE_SSHPASS -eq 1 ]; then
        sshpass -p "${REMOTE_PASSWORD}" ssh -t "${REMOTE_HOST}" "${build_px4_hexctl}"
    else
        ssh -t "${REMOTE_HOST}" "${build_px4_hexctl}"
    fi
    
    if [ $? -eq 0 ]; then
        log_info "px4_hexctl 编译成功✓"
        log_info "远端编译全部完成✓"
    else
        log_error "px4_hexctl 编译失败"
        exit 1
    fi
}

# 远端部署
deploy_on_remote() {
    log_info "在远端执行部署..."
    
    # 这里可以根据实际需求添加部署命令
    # 例如：启动服务、配置权限等
    
    local deploy_cmd="cd ${REMOTE_PROJECT_PATH} && \
        source /opt/ros/humble/setup.bash && \
        source install/setup.bash && \
        echo '部署完成' 2>&1"
    
    if [ $USE_SSHPASS -eq 1 ]; then
        sshpass -p "${REMOTE_PASSWORD}" ssh "${REMOTE_HOST}" "${deploy_cmd}"
    else
        ssh "${REMOTE_HOST}" "${deploy_cmd}"
    fi
    
    if [ $? -eq 0 ]; then
        log_info "远端部署成功✓"
    else
        log_error "远端部署失败"
        exit 1
    fi
}

# 显示部署摘要
show_summary() {
    cat << EOF

${GREEN}===============================================================${NC}
  部署任务完成！
${GREEN}===============================================================${NC}

部署信息摘要：
  - 本地项目路径: ${LOCAL_PROJECT_PATH}
  - 远端用户: ${REMOTE_USER}
  - 远端IP: ${REMOTE_IP}
  - 远端项目路径: ${REMOTE_PROJECT_PATH}
  - Git提交信息: $(generate_commit_message)
  - 部署时间: $(date '+%Y-%m-%d %H:%M:%S')

后续操作建议：
  1. 检查远端编译日志是否有错误
  2. 根据需要在远端手动运行程序进行测试
  3. 如有问题，请检查构建日志并修复

${GREEN}===============================================================${NC}

EOF
}

# ============================================================================
# 主程序
# ============================================================================

main() {
    # 初始化变量
    CUSTOM_DATE=""
    CUSTOM_MESSAGE=""
    SKIP_COMMIT=0
    SKIP_SSH=0
    USE_SSHPASS=0
    
    # 解析命令行参数
    while [[ $# -gt 0 ]]; do
        case $1 in
            -h|--help)
                show_help
                exit 0
                ;;
            -d|--date)
                CUSTOM_DATE="$2"
                shift 2
                ;;
            -m|--message)
                CUSTOM_MESSAGE="$2"
                shift 2
                ;;
            -n|--no-commit)
                SKIP_COMMIT=1
                shift
                ;;
            -s|--skip-ssh)
                SKIP_SSH=1
                shift
                ;;
            *)
                log_error "未知选项: $1"
                show_help
                exit 1
                ;;
        esac
    done
    
    # 打印欢迎信息
    echo ""
    echo -e "${BLUE}===============================================================${NC}"
    echo -e "${BLUE}  无人机离板控制项目一键部署脚本${NC}"
    echo -e "${BLUE}===============================================================${NC}"
    echo ""
    
    # 执行部署步骤
    check_dependencies
    check_git_repo
    check_git_status
    
    # Git提交步骤
    if [ $SKIP_COMMIT -eq 0 ]; then
        do_git_commit
    else
        log_warn "已跳过Git提交步骤"
    fi
    
    # SSH部署步骤
    if [ $SKIP_SSH -eq 0 ]; then
        check_remote_connection
        sync_code_to_remote
        
        # 询问是否进行远端编译和部署
        echo ""
        read -p "是否在远端执行编译和部署？(y/n) " -n 1 -r
        echo
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            build_on_remote
            deploy_on_remote
        else
            log_info "已跳过远端编译和部署"
        fi
    else
        log_warn "已跳过SSH部署步骤"
    fi
    
    # 显示摘要
    show_summary
}

# 捕获退出信号进行清理
trap 'log_error "脚本执行被中断"; exit 130' INT TERM

# 运行主程序
main "$@"
