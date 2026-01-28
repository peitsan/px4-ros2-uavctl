#!/bin/bash
################################################################################
# clean_remote_build.sh - 清理远端构建缓存
#
# 说明：
#   清理远端工作空间的构建文件,解决编译缓存问题
#
# 使用方法：
#   ./deploy/clean_remote_build.sh [选项]
#
# 选项：
#   -a, --all       清理所有包的构建缓存
#   -p, --package   清理指定包的构建缓存
#   -h, --help      显示帮助信息
#
# @date 2026-01-28
################################################################################

set -e

# 远端信息
REMOTE_USER="orangepi"
REMOTE_IP="192.168.3.17"
REMOTE_HOST="${REMOTE_USER}@${REMOTE_IP}"
WORKSPACE_ROOT="/home/orangepi/uav_ws"

# 颜色定义
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

show_help() {
    cat << EOF
${BLUE}===============================================================${NC}
  清理远端构建缓存工具
${BLUE}===============================================================${NC}

使用方法：
  ./deploy/clean_remote_build.sh [选项]

选项：
  -a, --all               清理所有包的构建缓存
  -p, --package PKG       清理指定包的构建缓存
  -h, --help              显示此帮助信息

示例：
  ./deploy/clean_remote_build.sh -a                    # 清理所有
  ./deploy/clean_remote_build.sh -p px4_msgs           # 仅清理px4_msgs
  ./deploy/clean_remote_build.sh -p offboard_control_cpp

EOF
}

# 清理所有构建缓存
clean_all() {
    echo -e "${YELLOW}清理所有构建缓存...${NC}"
    ssh "${REMOTE_HOST}" "cd ${WORKSPACE_ROOT} && rm -rf build/ install/ log/"
    echo -e "${GREEN}清理完成✓${NC}"
}

# 清理指定包的构建缓存
clean_package() {
    local package=$1
    echo -e "${YELLOW}清理 ${package} 的构建缓存...${NC}"
    ssh "${REMOTE_HOST}" "cd ${WORKSPACE_ROOT} && \
        rm -rf build/${package} install/${package} log/latest_build/${package}"
    echo -e "${GREEN}${package} 清理完成✓${NC}"
}

# 主逻辑
main() {
    if [ $# -eq 0 ]; then
        show_help
        exit 0
    fi

    case "$1" in
        -a|--all)
            clean_all
            ;;
        -p|--package)
            if [ -z "$2" ]; then
                echo "错误：请指定包名"
                exit 1
            fi
            clean_package "$2"
            ;;
        -h|--help)
            show_help
            ;;
        *)
            echo "未知选项: $1"
            show_help
            exit 1
            ;;
    esac
}

main "$@"
