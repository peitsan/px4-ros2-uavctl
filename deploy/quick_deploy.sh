#!/bin/bash
################################################################################
# quick_deploy.sh - 快速部署脚本（预置常用场景）
#
# 说明：
#   快捷脚本，预置了常用的部署场景，用户可直接运行
#
# 使用方法：
#   ./deploy/quick_deploy.sh [场景]
#
# 场景列表：
#   simple     - 简单部署（仅推送代码，不编译）
#   build      - 编译部署（推送+编译）
#   full       - 完整部署（推送+编译+部署）
#   test       - 测试部署（干运行，仅显示操作）
#
# @date 2026-01-28
################################################################################

set -e

# 颜色定义
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m'

# 获取脚本目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

show_menu() {
    echo ""
    echo -e "${BLUE}===============================================================${NC}"
    echo -e "${BLUE}  快速部署工具${NC}"
    echo -e "${BLUE}===============================================================${NC}"
    echo ""
    echo "可用场景："
    echo "  1. simple  - 简单推送（仅同步代码）"
    echo "  2. build   - 编译推送（同步+编译）"
    echo "  3. full    - 完整部署（同步+编译+运行）"
    echo "  4. test    - 测试部署（干运行模式）"
    echo "  5. status  - 检查状态"
    echo "  0. 退出"
    echo ""
}

# 简单部署 - 仅推送代码
do_simple_deploy() {
    echo -e "${GREEN}开始简单部署（仅推送代码）...${NC}"
    bash "${SCRIPT_DIR}/deploy_offboard.sh" -n
}

# 编译部署 - 推送并编译
do_build_deploy() {
    echo -e "${GREEN}开始编译部署（推送+编译）...${NC}"
    bash "${SCRIPT_DIR}/deploy_offboard.sh"
}

# 完整部署 - 推送、编译、部署
do_full_deploy() {
    echo -e "${GREEN}开始完整部署（推送+编译+部署）...${NC}"
    bash "${SCRIPT_DIR}/deploy_offboard.sh"
}

# 测试部署 - 干运行模式
do_test_deploy() {
    echo -e "${YELLOW}开始测试部署（干运行模式，仅显示操作）...${NC}"
    echo "此模式下会显示所有操作但不执行任何更改"
    bash "${SCRIPT_DIR}/deploy_offboard.sh" -s
}

# 检查部署状态
do_check_status() {
    echo -e "${GREEN}检查部署状态...${NC}"
    echo ""
    
    echo "1. 检查Git状态"
    cd "$(dirname "${SCRIPT_DIR}")"
    git status
    
    echo ""
    echo "2. 检查网络连接"
    ping -c 1 192.168.3.17 > /dev/null 2>&1 && echo "✓ 香橙派网络连接正常" || echo "✗ 香橙派网络连接失败"
    
    echo ""
    echo "3. 检查SSH连接"
    ssh -o ConnectTimeout=5 -o BatchMode=yes orangepi@192.168.3.17 "echo '✓ SSH连接正常'" 2>/dev/null || \
        echo "✗ SSH连接失败，请检查凭证或SSH配置"
}

# 主菜单循环
main() {
    while true; do
        show_menu
        read -p "请选择操作 (0-5): " choice
        
        case $choice in
            1)
                do_simple_deploy
                ;;
            2)
                do_build_deploy
                ;;
            3)
                do_full_deploy
                ;;
            4)
                do_test_deploy
                ;;
            5)
                do_check_status
                ;;
            0)
                echo -e "${GREEN}退出部署工具${NC}"
                exit 0
                ;;
            *)
                echo -e "${YELLOW}无效选择，请重试${NC}"
                ;;
        esac
        
        echo ""
        read -p "按Enter继续..." dummy
    done
}

# 如果提供了参数，直接执行对应场景
if [ $# -gt 0 ]; then
    case $1 in
        simple)
            do_simple_deploy
            ;;
        build)
            do_build_deploy
            ;;
        full)
            do_full_deploy
            ;;
        test)
            do_test_deploy
            ;;
        status)
            do_check_status
            ;;
        *)
            echo "未知场景: $1"
            echo "可用场景: simple, build, full, test, status"
            exit 1
            ;;
    esac
else
    # 交互模式
    main
fi
