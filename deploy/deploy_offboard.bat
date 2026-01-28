@echo off
REM ============================================================================
REM  deploy_offboard.bat - Windows批处理版一键部署脚本
REM ============================================================================
REM  功能说明：
REM  - 调用deploy_offboard.sh脚本进行部署
REM  - 支持在Windows PowerShell或Git Bash中运行
REM  - 自动检测环境并执行
REM  
REM  使用方法：
REM  deploy\deploy_offboard.bat [选项]
REM  
REM  选项：
REM  -h          显示帮助信息
REM  -d MMDD     指定日期
REM  -m MSG      指定git提交信息
REM  -n          跳过git提交
REM  -s          仅提交，不推送
REM
REM  @author [Your Name]
REM  @date 2026-01-28
REM ============================================================================

setlocal enabledelayedexpansion

REM 设置编码为UTF-8
chcp 65001 >nul 2>&1

REM 获取脚本所在目录
set "SCRIPT_DIR=%~dp0"
set "BASH_SCRIPT=%SCRIPT_DIR%deploy_offboard.sh"

REM 颜色定义
set "INFO=[INFO]"
set "WARN=[WARN]"
set "ERROR=[ERROR]"

echo.
echo ===============================================================
echo   无人机离板控制项目一键部署脚本 (Windows版)
echo ===============================================================
echo.

REM 检查bash脚本是否存在
if not exist "%BASH_SCRIPT%" (
    echo %ERROR% 找不到 deploy_offboard.sh 脚本
    echo %ERROR% 请确保脚本与此批处理文件在同一目录
    pause
    exit /b 1
)

REM 检查git安装
where git >nul 2>&1
if errorlevel 1 (
    echo %ERROR% 未找到git，请先安装git
    echo %ERROR% 下载地址: https://git-scm.com/download/win
    pause
    exit /b 1
)

REM 检查ssh安装
where ssh >nul 2>&1
if errorlevel 1 (
    echo %ERROR% 未找到ssh
    echo %INFO% 如果安装了Git，请使用Git Bash或WSL运行脚本
    echo %INFO% 或者安装OpenSSH: https://docs.microsoft.com/en-us/windows-server/administration/openssh/openssh_install_configure
    pause
    exit /b 1
)

REM 尝试使用bash运行脚本
echo %INFO% 检测到bash环境，准备执行部署脚本...
echo %INFO% 脚本位置: %BASH_SCRIPT%
echo.

bash.exe "%BASH_SCRIPT%" %*

if errorlevel 1 (
    echo.
    echo %ERROR% 脚本执行失败，请检查错误信息
    pause
    exit /b 1
)

echo.
echo ===============================================================
echo   部署完成！
echo ===============================================================
echo.
pause

exit /b 0
