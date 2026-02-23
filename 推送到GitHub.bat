@echo off
chcp 65001 >nul
echo ============================================
echo 推送到 GitHub
echo ============================================
echo.

cd /d "%~dp0"

echo [1/3] 检查远程仓库...
git remote -v | findstr origin >nul
if %errorlevel% neq 0 (
    echo [INFO] 添加远程仓库...
    git remote add origin https://github.com/top1944/ros2-rclpy-dll-fixer.git
)

echo [OK] 远程仓库已配置

echo.
echo [2/3] 切换到 main 分支...
git branch -M main
echo [OK] 已切换到 main 分支

echo.
echo [3/3] 推送到 GitHub...
echo [INFO] 正在推送，请稍候...
echo.
git push -u origin main
if %errorlevel% neq 0 (
    echo.
    echo [ERROR] 推送失败
    echo.
    echo 可能的原因:
    echo   1. 网络连接问题 - 请稍后重试
    echo   2. GitHub认证失败 - 请运行: gh auth login
    echo   3. 仓库权限问题 - 请检查仓库设置
    echo.
    echo 手动推送命令:
    echo   cd /d "%~dp0"
    echo   git push -u origin main
    echo.
    pause
    exit /b 1
)

echo.
echo ============================================
echo 推送成功！
echo ============================================
echo.
echo 仓库地址:
echo   https://github.com/top1944/ros2-rclpy-dll-fixer
echo.
echo 访问仓库:
echo   start https://github.com/top1944/ros2-rclpy-dll-fixer
echo.
pause
