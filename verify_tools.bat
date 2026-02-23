@echo off
chcp 65001 >nul
setlocal enabledelayedexpansion

echo ============================================
echo ROS2 DLL修复工具验证脚本
echo ============================================
echo.

REM 检查ROS2_HOME环境变量
echo [1/5] 检查ROS2_HOME环境变量...
if defined ROS2_HOME (
    echo [OK] ROS2_HOME 已设置: %ROS2_HOME%
    set "ROS2_BIN=%ROS2_HOME%\bin"
) else (
    echo [警告] ROS2_HOME 未设置
    echo      请运行: set ROS2_HOME=C:\dev\ros2_humble
    set "ROS2_BIN=C:\dev\ros2_humble\bin"
)

echo.
echo [2/5] 检查ROS2目录...
if exist "%ROS2_BIN%" (
    echo [OK] ROS2 bin目录存在: %ROS2_BIN%
) else (
    echo [错误] ROS2 bin目录不存在: %ROS2_BIN%
    pause
    exit /b 1
)

echo.
echo [3/5] 检查关键DLL文件...
cd /d "%ROS2_BIN%"

set "DLL_COUNT=0"
set "MISSING_DLLS="

for %%f in (spdlog.dll tinyxml2.dll rcl.dll rcl_logging_spdlog.dll) do (
    if exist "%%f" (
        echo [OK] %%f
        set /a DLL_COUNT+=1
    ) else (
        echo [!] %%f 缺失
        set "MISSING_DLLS=!MISSING_DLLS! %%f"
    )
)

echo.
echo [4/5] 检查修复工具...
cd /d "%~dp0"

if exist "fix_rclpy_dll_simple.bat" (
    echo [OK] fix_rclpy_dll_simple.bat 存在
) else (
    echo [错误] fix_rclpy_dll_simple.bat 不存在
)

if exist "一键修复rclpy.py" (
    echo [OK] 一键修复rclpy.py 存在
) else (
    echo [错误] 一键修复rclpy.py 不存在
)

if exist "fix_rclpy_dll_deps.py" (
    echo [OK] fix_rclpy_dll_deps.py 存在
) else (
    echo [错误] fix_rclpy_dll_deps.py 不存在
)

echo.
echo [5/5] 检查Python...
python --version >nul 2>&1
if %ERRORLEVEL% EQU 0 (
    echo [OK] Python 可用
    for /f "tokens=2" %%i in ('python --version 2^>^&1') do set PY_VERSION=%%i
    echo      版本: %PY_VERSION%
) else (
    echo [警告] Python 不可用或不在PATH中
)

echo.
echo ============================================
echo 验证完成
echo ============================================
echo.

if %DLL_COUNT% EQU 4 (
    echo [成功] 所有关键DLL都已存在
    echo        ROS2环境配置正常！
) else (
    echo [提示] 有 4-%DLL_COUNT% 个DLL缺失
    echo        请运行: fix_rclpy_dll_simple.bat
)

echo.
echo 按任意键退出...
pause >nul
