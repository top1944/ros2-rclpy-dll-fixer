@echo off
chcp 65001 >nul
setlocal enabledelayedexpansion

echo ============================================
echo ROS2 rclpy DLL依赖快速修复工具
echo ============================================
echo.

REM 获取脚本所在目录
set "SCRIPT_DIR=%~dp0"
set "DLL_BACKUP=%SCRIPT_DIR%dll_backup"

set "ROS2_BIN=%ROS2_HOME%\bin"

if not exist "%ROS2_BIN%" (
    echo [错误] 找不到ROS2目录: %ROS2_BIN%
    echo 请先设置ROS2_HOME环境变量
    echo 例如: set ROS2_HOME=C:\dev\ros2_humble
    pause
    exit /b 1
)

echo [1/6] 设置环境变量...
set "PATH=%ROS2_BIN%;%PATH%"
set "PYTHONPATH=%ROS2_HOME%\Lib\site-packages;%PYTHONPATH%"

echo [2/6] 检查DLL备份文件夹...
if exist "%DLL_BACKUP%" (
    echo [OK] 找到DLL备份文件夹: %DLL_BACKUP%
) else (
    echo [警告] DLL备份文件夹不存在: %DLL_BACKUP%
    echo      将无法从本地复制DLL
)

echo.
echo [3/6] 检查并复制关键DLL...
cd /d "%ROS2_BIN%"

:: 检查spdlog.dll (最常见的问题)
if not exist "spdlog.dll" (
    echo [!] spdlog.dll 缺失，正在查找...
    if exist "%DLL_BACKUP%\spdlog.dll" (
        echo [+] 从本地备份复制spdlog.dll
        copy /Y "%DLL_BACKUP%\spdlog.dll" "%ROS2_BIN%\" >nul
        if exist "spdlog.dll" (
            echo [OK] spdlog.dll 已复制
        ) else (
            echo [错误] spdlog.dll 复制失败
        )
    ) else if exist "..\.pixi\envs\default\Library\bin\spdlog.dll" (
        echo [+] 从pixi环境复制spdlog.dll
        copy /Y "..\.pixi\envs\default\Library\bin\spdlog.dll" "%ROS2_BIN%\" >nul
        if exist "spdlog.dll" (
            echo [OK] spdlog.dll 已复制
        ) else (
            echo [错误] spdlog.dll 复制失败
        )
    ) else (
        echo [-] 未找到spdlog.dll
        echo     请从以下位置之一获取:
        echo     1. 本地备份: %DLL_BACKUP%
        echo     2. pixi环境: ROS2_HOME\.pixi\envs\default\Library\bin\
        echo     3. 在线下载: https://github.com/gabime/spdlog/releases
    )
) else (
    echo [OK] spdlog.dll 已存在
)

:: 检查tinyxml2.dll
if not exist "tinyxml2.dll" (
    echo [!] tinyxml2.dll 缺失
    if exist "%DLL_BACKUP%\tinyxml2.dll" (
        echo [+] 从本地备份复制tinyxml2.dll
        copy /Y "%DLL_BACKUP%\tinyxml2.dll" "%ROS2_BIN%\" >nul
        if exist "tinyxml2.dll" (
            echo [OK] tinyxml2.dll 已复制
        )
    ) else if exist "..\.pixi\envs\default\Library\bin\tinyxml2.dll" (
        echo [+] 从pixi环境复制tinyxml2.dll
        copy /Y "..\.pixi\envs\default\Library\bin\tinyxml2.dll" "%ROS2_BIN%\" >nul
    )
) else (
    echo [OK] tinyxml2.dll 已存在
)

:: 检查rcl_logging_spdlog.dll
if not exist "rcl_logging_spdlog.dll" (
    echo [!] rcl_logging_spdlog.dll 缺失
    if exist "%DLL_BACKUP%\rcl_logging_spdlog.dll" (
        echo [+] 从本地备份复制rcl_logging_spdlog.dll
        copy /Y "%DLL_BACKUP%\rcl_logging_spdlog.dll" "%ROS2_BIN%\" >nul
        if exist "rcl_logging_spdlog.dll" (
            echo [OK] rcl_logging_spdlog.dll 已复制
        )
    )
) else (
    echo [OK] rcl_logging_spdlog.dll 已存在
)

:: 检查rcl.dll
if not exist "rcl.dll" (
    echo [!] rcl.dll 缺失
    if exist "%DLL_BACKUP%\rcl.dll" (
        echo [+] 从本地备份复制rcl.dll
        copy /Y "%DLL_BACKUP%\rcl.dll" "%ROS2_BIN%\" >nul
        if exist "rcl.dll" (
            echo [OK] rcl.dll 已复制
        )
    )
) else (
    echo [OK] rcl.dll 已存在
)

:: 检查fastrtps-2.6.dll
if not exist "fastrtps-2.6.dll" (
    echo [!] fastrtps-2.6.dll 缺失
    if exist "%DLL_BACKUP%\fastrtps-2.6.dll" (
        echo [+] 从本地备份复制fastrtps-2.6.dll
        copy /Y "%DLL_BACKUP%\fastrtps-2.6.dll" "%ROS2_BIN%\" >nul
        if exist "fastrtps-2.6.dll" (
            echo [OK] fastrtps-2.6.dll 已复制
        )
    )
) else (
    echo [OK] fastrtps-2.6.dll 已存在
)

:: 检查UCRT DLLs
set "UCRT_COUNT=0"
for %%f in (api-ms-win-crt-*.dll ucrtbase.dll) do (
    if not exist "%%f" (
        set /a UCRT_COUNT+=1
    )
)

if %UCRT_COUNT% GTR 0 (
    echo [!] 缺少 %UCRT_COUNT% 个UCRT DLL
    echo     请安装 Visual C++ Redistributable
    echo     下载地址: https://aka.ms/vs/17/release/vc_redist.x64.exe
) else (
    echo [OK] UCRT DLLs 已存在
)

echo.
echo [4/6] 检查Python版本...
python --version >nul 2>&1
if %ERRORLEVEL% NEQ 0 (
    echo [警告] Python不在PATH中
    echo 请确保Python 3.10已安装并添加到PATH
) else (
    for /f "tokens=2" %%i in ('python --version 2^>^&1') do echo [OK] Python版本: %%i
)

echo.
echo [5/6] 测试rclpy导入...
python -c "import rclpy; print('[OK] rclpy导入成功!')" 2>nul
if %ERRORLEVEL% NEQ 0 (
    echo [!] rclpy导入失败
    echo.
    echo [诊断] 详细错误信息:
    python -c "import rclpy" 2>&1
    echo.
) else (
    echo.
    echo [6/6] 创建完整的测试节点...
    if not exist "test_node_simple.py" (
        echo #!/usr/bin/env python3 > test_node_simple.py
        echo # -*- coding: utf-8 -*- >> test_node_simple.py
        echo. >> test_node_simple.py
        echo import os >> test_node_simple.py
        echo ros2_bin = os.path.expandvars(r'%%ROS2_HOME%%\bin') >> test_node_simple.py
        echo os.add_dll_directory(ros2_bin) >> test_node_simple.py
        echo os.environ['PATH'] = ros2_bin + os.pathsep + os.environ.get('PATH', '') >> test_node_simple.py
        echo. >> test_node_simple.py
        echo import rclpy >> test_node_simple.py
        echo from rclpy.node import Node >> test_node_simple.py
        echo. >> test_node_simple.py
        echo rclpy.init^(^) >> test_node_simple.py
        echo node = Node^('test_node'^) >> test_node_simple.py
        echo node.get_logger^(^).info^('ROS2节点运行正常!'^) >> test_node_simple.py
        echo rclpy.shutdown^(^) >> test_node_simple.py
        echo. >> test_node_simple.py
        echo [OK] 已创建test_node_simple.py
    )

    echo [完成] 修复成功！
    echo.
    echo 运行测试节点: python test_node_simple.py
)

echo.
echo ============================================
echo 修复完成
echo ============================================
echo.
echo 如果rclpy仍然导入失败，请尝试以下步骤:
echo 1. 重启命令行窗口
echo 2. 运行: set ROS2_HOME=您的ROS2路径
echo 3. 运行: set PATH=%%ROS2_HOME%%\bin;%%PATH%%
echo 4. 运行: python -c "import rclpy"
echo 5. 检查DLL备份文件夹: %DLL_BACKUP%
echo.
echo DLL备份清单:
if exist "%DLL_BACKUP%" (
    dir "%DLL_BACKUP%" /b
) else (
    echo (备份文件夹不存在)
)
echo.
pause
