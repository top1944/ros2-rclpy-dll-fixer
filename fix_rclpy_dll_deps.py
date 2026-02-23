#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS2 rclpy DLL依赖诊断和修复工具
用于检测和修复ROS2 Python导入失败的DLL依赖问题
"""

import os
import sys
import ctypes
import subprocess
from pathlib import Path

# Windows控制台UTF-8编码支持
if sys.platform == 'win32':
    import codecs
    sys.stdout = codecs.getwriter('utf-8')(sys.stdout.detach())
    sys.stderr = codecs.getwriter('utf-8')(sys.stderr.detach())

def check_dll_exists(dll_name, search_paths):
    """检查DLL是否存在"""
    for path in search_paths:
        dll_path = Path(path) / dll_name
        if dll_path.exists():
            return str(dll_path)
    return None

def get_ros2_bin_path():
    """获取ROS2 bin目录路径"""
    # 优先使用环境变量
    if 'ROS2_HOME' in os.environ:
        ros2_home = Path(os.environ['ROS2_HOME'])
        bin_dir = ros2_home / 'bin'
        if bin_dir.exists():
            return str(bin_dir)

    # 如果环境变量不存在，尝试常见路径
    possible_paths = [
        "C:/dev/ros2_humble/bin",
        "C:/ros2_humble/bin",
        "D:/dev/ros2_humble/bin",
        os.path.expanduser("~/ros2_humble/bin"),
    ]
    for path in possible_paths:
        if Path(path).exists():
            return path
    return None

def analyze_dll_dependencies(pyd_file):
    """分析.pyd文件的DLL依赖"""
    try:
        import pefile
        pe = pefile.PE(pyd_file)
        deps = []
        if hasattr(pe, 'DIRECTORY_ENTRY_IMPORT'):
            for entry in pe.DIRECTORY_ENTRY_IMPORT:
                deps.append(entry.dll.decode('utf-8'))
        return deps
    except Exception as e:
        print(f"分析DLL依赖时出错: {e}")
        return []

def main():
    print("=" * 60)
    print("ROS2 rclpy DLL依赖诊断工具")
    print("=" * 60)

    # 获取ROS2路径
    ros2_bin = get_ros2_bin_path()
    if not ros2_bin:
        print("\n❌ 错误: 找不到ROS2 bin目录")
        print("请确保ROS2已安装到以下位置之一:")
        print("  - C:/dev/ros2_humble")
        print("  - C:/ros2_humble")
        return False

    print(f"\n✓ ROS2 bin目录: {ros2_bin}")

    # 构建搜索路径
    search_paths = [
        ros2_bin,
        "C:/Windows/System32",
        "C:/Windows/SysWOW64",
    ]

    # 添加PATH中的路径
    if "PATH" in os.environ:
        search_paths.extend(os.environ["PATH"].split(os.pathsep))

    print("\n" + "=" * 60)
    print("步骤1: 检查关键DLL文件")
    print("=" * 60)

    # 关键DLL列表（基于之前发现的依赖）
    critical_dlls = {
        'spdlog.dll': 'spdlog日志库（最常见缺失）',
        'tinyxml2.dll': 'XML解析库',
        'ucrtbase.dll': 'Universal C Runtime',
        'api-ms-win-crt-runtime-l1-1-0.dll': 'CRT运行时',
        'fastrtps-2.6.dll': 'FastDDS通信中间件',
        'rcl.dll': 'ROS2客户端库核心',
        'rclpy.dll': 'ROS2 Python绑定库',
        'rcl_logging_spdlog.dll': 'ROS2日志支持',
    }

    missing_dlls = []
    found_dlls = []

    for dll_name, description in critical_dlls.items():
        dll_path = check_dll_exists(dll_name, search_paths)
        if dll_path:
            print(f"✓ {dll_name:<30} - {description}")
            found_dlls.append((dll_name, dll_path))
        else:
            print(f"✗ {dll_name:<30} - {description}")
            missing_dlls.append(dll_name)

    print("\n" + "=" * 60)
    print("步骤2: 测试rclpy导入")
    print("=" * 60)

    # 设置DLL搜索路径
    if ros2_bin:
        os.add_dll_directory(ros2_bin)

    # 添加到PATH
    if "PATH" in os.environ:
        os.environ["PATH"] = ros2_bin + os.pathsep + os.environ["PATH"]
    else:
        os.environ["PATH"] = ros2_bin

    try:
        import rclpy
        print("✓ rclpy导入成功！")
        return True
    except ImportError as e:
        print(f"✗ rclpy导入失败: {e}")

    print("\n" + "=" * 60)
    print("步骤3: 诊断报告")
    print("=" * 60)

    if missing_dlls:
        print(f"\n⚠️  发现 {len(missing_dlls)} 个缺失的DLL:")
        for dll in missing_dlls:
            print(f"  - {dll}")
        print("\n这些DLL需要被复制到ROS2 bin目录中")
    else:
        print("\n✓ 所有关键DLL都已存在")
        print("问题可能是其他原因（如Python版本不匹配）")

    print("\n" + "=" * 60)
    print("修复方案")
    print("=" * 60)

    if 'spdlog.dll' in missing_dlls:
        print("\n1. spdlog.dll缺失（最常见问题）:")
        print("   解决方法:")
        print("   a. 在ROS2目录中查找: dir /s %ROS2_HOME% spdlog.dll")
        print("   b. 如果在.pixi目录中找到，复制到bin目录:")
        print("      copy %ROS2_HOME%\\.pixi\\envs\\default\\Library\\bin\\spdlog.dll %ROS2_HOME%\\bin\\")
        print("   c. 或从在线下载: https://github.com/gabime/spdlog/releases")

    if 'tinyxml2.dll' in missing_dlls:
        print("\n2. tinyxml2.dll缺失:")
        print("   解决方法:")
        print("   a. 从系统安装的Conda/pixi环境查找")
        print("   b. 或从 https://github.com/leethomason/tinyxml2 下载")

    if 'ucrtbase.dll' in missing_dlls:
        print("\n3. UCRT DLL缺失:")
        print("   解决方法:")
        print("   a. 安装 Visual C++ Redistributable:")
        print("      https://learn.microsoft.com/en-us/cpp/windows/latest-supported-vc-redist")
        print("   b. 或从Windows System32复制相关DLL")

    print("\n" + "=" * 60)
    print("自动修复脚本生成")
    print("=" * 60)

    if missing_dlls:
        # 生成修复脚本
        script_path = Path(ros2_bin) / "auto_fix_dlls.bat"
        with open(script_path, 'w', encoding='utf-8') as f:
            f.write('@echo off\n')
            f.write('echo ROS2 DLL依赖自动修复脚本\n')
            f.write('echo ===========================\n\n')

            if 'spdlog.dll' in missing_dlls:
                f.write('echo 查找并复制spdlog.dll...\n')
                f.write('if exist "..\\.pixi\\envs\\default\\Library\\bin\\spdlog.dll" (\n')
                f.write('    copy /Y "..\\.pixi\\envs\\default\\Library\\bin\\spdlog.dll" .\n')
                f.write('    echo spdlog.dll已复制\n')
                f.write(') else (\n')
                f.write('    echo 警告: 未找到spdlog.dll，请手动下载并复制到此目录\n')
                f.write(')\n\n')

            f.write('echo.\n')
            f.write('echo 修复完成！请重新运行rclpy导入测试\n')
            f.write('pause\n')

        print(f"\n✓ 已生成自动修复脚本: {script_path}")
        print("  请运行此脚本来自动修复DLL问题")

    return False

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
