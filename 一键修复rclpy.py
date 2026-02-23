#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS2 rclpy DLL一键修复工具
自动检测并修复ROS2 Python导入失败的DLL依赖问题
"""

import os
import sys
import shutil
from pathlib import Path
from typing import List, Dict

class Color:
    """终端颜色"""
    OK = '\033[92m'      # 绿色
    WARNING = '\033[93m' # 黄色
    ERROR = '\033[91m'   # 红色
    INFO = '\033[94m'    # 蓝色
    END = '\033[0m'      # 结束

def print_ok(msg):
    print(f"{Color.OK}[OK]{Color.END} {msg}")

def print_warning(msg):
    print(f"{Color.WARNING}[!]{Color.END} {msg}")

def print_error(msg):
    print(f"{Color.ERROR}[错误]{Color.END} {msg}")

def print_info(msg):
    print(f"{Color.INFO}[信息]{Color.END} {msg}")

def find_ros2_bin() -> Path:
    """查找ROS2 bin目录"""
    # 优先使用环境变量
    if 'ROS2_HOME' in os.environ:
        ros2_home = Path(os.environ['ROS2_HOME'])
        bin_dir = ros2_home / 'bin'
        if bin_dir.exists():
            return bin_dir

    # 如果环境变量不存在，尝试常见路径
    possible_paths = [
        "C:/dev/ros2_humble/bin",
        "C:/ros2_humble/bin",
        str(Path.home() / "ros2_humble/bin"),
        "D:/dev/ros2_humble/bin",
    ]

    for path in possible_paths:
        if Path(path).exists():
            return Path(path)

    return None

def find_dll_in_ros2(ros2_dir: Path, dll_name: str) -> Path:
    """在ROS2目录中查找DLL"""
    # 常见搜索位置
    search_paths = [
        ros2_dir / ".pixi" / "envs" / "default" / "Library" / "bin",
        ros2_dir / "bin",
        ros2_dir / "lib",
    ]

    for search_path in search_paths:
        if search_path.exists():
            dll_path = search_path / dll_name
            if dll_path.exists():
                return dll_path

    # 递归搜索（较慢）
    print_info(f"在ROS2目录中递归搜索 {dll_name}...")
    for dll_path in ros2_dir.rglob(dll_name):
        if dll_path.is_file():
            return dll_path

    return None

def find_dll_in_backup(dll_name: str) -> Path:
    """在本地dll_backup文件夹中查找DLL"""
    # 获取脚本所在目录
    script_dir = Path(__file__).parent
    backup_dir = script_dir / "dll_backup"

    if backup_dir.exists():
        dll_path = backup_dir / dll_name
        if dll_path.exists():
            return dll_path

    return None

def copy_dll_to_bin(dll_source: Path, bin_dir: Path) -> bool:
    """复制DLL到bin目录"""
    try:
        target = bin_dir / dll_source.name
        shutil.copy2(dll_source, target)
        print_ok(f"已复制 {dll_source.name} 到 bin 目录")
        return True
    except Exception as e:
        print_error(f"复制 {dll_source.name} 失败: {e}")
        return False

def check_dll_status(bin_dir: Path) -> Dict[str, bool]:
    """检查关键DLL状态"""
    critical_dlls = [
        'spdlog.dll',            # 最常见缺失
        'tinyxml2.dll',
        'rcl.dll',
        'rclpy.dll',
        'rcl_logging_spdlog.dll',
        'fastrtps-2.6.dll',
    ]

    status = {}
    for dll_name in critical_dlls:
        dll_path = bin_dir / dll_name
        status[dll_name] = dll_path.exists()

    return status

def test_rclpy_import(bin_dir: Path) -> bool:
    """测试rclpy导入"""
    print_info("测试rclpy导入...")

    # 设置环境
    os.add_dll_directory(str(bin_dir))
    os.environ['PATH'] = str(bin_dir) + os.pathsep + os.environ.get('PATH', '')

    try:
        import rclpy
        print_ok("rclpy导入成功！")

        # 测试创建节点
        rclpy.init()
        node = rclpy.create_node('test_node')
        node.get_logger().info('ROS2节点运行正常！')
        node.destroy_node()
        rclpy.shutdown()

        print_ok("节点创建测试通过！")
        return True

    except ImportError as e:
        print_error(f"rclpy导入失败: {e}")
        return False
    except Exception as e:
        print_error(f"测试失败: {e}")
        return False

def create_test_node(bin_dir: Path) -> bool:
    """创建测试节点脚本"""
    test_script = bin_dir / "test_node_simple.py"

    # 使用环境变量获取ROS2_HOME
    ros2_home = os.environ.get('ROS2_HOME', 'C:\\dev\\ros2_humble')

    script_content = f'''#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS2简单测试节点"""

import os

# 使用环境变量ROS2_HOME
ros2_bin = os.path.expandvars(r'%ROS2_HOME%\\bin')
if not os.path.exists(ros2_bin):
    ros2_bin = r'{bin_dir}'

os.add_dll_directory(ros2_bin)
os.environ['PATH'] = ros2_bin + os.pathsep + os.environ.get('PATH', '')

import rclpy
from rclpy.node import Node

class SimpleNode(Node):
    def __init__(self):
        super().__init__('simple_node')
        self.get_logger().info('ROS2节点运行正常！')

def main():
    rclpy.init()
    node = SimpleNode()
    try:
        rclpy.spin_once(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
'''

    try:
        with open(test_script, 'w', encoding='utf-8') as f:
            f.write(script_content)
        print_ok(f"已创建测试节点: {test_script}")
        return True
    except Exception as e:
        print_error(f"创建测试节点失败: {e}")
        return False

def main():
    print("=" * 60)
    print("ROS2 rclpy DLL一键修复工具")
    print("=" * 60)
    print()

    # 步骤1: 查找ROS2
    print_info("步骤1: 查找ROS2安装目录...")
    bin_dir = find_ros2_bin()

    if not bin_dir:
        print_error("找不到ROS2目录！")
        print("请将ROS2安装到以下位置之一：")
        print("  - C:\\dev\\ros2_humble")
        print("  - C:\\ros2_humble")
        print("  - D:\\dev\\ros2_humble")
        return False

    print_ok(f"ROS2目录: {bin_dir.parent}")
    ros2_dir = bin_dir.parent

    # 检查DLL备份文件夹
    script_dir = Path(__file__).parent
    backup_dir = script_dir / "dll_backup"
    if backup_dir.exists():
        print_ok(f"DLL备份文件夹: {backup_dir}")
        dll_files = list(backup_dir.glob("*.dll"))
        if dll_files:
            print_info(f"  备份DLL数量: {len(dll_files)}")
    else:
        print_warning("未找到DLL备份文件夹: dll_backup/")
    print()

    # 步骤2: 检查DLL状态
    print_info("步骤2: 检查关键DLL状态...")
    dll_status = check_dll_status(bin_dir)

    missing_dlls = [dll for dll, exists in dll_status.items() if not exists]
    present_dlls = [dll for dll, exists in dll_status.items() if exists]

    for dll in present_dlls:
        print_ok(f"  {dll}")

    for dll in missing_dlls:
        print_warning(f"  {dll} (缺失)")
    print()

    # 步骤3: 修复缺失的DLL
    if missing_dlls:
        print_info("步骤3: 修复缺失的DLL...")

        for dll_name in missing_dlls:
            print_info(f"  查找 {dll_name}...")

            # 优先从本地备份查找
            dll_source = find_dll_in_backup(dll_name)

            if dll_source:
                print_info(f"    从本地备份找到: {dll_source}")
                copy_dll_to_bin(dll_source, bin_dir)
                continue

            # 从ROS2目录查找
            dll_source = find_dll_in_ros2(ros2_dir, dll_name)

            if dll_source:
                print_info(f"    从ROS2目录找到: {dll_source}")
                copy_dll_to_bin(dll_source, bin_dir)
            else:
                print_warning(f"    未找到 {dll_name}")

                # 提供下载提示
                print_info("    请从以下来源获取:")
                print("      1. 本地备份文件夹: dll_backup/")
                print("      2. pixi环境: ROS2_HOME\\.pixi\\envs\\default\\Library\\bin\\")
                if dll_name == 'spdlog.dll':
                    print("      3. 在线下载: https://github.com/gabime/spdlog/releases")
                elif dll_name == 'tinyxml2.dll':
                    print("      3. 在线下载: https://github.com/leethomason/tinyxml2")
                print(f"      然后复制到: {bin_dir}")
        print()

    # 步骤4: 测试导入
    print_info("步骤4: 测试rclpy导入...")
    import_success = test_rclpy_import(bin_dir)
    print()

    # 步骤5: 创建测试脚本
    if import_success:
        print_info("步骤5: 创建测试节点...")
        create_test_node(bin_dir)
        print()

    # 总结
    print("=" * 60)
    print("修复完成")
    print("=" * 60)

    if import_success:
        print_ok("✓ rclpy已可以正常使用！")
        print()
        print("下一步:")
        print(f"  cd {bin_dir}")
        print("  python test_node_simple.py")
    else:
        print_warning("⚠ rclpy仍然无法导入")
        print()
        print("请尝试以下方法:")
        print("  1. 检查Python版本（需要3.10）: python --version")
        print("  2. 重启命令行窗口")
        print("  3. 手动下载并复制缺失的DLL")
        print("  4. 参考: ROS2_rclpy_DLL修复教程.md")

    print()
    print("按回车键退出...")
    input()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n用户中断")
        sys.exit(1)
    except Exception as e:
        print_error(f"程序出错: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
