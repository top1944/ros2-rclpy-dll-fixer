# DLL备份文件夹说明

## 📁 文件夹作用

本文件夹包含ROS2运行所需的关键DLL备份文件。

当修复工具检测到ROS2安装目录中缺少这些DLL时，会自动从此文件夹复制到正确的位置。

---

## 📦 当前备份的DLL

| 文件名 | 大小 | 说明 |
|--------|------|------|
| spdlog.dll | 360KB | spdlog日志库（最常见缺失） |
| tinyxml2.dll | 90KB | XML解析库 |
| fastrtps-2.6.dll | 11MB | FastDDS通信中间件 |
| rcl.dll | 263KB | ROS2客户端库核心 |
| rcl_logging_spdlog.dll | 66KB | ROS2日志支持库 |

---

## 🔧 使用方式

### 自动复制

运行修复工具时，脚本会自动检查并复制缺失的DLL：

```batch
# 批处理工具
fix_rclpy_dll_simple.bat

# 或Python工具
python 一键修复rclpy.py
```

### 手动复制

如果需要手动复制：

```batch
copy dll_backup\spdlog.dll %ROS2_HOME%\bin\
copy dll_backup\tinyxml2.dll %ROS2_HOME%\bin\
copy dll_backup\fastrtps-2.6.dll %ROS2_HOME%\bin\
copy dll_backup\rcl.dll %ROS2_HOME%\bin\
copy dll_backup\rcl_logging_spdlog.dll %ROS2_HOME%\bin\
```

---

## ⚠️ 注意事项

1. **DLL来源**：这些DLL从标准ROS2 Humble安装中提取
2. **版本兼容**：仅适用于ROS2 Humble (Python 3.10)
3. **位数要求**：仅支持64位Windows系统
4. **系统DLL**：UCRT等系统DLL未包含，需通过安装Visual C++ Redistributable获取

---

## 🔄 更新DLL

如果您的ROS2版本不同，可以替换这些DLL：

1. 从您的ROS2安装目录复制对应的DLL
2. 替换本文件夹中的文件
3. 确保文件名匹配

---

## 📚 相关文档

- **使用前必读.md** - 环境变量设置
- **快速修复指南.md** - 快速入门
- **ROS2_rclpy_DLL修复教程.md** - 详细教程

---

**更新日期**: 2026-02-23
**适用版本**: ROS2 Humble + Python 3.10 + Windows 10/11 (64位)
