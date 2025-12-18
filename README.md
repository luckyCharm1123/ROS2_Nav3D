# ROS2 Nav3D - 3D路径规划与导航系统

<div align="center">

![ROS2](https://img.shields.io/badge/ROS2-Humble-blue?logo=ros&logoColor=white)
![C++](https://img.shields.io/badge/C++-17-blue?logo=c%2B%2B)
![License](https://img.shields.io/badge/License-Apache%202.0-green)
![Build Status](https://img.shields.io/badge/build-passing-brightgreen)

**高性能3D路径规划框架，支持A*和JPS算法**

[功能特性](#功能特性) • [快速开始](#快速开始) • [部署指南](#部署指南) • [使用说明](#使用说明) • [API文档](#api文档) • [贡献指南](#贡献指南)

</div>

---

## 📖 项目简介

ROS2 Nav3D是一个功能强大的3D路径规划与导航系统，专为复杂三维环境下的机器人导航而设计。该项目集成了多种先进的路径搜索算法，包括A*算法和Jump Point Search (JPS)算法，并提供了直观的RViz可视化工具。

### 🎯 核心功能
- **3D路径规划**: 在三维空间中进行高效路径搜索
- **多算法支持**: A*算法和JPS算法对比分析
- **实时可视化**: 基于RViz的实时路径和障碍物可视化
- **随机地图生成**: 自动生成复杂3D测试环境
- **3D导航目标**: RViz插件支持6自由度目标设置
- **路径点生成器**: 预设轨迹模式生成

---

## 🚀 功能特性

### 🔍 路径搜索算法
#### A*搜索器 (`grid_path_searcher`)
- **三维栅格地图**: 支持可配置分辨率的3D栅格化
- **启发式搜索**: 采用欧几里得距离作为启发函数
- **实时避障**: 动态处理障碍物和碰撞检测
- **路径优化**: 自动生成平滑的三维路径

#### JPS搜索器 (`grid_path_searcher`)
- **跳跃点优化**: 通过跳过冗余节点大幅提升搜索效率
- **强制邻居检测**: 智能识别路径转折点
- **性能对比**: 与A*算法进行实时性能对比

### 🎨 可视化工具
#### RViz 3D导航目标插件 (`rviz_3d_nav_goal_tool`)
- **6自由度目标**: 支持位置(x,y,z)和姿态(roll,pitch,yaw)设置
- **实时交互**: 通过RViz界面直接设置导航目标
- **ROS2集成**: 原生支持ROS2消息发布

#### 路径点生成器 (`waypoint_generator`)
- **预设轨迹**: 支持圆形、8字形等预设轨迹
- **序列模式**: 分段路径生成与执行
- **手动模式**: 交互式路径点构建

### 🗺️ 地图生成器
- **复杂障碍物**: 随机生成球形、柱形等3D障碍物
- **地形模拟**: 支持斜坡和不平整地面
- **参数化配置**: 可调节障碍物数量、尺寸、分布

---

## 🏁 快速开始

### 系统要求

| 依赖项 | 版本要求 | 说明 |
|--------|----------|------|
| Ubuntu | 20.04+ | 推荐22.04 |
| ROS2 | Humble | 其他版本需适配 |
| CMake | 3.16+ | 构建系统 |
| GCC | 9.0+ | C++17支持 |
| Eigen3 | 3.3+ | 线性代数计算 |
| PCL | 1.10+ | 点云处理 |

### 环境检查

```bash
# 检查ROS2版本
ros2 --version

# 检查工作空间
echo $ROS2_WORKSPACE
```

---

## 📦 部署指南

### 步骤1: 环境准备

```bash
# 1.1 安装ROS2 Humble (如果未安装)
sudo apt update && sudo apt install -y curl gnupg2 lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install -y ros-humble-desktop

# 1.2 安装依赖包
sudo apt install -y \
    ros-humble-rviz2 \
    ros-humble-pcl-ros \
    ros-humble-tf2-geometry-msgs \
    libeigen3-dev \
    libpcl-dev \
    qtbase5-dev \
    libqt5opengl5-dev

# 1.3 配置ROS2环境
source /opt/ros/humble/setup.bash
```

### 步骤2: 获取源码

```bash
# 2.1 创建工作空间
mkdir -p ~/nav3d_ws/src
cd ~/nav3d_ws/src

# 2.2 克隆仓库
git clone https://github.com/luckyCharm1123/ROS2_Nav3D.git nav3d
cd nav3d

# 2.3 检查目录结构
ls -la
# 应该看到: src/ README.md .gitignore
```

### 步骤3: 构建项目

```bash
# 3.1 返回工作空间根目录
cd ~/nav3d_ws

# 3.2 安装依赖 (如果需要)
rosdep install --from-paths src --ignore-src -r -y

# 3.3 构建所有包
colcon build --symlink-install

# 3.4 构建完成后验证
ls install/
# 应该看到: lib/ share/ etc/ ...
```

### 步骤4: 环境配置

```bash
# 4.1 添加环境变量到bashrc
echo "source ~/nav3d_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc

# 4.2 验证包是否正确安装
ros2 pkg list | grep grid_path_searcher
ros2 pkg list | grep rviz_3d_nav_goal_tool
ros2 pkg list | grep waypoint_generator
```

### 步骤5: 验证安装

```bash
# 5.1 检查可执行文件
ls install/grid_path_searcher/lib/grid_path_searcher/
# 应该看到: demo_node random_complex_generator

# 5.2 检查RViz插件
ls install/rviz_3d_nav_goal_tool/lib/rviz_3d_nav_goal_tool/
# 应该看到: librviz_3d_nav_goal_tool.so
```

---

## 🎮 使用说明

### 场景1: 完整演示 (推荐新手)

```bash
# 终端1: 启动完整演示
ros2 launch grid_path_searcher demo.launch.py

# 参数自定义 (可选)
ros2 launch grid_path_searcher demo.launch.py \
  map_size_x:=20.0 \
  map_size_y:=20.0 \
  start_x:=5.0
```

**操作流程:**
1. RViz会自动启动并显示3D网格
2. 等待随机地图生成完成 (约2-3秒)
3. 在RViz中使用 **"3D Nav Goal"** 工具
4. 点击地图任意位置设置目标点
5. 观察A* (绿色) 和JPS (红色) 路径对比
6. 查看访问节点 (蓝色点云) 可视化

### 场景2: 独立组件使用

#### 2.1 路径搜索器 (无地图生成)

```bash
# 终端1: 启动路径搜索器
ros2 run grid_path_searcher demo_node

# 终端2: 发送随机地图 (可选)
ros2 run grid_path_searcher random_complex_generator

# 终端3: 发送目标点
ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped "header:
  frame_id: 'world'
pose:
  position:
    x: 10.0
    y: 5.0
    z: 2.0
  orientation:
    w: 1.0"
```

#### 2.2 路径点生成器

```bash
# 终端1: 启动路径点生成器
ros2 run waypoint_generator waypoint_generator --ros-args -p waypoint_type:=circle

# 终端2: 发送触发信号
ros2 topic pub /goal geometry_msgs/msg/PoseStamped "header:
  frame_id: 'world'
pose:
  position:
    x: 0.0
    y: 0.0
    z: 1.0
  orientation:
    w: 1.0"
```

**支持的轨迹模式:**
- `circle`: 圆形轨迹
- `eight`: 8字形轨迹  
- `point`: 单点轨迹
- `series`: 分段序列
- `manual`: 手动添加点
- `noyaw`: 无偏航手动模式

#### 2.3 RViz 3D导航插件

```bash
# 启动RViz并加载插件
ros2 run rviz2 rviz2

# 在RViz中:
# 1. 点击 "Panels" -> "Tools"
# 2. 在工具列表中选择 "3D Nav Goal"
# 3. 在3D视图中点击并拖动设置目标
```

### 场景3: 自定义地图生成

```bash
# 生成自定义复杂地图
ros2 run grid_path_searcher random_complex_generator --ros-args \
  -p map.x_size:=30.0 \
  -p map.y_size:=30.0 \
  -p map.z_size:=10.0 \
  -p map.circle_num:=50 \
  -p map.obs_num:=500 \
  -p map.resolution:=0.15
```

---

## 🔧 配置参数

### grid_path_searcher 参数

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `map.resolution` | double | 0.2 | 栅格分辨率(m) |
| `map.x_size` | double | 50.0 | X轴尺寸(m) |
| `map.y_size` | double | 50.0 | Y轴尺寸(m) |
| `map.z_size` | double | 5.0 | Z轴尺寸(m) |
| `map.cloud_margin` | double | 0.0 | 云边缘距离 |
| `planning.start_x` | double | 0.0 | 起点X坐标 |
| `planning.start_y` | double | 0.0 | 起点Y坐标 |
| `planning.start_z` | double | 0.0 | 起点Z坐标 |

### random_complex_generator 参数

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `map.circle_num` | int | 40 | 圆形障碍物数量 |
| `map.obs_num` | int | 300 | 随机障碍物数量 |
| `map.generate_ground` | int | 1 | 是否生成地面 |
| `map.max_ground_slope` | double | 0.3 | 最大地面坡度 |
| `ObstacleShape.lower_rad` | double | 0.1 | 最小障碍物半径 |
| `ObstacleShape.upper_rad` | double | 0.7 | 最大障碍物半径 |
| `sensing.rate` | double | 0.5 | 发布频率(Hz) |

### waypoint_generator 参数

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `waypoint_type` | string | "manual" | 轨迹生成模式 |
| `segment_cnt` | int | 0 | 分段数量(序列模式) |

---

## 📊 性能对比

### 算法性能 (测试环境: 50×50×5m, 分辨率0.2m)

| 算法 | 搜索时间 | 路径长度 | 访问节点 | 适用场景 |
|------|----------|----------|----------|----------|
| **A*** | 15-25ms | 1.0× | 800-1200 | 精确路径 |
| **JPS** | 8-12ms | 1.05× | 300-500 | 快速规划 |

**结论**: JPS在保持路径质量的同时，搜索速度提升约50%，访问节点减少60%。

---

## 📁 项目结构

```
nav3D/
├── src/
│   ├── grid_path_searcher/          # 路径搜索核心包
│   │   ├── include/                 # 头文件
│   │   │   ├── Astar_searcher.hpp  # A*算法实现
│   │   │   ├── JPS_searcher.h      # JPS算法实现
│   │   │   └── node.h              # 节点定义
│   │   ├── src/                    # 源文件
│   │   │   ├── Astar_searcher.cpp
│   │   │   ├── JPS_searcher.cpp
│   │   │   ├── demo_node.cpp       # 主节点
│   │   │   └── random_complex_generator.cpp
│   │   ├── launch/                 # 启动文件
│   │   │   ├── demo.launch.py
│   │   │   └── demo.rviz
│   │   └── CMakeLists.txt
│   │
│   ├── rviz_3d_nav_goal_tool/      # RViz插件
│   │   ├── include/
│   │   │   └── rviz_3d_nav_goal_tool/
│   │   │       ├── goal_tool.hpp   # 3D目标工具
│   │   │       └── pose_tool.hpp   # 基础工具
│   │   ├── src/
│   │   │   ├── goal_tool.cpp
│   │   │   └── pose_tool.cpp
│   │   └── rviz_plugin.xml
│   │
│   └── waypoint_generator/         # 路径点生成器
│       ├── src/
│       │   ├── waypoint_generator.cpp
│       │   └── sample_waypoints.hpp
│       └── CMakeLists.txt
│
├── .gitignore
└── README.md
```

---

## 🔍 API文档

### 核心类参考

#### AstarPathFinder
```cpp
class AstarPathFinder {
    // 初始化栅格地图
    void initGridMap(double resolution, Eigen::Vector3d global_xyz_l, 
                     Eigen::Vector3d global_xyz_u, int max_x_id, int max_y_id, int max_z_id);
    
    // 设置障碍物
    void setObs(const double coord_x, const double coord_y, const double coord_z);
    
    // A*图搜索
    void AstarGraphSearch(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt);
    
    // 获取路径
    std::vector<Eigen::Vector3d> getPath();
    
    // 获取访问节点
    std::vector<Eigen::Vector3d> getVisitedNodes();
    
    // 重置栅格
    void resetUsedGrids();
};
```

#### JPSPathFinder (继承自AstarPathFinder)
```cpp
class JPSPathFinder : public AstarPathFinder {
    // JPS图搜索
    void JPSGraphSearch(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt);
    
    // 跳跃点检测
    bool jump(const Eigen::Vector3i &curIdx, const Eigen::Vector3i &expDir, 
              Eigen::Vector3i &neiIdx);
};
```

#### WaypointGenerator
```cpp
class WaypointGenerator : public rclcpp::Node {
    // 生成圆形轨迹
    nav_msgs::msg::Path circle();
    
    // 生成8字形轨迹
    nav_msgs::msg::Path eight();
    
    // 加载分段轨迹
    void load_waypoints(const rclcpp::Time& time_base);
};
```

---

## 🤝 贡献指南

### 开发环境设置

```bash
# 1. 克隆项目
git clone https://github.com/luckyCharm1123/ROS2_Nav3D.git
cd ROS2_Nav3D

# 2. 安装开发依赖
sudo apt install -y clang-format cppcheck

# 3. 代码格式化
find src/ -name "*.cpp" -o -name "*.hpp" | xargs clang-format -i

# 4. 静态检查
cppcheck --enable=all --project=compile_commands.json
```

### 提交规范

```bash
# 功能开发
git add .
git commit -m "feat: 添加JPS跳跃点优化算法"

# Bug修复
git commit -m "fix: 修复A*路径为空的边界情况"

# 文档更新
git commit -m "docs: 更新部署指南"
```

---

## 🐛 常见问题

### Q1: 编译时找不到依赖包
```bash
# 解决方案
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### Q2: RViz插件未加载
```bash
# 检查插件路径
echo $LD_LIBRARY_PATH
# 确保包含: ~/nav3d_ws/install/rviz_3d_nav_goal_tool/lib

# 重启RViz
killall rviz2 && ros2 run rviz2 rviz2
```

### Q3: 路径搜索超时
```bash
# 增大地图分辨率
ros2 launch grid_path_searcher demo.launch.py map_resolution:=0.5

# 或减小地图尺寸
ros2 launch grid_path_searcher demo.launch.py map_size_x:=20.0
```

---

## 📄 许可证

本项目采用 **Apache 2.0** 许可证，详见 [LICENSE](LICENSE) 文件。

```
Copyright 2025 ROS2 Nav3D Contributors

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
```

---

## 🙏 致谢

- **ROS2社区**: 提供强大的机器人开发框架
- **PCL团队**: 优秀的点云处理库
- **Eigen团队**: 高性能线性代数计算
- **RViz开发团队**: 强大的可视化工具

---

## 📞 联系方式

- **作者**: Bruce Chan Jian Le / ubuntu22
- **邮箱**: jianle001@e.ntu.edu.sg / 599623904@qq.com
- **GitHub**: [@luckyCharm1123](https://github.com/luckyCharm1123)

---

## 🌟 Star History

[![Star History Chart](https://api.star-history.com/svg?repos=luckyCharm1123/ROS2_Nav3D&type=Date)](https://star-history.com/#luckyCharm1123/ROS2_Nav3D&Date)

---

<div align="center">

**如果这个项目对您有帮助，请给我们一个 ⭐️ Star！**

Made with ❤️ by the ROS2 Nav3D Team

</div>
