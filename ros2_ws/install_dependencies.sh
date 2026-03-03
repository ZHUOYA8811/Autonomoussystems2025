#!/bin/bash

# ROS2探索系统依赖自动安装脚本
# 适用于Ubuntu 22.04 + ROS2 Jazzy/Humble

echo "================================================"
echo "  ROS2 自主探索系统 - 依赖自动安装"
echo "================================================"
echo ""

# 检查是否为root用户
if [ "$EUID" -eq 0 ]; then 
    echo "错误: 请不要使用sudo运行此脚本"
    echo "脚本会在需要时自动请求sudo权限"
    exit 1
fi

# 检测ROS版本
if [ -z "$ROS_DISTRO" ]; then
    echo "错误: 未检测到ROS2环境"
    echo "请先运行: source /opt/ros/<your-distro>/setup.bash"
    exit 1
fi

echo "检测到ROS版本: $ROS_DISTRO"
echo ""

# 确认安装
read -p "是否继续安装所有依赖？ (y/n) " -n 1 -r
echo ""
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "安装已取消"
    exit 0
fi

echo ""
echo "开始安装依赖..."
echo ""

# 更新系统
echo "=== [1/12] 更新系统包管理器 ==="
sudo apt update
echo ""

# 安装编译工具
echo "=== [2/12] 安装编译工具 ==="
sudo apt install -y \
    build-essential \
    cmake \
    git \
    python3-colcon-common-extensions \
    python3-rosdep
echo ""

# 安装ROS2核心包
echo "=== [3/12] 安装ROS2核心包 ==="
sudo apt install -y \
    ros-$ROS_DISTRO-rclcpp \
    ros-$ROS_DISTRO-std-msgs \
    ros-$ROS_DISTRO-geometry-msgs \
    ros-$ROS_DISTRO-nav-msgs \
    ros-$ROS_DISTRO-visualization-msgs \
    ros-$ROS_DISTRO-trajectory-msgs \
    ros-$ROS_DISTRO-sensor-msgs
echo ""

# 安装PCL
echo "=== [4/12] 安装PCL点云库 ==="
sudo apt install -y \
    libpcl-dev \
    ros-$ROS_DISTRO-pcl-conversions \
    ros-$ROS_DISTRO-pcl-ros
echo ""

# 安装Octomap
echo "=== [5/12] 安装Octomap 3D地图库 ==="
sudo apt install -y \
    liboctomap-dev \
    octomap-tools \
    ros-$ROS_DISTRO-octomap \
    ros-$ROS_DISTRO-octomap-msgs \
    ros-$ROS_DISTRO-octomap-server
echo ""

# 安装Eigen
echo "=== [6/12] 安装Eigen线性代数库 ==="
sudo apt install -y \
    libeigen3-dev \
    ros-$ROS_DISTRO-eigen3-cmake-module
echo ""

# 安装OpenCV
echo "=== [7/12] 安装OpenCV计算机视觉库 ==="
sudo apt install -y \
    libopencv-dev \
    python3-opencv \
    ros-$ROS_DISTRO-cv-bridge \
    ros-$ROS_DISTRO-image-transport \
    ros-$ROS_DISTRO-vision-msgs
echo ""

# 安装TF2
echo "=== [8/12] 安装TF2坐标变换库 ==="
sudo apt install -y \
    ros-$ROS_DISTRO-tf2 \
    ros-$ROS_DISTRO-tf2-ros \
    ros-$ROS_DISTRO-tf2-eigen \
    ros-$ROS_DISTRO-tf2-geometry-msgs
echo ""

# 安装轨迹生成依赖
echo "=== [9/12] 安装轨迹生成依赖 ==="
sudo apt install -y \
    libglog-dev \
    libnlopt-dev \
    libnlopt-cxx-dev \
    libyaml-cpp-dev \
    libboost-all-dev
echo ""

# 安装深度图像处理
echo "=== [10/12] 安装深度图像处理包 ==="
sudo apt install -y \
    ros-$ROS_DISTRO-depth-image-proc \
    ros-$ROS_DISTRO-image-pipeline
echo ""

# 安装可视化工具
echo "=== [11/12] 安装可视化工具（可选） ==="
sudo apt install -y \
    ros-$ROS_DISTRO-rviz2 \
    ros-$ROS_DISTRO-rviz-common \
    ros-$ROS_DISTRO-rqt \
    ros-$ROS_DISTRO-rqt-common-plugins \
    ros-$ROS_DISTRO-rqt-graph
echo ""

# 初始化rosdep（如果未初始化）
echo "=== [12/12] 配置rosdep ==="
if [ ! -d "/etc/ros/rosdep/sources.list.d" ]; then
    sudo rosdep init
fi
rosdep update
echo ""

echo "================================================"
echo "✓ 所有系统依赖安装完成！"
echo "================================================"
echo ""
echo "下一步操作："
echo ""
echo "1. 进入工作空间并使用rosdep安装剩余依赖："
echo "   cd ~/ros2_ws"
echo "   rosdep install --from-paths src --ignore-src -r -y"
echo ""
echo "2. 按依赖顺序编译各个包："
echo "   # 编译消息包"
echo "   colcon build --packages-select mav_msgs mav_planning_msgs"
echo "   source install/setup.bash"
echo ""
echo "   # 编译状态机"
echo "   colcon build --packages-select state_machine"
echo "   source install/setup.bash"
echo ""
echo "   # 编译轨迹生成"
echo "   colcon build --packages-select mav_trajectory_generation"
echo "   source install/setup.bash"
echo ""
echo "   # 编译其他包"
echo "   colcon build --packages-select controller_pkg perception planner_pkg"
echo "   source install/setup.bash"
echo ""
echo "   # 或一次性编译所有："
echo "   colcon build"
echo ""
echo "3. 验证依赖是否完整："
echo "   ./check_dependencies.sh"
echo ""
echo "================================================"
