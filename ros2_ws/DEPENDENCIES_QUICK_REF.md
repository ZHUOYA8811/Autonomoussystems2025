# 依赖项快速参考

## 🎯 核心依赖（必需）

### 系统库
```bash
sudo apt install -y \
  libpcl-dev \
  liboctomap-dev \
  libeigen3-dev \
  libopencv-dev \
  libglog-dev \
  libnlopt-dev \
  libyaml-cpp-dev
```

### ROS2包（将jazzy替换为您的ROS版本）
```bash
sudo apt install -y \
  ros-jazzy-pcl-conversions \
  ros-jazzy-pcl-ros \
  ros-jazzy-octomap \
  ros-jazzy-octomap-msgs \
  ros-jazzy-octomap-server \
  ros-jazzy-cv-bridge \
  ros-jazzy-image-transport \
  ros-jazzy-tf2 \
  ros-jazzy-tf2-ros \
  ros-jazzy-tf2-eigen \
  ros-jazzy-eigen3-cmake-module
```

## 📦 按包分类

### planner_pkg
- libpcl-dev
- liboctomap-dev
- libeigen3-dev
- ros-jazzy-pcl-ros
- ros-jazzy-octomap-msgs

### perception
- liboctomap-dev
- libopencv-dev
- ros-jazzy-cv-bridge
- ros-jazzy-octomap-server

### mav_trajectory_generation
- libeigen3-dev
- libglog-dev
- libnlopt-dev
- libyaml-cpp-dev

### controller_pkg
- libeigen3-dev
- ros-jazzy-tf2-eigen

## 🚀 快速开始

### 方法1: 自动安装（推荐）
```bash
cd ~/ros2_ws
chmod +x install_dependencies.sh
./install_dependencies.sh
```

### 方法2: 使用rosdep
```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 方法3: 手动安装
参考 `DEPENDENCIES_INSTALLATION.md` 详细步骤

## ✅ 验证安装
```bash
cd ~/ros2_ws
chmod +x check_dependencies.sh
./check_dependencies.sh
```

## 📚 详细文档
- **完整安装指南**: `DEPENDENCIES_INSTALLATION.md`
- **系统集成指南**: `src/planner_pkg/INTEGRATION.md`
- **使用手册**: `AUTONOMOUS_EXPLORATION_GUIDE.md`
