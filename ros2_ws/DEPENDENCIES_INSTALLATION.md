# 依赖项安装指南

## 📋 完整依赖清单

### 1. ROS2 基础包（通常已安装）
- `rclcpp` - ROS2 C++ 客户端库
- `std_msgs` - 标准消息类型
- `geometry_msgs` - 几何消息类型
- `nav_msgs` - 导航消息类型
- `visualization_msgs` - 可视化消息类型
- `trajectory_msgs` - 轨迹消息类型
- `sensor_msgs` - 传感器消息类型

### 2. 第三方库（需要安装）
- **PCL (Point Cloud Library)** - 点云处理
- **Octomap** - 3D占据栅格地图
- **Eigen3** - 线性代数库
- **OpenCV** - 计算机视觉
- **Boost** - C++辅助库（通常已安装）
- **GLOG** - 日志库
- **NLOPT** - 优化库

### 3. ROS2 扩展包（需要安装）
- `pcl_conversions` - PCL与ROS2消息转换
- `pcl_ros` - PCL的ROS2接口
- `octomap_msgs` - Octomap消息类型
- `tf2` / `tf2_ros` - 坐标变换
- `cv_bridge` - OpenCV与ROS2桥接
- `image_transport` - 图像传输

### 4. 自定义包（需要编译）
- `state_machine` - 状态机包
- `mav_msgs` - MAV消息定义
- `mav_planning_msgs` - MAV规划消息
- `mav_trajectory_generation` - 轨迹生成库
- `controller_pkg` - 控制器包
- `perception` - 感知包
- `simulation` - 仿真接口

---

## 🔧 安装步骤

### Step 1: 更新系统包管理器

```bash
sudo apt update
sudo apt upgrade -y
```

### Step 2: 安装ROS2核心依赖（如果缺失）

```bash
# ROS2 Jazzy (或您使用的版本)
sudo apt install -y \
    ros-jazzy-rclcpp \
    ros-jazzy-std-msgs \
    ros-jazzy-geometry-msgs \
    ros-jazzy-nav-msgs \
    ros-jazzy-visualization-msgs \
    ros-jazzy-trajectory-msgs \
    ros-jazzy-sensor-msgs
```

**注意**：将 `jazzy` 替换为您的ROS2版本（humble, iron, rolling等）

### Step 3: 安装PCL相关包

```bash
# PCL库及ROS2接口
sudo apt install -y \
    libpcl-dev \
    ros-jazzy-pcl-conversions \
    ros-jazzy-pcl-ros
```

### Step 4: 安装Octomap相关包

```bash
# Octomap库及ROS2消息
sudo apt install -y \
    liboctomap-dev \
    octomap-tools \
    ros-jazzy-octomap \
    ros-jazzy-octomap-msgs \
    ros-jazzy-octomap-server
```

### Step 5: 安装Eigen3

```bash
sudo apt install -y \
    libeigen3-dev \
    ros-jazzy-eigen3-cmake-module
```

### Step 6: 安装OpenCV及相关ROS2包

```bash
# OpenCV及cv_bridge
sudo apt install -y \
    libopencv-dev \
    python3-opencv \
    ros-jazzy-cv-bridge \
    ros-jazzy-image-transport \
    ros-jazzy-vision-msgs
```

### Step 7: 安装TF2相关包

```bash
sudo apt install -y \
    ros-jazzy-tf2 \
    ros-jazzy-tf2-ros \
    ros-jazzy-tf2-eigen \
    ros-jazzy-tf2-geometry-msgs
```

### Step 8: 安装轨迹生成依赖

```bash
# GLOG日志库
sudo apt install -y libglog-dev

# NLOPT优化库
sudo apt install -y libnlopt-dev libnlopt-cxx-dev

# YAML-CPP
sudo apt install -y libyaml-cpp-dev
```

### Step 9: 安装其他工具

```bash
# 编译工具
sudo apt install -y \
    build-essential \
    cmake \
    git

# ROS2开发工具
sudo apt install -y \
    python3-colcon-common-extensions \
    python3-rosdep

# 可视化工具（可选）
sudo apt install -y \
    ros-jazzy-rviz2 \
    ros-jazzy-rqt \
    ros-jazzy-rqt-common-plugins
```

### Step 10: 初始化rosdep（如果未初始化）

```bash
sudo rosdep init
rosdep update
```

### Step 11: 使用rosdep自动安装工作空间依赖

```bash
cd ~/ros2_ws  # 或您的工作空间路径
rosdep install --from-paths src --ignore-src -r -y
```

---

## 📦 按包分类的依赖

### planner_pkg 依赖
```bash
sudo apt install -y \
    libpcl-dev \
    ros-jazzy-pcl-conversions \
    ros-jazzy-pcl-ros \
    liboctomap-dev \
    ros-jazzy-octomap-msgs \
    libeigen3-dev \
    ros-jazzy-eigen3-cmake-module \
    ros-jazzy-tf2 \
    ros-jazzy-tf2-ros
```

### perception 依赖
```bash
sudo apt install -y \
    libopencv-dev \
    ros-jazzy-cv-bridge \
    ros-jazzy-image-transport \
    ros-jazzy-sensor-msgs \
    ros-jazzy-tf2-geometry-msgs \
    liboctomap-dev \
    ros-jazzy-octomap-server
```

### mav_trajectory_generation 依赖
```bash
sudo apt install -y \
    libeigen3-dev \
    libglog-dev \
    libnlopt-dev \
    libyaml-cpp-dev
```

### controller_pkg 依赖
```bash
sudo apt install -y \
    libeigen3-dev \
    ros-jazzy-tf2-eigen \
    ros-jazzy-eigen3-cmake-module
```

---

## 🐳 Docker环境（推荐）

如果使用Docker容器，可以基于官方ROS2镜像：

```dockerfile
FROM ros:jazzy

# 安装所有依赖
RUN apt-get update && apt-get install -y \
    libpcl-dev \
    ros-jazzy-pcl-conversions \
    ros-jazzy-pcl-ros \
    liboctomap-dev \
    ros-jazzy-octomap \
    ros-jazzy-octomap-msgs \
    ros-jazzy-octomap-server \
    libeigen3-dev \
    ros-jazzy-eigen3-cmake-module \
    libopencv-dev \
    ros-jazzy-cv-bridge \
    ros-jazzy-image-transport \
    ros-jazzy-tf2 \
    ros-jazzy-tf2-ros \
    ros-jazzy-tf2-eigen \
    ros-jazzy-tf2-geometry-msgs \
    libglog-dev \
    libnlopt-dev \
    libyaml-cpp-dev \
    ros-jazzy-rviz2 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
```

---

## ✅ 验证安装

### 检查ROS2包
```bash
# 检查PCL
ros2 pkg list | grep pcl

# 检查Octomap
ros2 pkg list | grep octomap

# 检查TF2
ros2 pkg list | grep tf2
```

### 检查库文件
```bash
# 检查PCL库
pkg-config --modversion pcl_common

# 检查Eigen
pkg-config --modversion eigen3

# 检查OpenCV
pkg-config --modversion opencv4

# 检查Octomap
pkg-config --modversion octomap
```

### 测试编译
```bash
cd ~/ros2_ws
colcon build --packages-select planner_pkg
```

---

## 🔍 常见问题

### 问题1: "Could not find PCL"
```bash
# 确保安装了PCL开发包
sudo apt install -y libpcl-dev

# 检查pkg-config
export PKG_CONFIG_PATH=/usr/lib/x86_64-linux-gnu/pkgconfig:$PKG_CONFIG_PATH
```

### 问题2: "Cannot find octomap"
```bash
# 安装完整的octomap套件
sudo apt install -y liboctomap-dev octomap-tools

# 如果是ROS包缺失
sudo apt install -y ros-jazzy-octomap ros-jazzy-octomap-msgs
```

### 问题3: "Eigen3 not found"
```bash
# 安装Eigen3及CMake模块
sudo apt install -y libeigen3-dev ros-jazzy-eigen3-cmake-module

# 如果还是找不到，手动设置路径
export Eigen3_DIR=/usr/share/eigen3/cmake
```

### 问题4: "mav_trajectory_generation not found"
```bash
# 这是自定义包，需要先编译
cd ~/ros2_ws
colcon build --packages-select mav_msgs mav_planning_msgs mav_trajectory_generation
source install/setup.bash
```

### 问题5: "state_machine package not found"
```bash
# 编译state_machine包
cd ~/ros2_ws
colcon build --packages-select state_machine
source install/setup.bash
```

---

## 📝 编译顺序建议

由于包之间有依赖关系，建议按以下顺序编译：

```bash
cd ~/ros2_ws

# 1. 消息定义包
colcon build --packages-select mav_msgs mav_planning_msgs

# 2. Source环境
source install/setup.bash

# 3. 状态机（其他包依赖它）
colcon build --packages-select state_machine
source install/setup.bash

# 4. 轨迹生成库
colcon build --packages-select mav_trajectory_generation
source install/setup.bash

# 5. 控制器和感知
colcon build --packages-select controller_pkg perception
source install/setup.bash

# 6. 规划器（依赖前面所有包）
colcon build --packages-select planner_pkg
source install/setup.bash

# 7. 仿真（可选）
colcon build --packages-select simulation
source install/setup.bash

# 或者一次性编译所有
colcon build
```

---

## 🚀 快速安装脚本

创建 `install_dependencies.sh`:

```bash
#!/bin/bash

# 自动安装所有依赖的脚本

echo "=== 安装ROS2探索系统依赖 ==="

# 检测ROS版本
if [ -z "$ROS_DISTRO" ]; then
    echo "错误: 未检测到ROS2环境，请先source ROS2"
    exit 1
fi

echo "检测到ROS版本: $ROS_DISTRO"

# 更新系统
echo "1. 更新系统..."
sudo apt update

# 安装ROS2核心包
echo "2. 安装ROS2核心包..."
sudo apt install -y \
    ros-$ROS_DISTRO-rclcpp \
    ros-$ROS_DISTRO-std-msgs \
    ros-$ROS_DISTRO-geometry-msgs \
    ros-$ROS_DISTRO-nav-msgs \
    ros-$ROS_DISTRO-visualization-msgs \
    ros-$ROS_DISTRO-trajectory-msgs \
    ros-$ROS_DISTRO-sensor-msgs

# 安装PCL
echo "3. 安装PCL..."
sudo apt install -y \
    libpcl-dev \
    ros-$ROS_DISTRO-pcl-conversions \
    ros-$ROS_DISTRO-pcl-ros

# 安装Octomap
echo "4. 安装Octomap..."
sudo apt install -y \
    liboctomap-dev \
    octomap-tools \
    ros-$ROS_DISTRO-octomap \
    ros-$ROS_DISTRO-octomap-msgs \
    ros-$ROS_DISTRO-octomap-server

# 安装Eigen
echo "5. 安装Eigen3..."
sudo apt install -y \
    libeigen3-dev \
    ros-$ROS_DISTRO-eigen3-cmake-module

# 安装OpenCV
echo "6. 安装OpenCV..."
sudo apt install -y \
    libopencv-dev \
    python3-opencv \
    ros-$ROS_DISTRO-cv-bridge \
    ros-$ROS_DISTRO-image-transport

# 安装TF2
echo "7. 安装TF2..."
sudo apt install -y \
    ros-$ROS_DISTRO-tf2 \
    ros-$ROS_DISTRO-tf2-ros \
    ros-$ROS_DISTRO-tf2-eigen \
    ros-$ROS_DISTRO-tf2-geometry-msgs

# 安装轨迹生成依赖
echo "8. 安装轨迹生成依赖..."
sudo apt install -y \
    libglog-dev \
    libnlopt-dev \
    libyaml-cpp-dev

# 安装工具
echo "9. 安装开发工具..."
sudo apt install -y \
    build-essential \
    cmake \
    git \
    python3-colcon-common-extensions

# 安装可视化工具（可选）
echo "10. 安装可视化工具..."
sudo apt install -y \
    ros-$ROS_DISTRO-rviz2 \
    ros-$ROS_DISTRO-rqt \
    ros-$ROS_DISTRO-rqt-common-plugins

echo "=== 依赖安装完成 ==="
echo "请运行: rosdep install --from-paths src --ignore-src -r -y"
```

使用方法：
```bash
chmod +x install_dependencies.sh
./install_dependencies.sh
```

---

## 📊 依赖检查清单

在编译前，运行以下命令检查所有依赖：

```bash
# 创建检查脚本
cat > check_dependencies.sh << 'EOF'
#!/bin/bash

echo "=== 检查ROS2探索系统依赖 ==="

check_pkg() {
    if dpkg -l | grep -q "^ii.*$1"; then
        echo "✓ $1"
    else
        echo "✗ $1 - 未安装"
    fi
}

check_ros_pkg() {
    if ros2 pkg list | grep -q "^$1$"; then
        echo "✓ $1"
    else
        echo "✗ $1 - 未安装"
    fi
}

echo "--- 系统库 ---"
check_pkg libpcl-dev
check_pkg liboctomap-dev
check_pkg libeigen3-dev
check_pkg libopencv-dev
check_pkg libglog-dev
check_pkg libnlopt-dev
check_pkg libyaml-cpp-dev

echo "--- ROS2包 ---"
check_ros_pkg pcl_conversions
check_ros_pkg pcl_ros
check_ros_pkg octomap_msgs
check_ros_pkg cv_bridge
check_ros_pkg tf2_ros

echo "=== 检查完成 ==="
EOF

chmod +x check_dependencies.sh
./check_dependencies.sh
```

---

## 💡 提示

1. **首次安装**: 建议使用 `install_dependencies.sh` 一键安装所有依赖
2. **Docker用户**: 推荐使用预配置的Docker镜像
3. **编译顺序**: 严格按照依赖顺序编译各个包
4. **环境变量**: 每次编译后记得 `source install/setup.bash`
5. **版本匹配**: 确保所有ROS2包的版本与您的ROS_DISTRO一致

如有任何问题，请查看 `AUTONOMOUS_EXPLORATION_GUIDE.md` 中的故障排查部分。
