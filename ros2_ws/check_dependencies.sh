#!/bin/bash

# ROS2探索系统依赖检查脚本
# 运行此脚本检查所有必需的依赖是否已安装

echo "================================================"
echo "  ROS2 自主探索系统 - 依赖检查工具"
echo "================================================"
echo ""

# 颜色定义
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

MISSING_COUNT=0

# 检查函数
check_command() {
    if command -v $1 &> /dev/null; then
        echo -e "${GREEN}✓${NC} $1"
        return 0
    else
        echo -e "${RED}✗${NC} $1 - 命令未找到"
        ((MISSING_COUNT++))
        return 1
    fi
}

check_pkg() {
    if dpkg -l | grep -q "^ii.*$1"; then
        echo -e "${GREEN}✓${NC} $1"
        return 0
    else
        echo -e "${RED}✗${NC} $1 - 未安装"
        ((MISSING_COUNT++))
        return 1
    fi
}

check_ros_pkg() {
    if ros2 pkg list 2>/dev/null | grep -q "^$1$"; then
        echo -e "${GREEN}✓${NC} $1"
        return 0
    else
        echo -e "${RED}✗${NC} $1 - 未找到"
        ((MISSING_COUNT++))
        return 1
    fi
}

check_lib() {
    if pkg-config --exists $1 2>/dev/null; then
        VERSION=$(pkg-config --modversion $1 2>/dev/null)
        echo -e "${GREEN}✓${NC} $1 (版本: $VERSION)"
        return 0
    else
        echo -e "${RED}✗${NC} $1 - 库未找到"
        ((MISSING_COUNT++))
        return 1
    fi
}

check_header() {
    if [ -f "$1" ]; then
        echo -e "${GREEN}✓${NC} $2"
        return 0
    else
        echo -e "${RED}✗${NC} $2 - 头文件未找到"
        ((MISSING_COUNT++))
        return 1
    fi
}

# 检查ROS2环境
echo "=== ROS2 环境检查 ==="
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${RED}✗${NC} ROS2环境未加载！"
    echo "请先运行: source /opt/ros/<your-distro>/setup.bash"
    exit 1
else
    echo -e "${GREEN}✓${NC} ROS_DISTRO: $ROS_DISTRO"
fi
echo ""

# 检查基本命令
echo "=== 基本工具检查 ==="
check_command cmake
check_command g++
check_command colcon
check_command ros2
echo ""

# 检查系统库
echo "=== 系统库检查 ==="
check_pkg libpcl-dev
check_pkg liboctomap-dev
check_pkg libeigen3-dev
check_pkg libopencv-dev
check_pkg libglog-dev
check_pkg libnlopt-dev
check_pkg libyaml-cpp-dev
check_pkg libboost-dev
echo ""

# 检查库版本
echo "=== 库版本检查 ==="
check_lib "eigen3"
check_lib "opencv4" || check_lib "opencv"
check_lib "octomap"
echo ""

# 检查关键头文件
echo "=== 头文件检查 ==="
check_header "/usr/include/eigen3/Eigen/Dense" "Eigen/Dense"
check_header "/usr/include/pcl-1.14/pcl/point_cloud.h" "PCL Headers" || \
check_header "/usr/include/pcl-1.12/pcl/point_cloud.h" "PCL Headers" || \
check_header "/usr/include/pcl/point_cloud.h" "PCL Headers"
check_header "/usr/include/octomap/OcTree.h" "Octomap Headers"
echo ""

# 检查ROS2核心包
echo "=== ROS2 核心包检查 ==="
check_ros_pkg "rclcpp"
check_ros_pkg "std_msgs"
check_ros_pkg "geometry_msgs"
check_ros_pkg "nav_msgs"
check_ros_pkg "visualization_msgs"
check_ros_pkg "sensor_msgs"
check_ros_pkg "trajectory_msgs"
echo ""

# 检查ROS2扩展包
echo "=== ROS2 扩展包检查 ==="
check_ros_pkg "pcl_conversions"
check_ros_pkg "pcl_ros"
check_ros_pkg "octomap_msgs"
check_ros_pkg "cv_bridge"
check_ros_pkg "image_transport"
check_ros_pkg "tf2"
check_ros_pkg "tf2_ros"
check_ros_pkg "tf2_eigen"
check_ros_pkg "tf2_geometry_msgs"
echo ""

# 检查自定义包（如果已编译）
echo "=== 自定义包检查（工作空间） ==="
if [ -f "$HOME/ros2_ws/install/setup.bash" ] || [ -f "install/setup.bash" ]; then
    # Source工作空间
    if [ -f "$HOME/ros2_ws/install/setup.bash" ]; then
        source "$HOME/ros2_ws/install/setup.bash"
    elif [ -f "install/setup.bash" ]; then
        source "install/setup.bash"
    fi
    
    check_ros_pkg "state_machine" && echo -e "  ${YELLOW}(已编译)${NC}" || echo -e "  ${YELLOW}(待编译)${NC}"
    check_ros_pkg "mav_msgs" && echo -e "  ${YELLOW}(已编译)${NC}" || echo -e "  ${YELLOW}(待编译)${NC}"
    check_ros_pkg "mav_planning_msgs" && echo -e "  ${YELLOW}(已编译)${NC}" || echo -e "  ${YELLOW}(待编译)${NC}"
    check_ros_pkg "mav_trajectory_generation" && echo -e "  ${YELLOW}(已编译)${NC}" || echo -e "  ${YELLOW}(待编译)${NC}"
    check_ros_pkg "controller_pkg" && echo -e "  ${YELLOW}(已编译)${NC}" || echo -e "  ${YELLOW}(待编译)${NC}"
    check_ros_pkg "perception" && echo -e "  ${YELLOW}(已编译)${NC}" || echo -e "  ${YELLOW}(待编译)${NC}"
    check_ros_pkg "planner_pkg" && echo -e "  ${YELLOW}(已编译)${NC}" || echo -e "  ${YELLOW}(待编译)${NC}"
else
    echo -e "${YELLOW}⚠${NC} 工作空间未编译，跳过自定义包检查"
fi
echo ""

# 检查可选工具
echo "=== 可选工具检查 ==="
check_command rviz2 || echo "  (建议安装: sudo apt install ros-$ROS_DISTRO-rviz2)"
check_command rqt || echo "  (建议安装: sudo apt install ros-$ROS_DISTRO-rqt)"
echo ""

# 总结
echo "================================================"
if [ $MISSING_COUNT -eq 0 ]; then
    echo -e "${GREEN}✓ 所有依赖检查通过！${NC}"
    echo "您可以开始编译项目了："
    echo "  cd ~/ros2_ws"
    echo "  colcon build"
else
    echo -e "${RED}✗ 发现 $MISSING_COUNT 个缺失项${NC}"
    echo ""
    echo "建议执行以下操作："
    echo "  1. 运行自动安装脚本："
    echo "     ./install_dependencies.sh"
    echo ""
    echo "  2. 或手动安装缺失的包："
    echo "     sudo apt update"
    echo "     sudo apt install <package-name>"
    echo ""
    echo "  3. 使用rosdep自动安装："
    echo "     cd ~/ros2_ws"
    echo "     rosdep install --from-paths src --ignore-src -r -y"
fi
echo "================================================"

exit $MISSING_COUNT
