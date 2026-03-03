#!/bin/bash
# 无人机系统诊断脚本

echo "========================================="
echo "无人机系统状态检查"
echo "========================================="
echo ""

# 检查 ROS2 环境
echo "1. 检查 ROS2 环境..."
if [ -z "$ROS_DISTRO" ]; then
    echo "   ❌ ROS2 环境未加载"
    echo "   请运行: source install/setup.bash"
    exit 1
else
    echo "   ✅ ROS_DISTRO: $ROS_DISTRO"
fi
echo ""

# 检查关键节点是否运行
echo "2. 检查关键节点..."
NODES=$(ros2 node list 2>/dev/null)

check_node() {
    if echo "$NODES" | grep -q "$1"; then
        echo "   ✅ $1"
        return 0
    else
        echo "   ❌ $1 (未运行)"
        return 1
    fi
}

check_node "/controller_node"
check_node "/waypoint_node"
check_node "/unity_ros"
check_node "/state_estimate_corruptor"
echo ""

# 检查关键话题
echo "3. 检查关键话题..."
TOPICS=$(ros2 topic list 2>/dev/null)

check_topic() {
    if echo "$TOPICS" | grep -q "$1"; then
        echo "   ✅ $1"
        # 显示发布频率
        HZ=$(timeout 2 ros2 topic hz $1 2>&1 | grep "average rate" | awk '{print $3}')
        if [ -n "$HZ" ]; then
            echo "      频率: ${HZ} Hz"
        fi
        return 0
    else
        echo "   ❌ $1 (不存在)"
        return 1
    fi
}

check_topic "/current_state_est"
check_topic "/command/trajectory"
check_topic "/rotor_speed_cmds"
echo ""

# 检查话题数据
echo "4. 检查话题数据 (采样2秒)..."

echo "   检查 /current_state_est ..."
timeout 2 ros2 topic echo /current_state_est --once 2>/dev/null | grep -A 3 "position:" | head -4
if [ $? -eq 0 ]; then
    echo "   ✅ 正在接收里程计数据"
else
    echo "   ❌ 没有里程计数据"
fi
echo ""

echo "   检查 /command/trajectory ..."
timeout 2 ros2 topic echo /command/trajectory --once 2>/dev/null | grep -A 3 "translation:" | head -4
if [ $? -eq 0 ]; then
    echo "   ✅ 正在接收轨迹命令"
else
    echo "   ❌ 没有轨迹命令"
fi
echo ""

echo "   检查 /rotor_speed_cmds ..."
timeout 2 ros2 topic echo /rotor_speed_cmds --once 2>/dev/null | grep "angular_velocities:" -A 1 | head -2
if [ $? -eq 0 ]; then
    echo "   ✅ 正在接收电机命令"
else
    echo "   ❌ 没有电机命令"
fi
echo ""

# 检查 Unity 是否运行
echo "5. 检查 Unity 仿真..."
UNITY_PROC=$(pgrep -f "Simulation.x86_64")
if [ -n "$UNITY_PROC" ]; then
    echo "   ✅ Unity 仿真正在运行 (PID: $UNITY_PROC)"
else
    echo "   ❌ Unity 仿真未运行"
    echo "      请检查 simulation 包中的 Simulation.x86_64"
fi
echo ""

echo "========================================="
echo "诊断完成"
echo "========================================="
echo ""
echo "如果发现问题，请检查："
echo "1. 是否启动了 full_system.launch.py"
echo "2. Unity 仿真是否正常运行"
echo "3. 查看节点日志: ros2 node list 和 ros2 topic echo <topic_name>"
echo ""
