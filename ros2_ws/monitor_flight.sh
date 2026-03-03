#!/bin/bash
# 实时飞行状态监控脚本

echo "======================================"
echo "🚁 无人机实时飞行状态监控"
echo "======================================"
echo ""

# 检查环境
if [ -z "$ROS_DISTRO" ]; then
    echo "❌ 请先运行: source install/setup.bash"
    exit 1
fi

# 实时监控
while true; do
    clear
    echo "======================================"
    echo "🚁 无人机实时飞行状态 $(date +%H:%M:%S)"
    echo "======================================"
    echo ""
    
    # 获取当前位置
    echo "📍 当前位置："
    POS=$(timeout 0.5 ros2 topic echo /current_state_est --field pose.pose.position --once 2>/dev/null)
    if [ -n "$POS" ]; then
        echo "$POS" | while IFS=: read key value; do
            printf "   %s: %s\n" "$key" "$value"
        done
    else
        echo "   ⚠️  无法获取位置数据"
    fi
    echo ""
    
    # 获取目标位置
    echo "🎯 目标位置："
    TARGET=$(timeout 0.5 ros2 topic echo /command/trajectory --once 2>/dev/null | grep -A 1 "translation:" | tail -3)
    if [ -n "$TARGET" ]; then
        echo "$TARGET" | sed 's/^/   /'
    else
        echo "   ⚠️  无法获取目标数据"
    fi
    echo ""
    
    # 获取速度
    echo "💨 当前速度："
    VEL=$(timeout 0.5 ros2 topic echo /current_state_est --field twist.twist.linear --once 2>/dev/null)
    if [ -n "$VEL" ]; then
        echo "$VEL" | while IFS=: read key value; do
            printf "   %s: %s m/s\n" "$key" "$value"
        done
    else
        echo "   ⚠️  无法获取速度数据"
    fi
    echo ""
    
    # 获取电机命令
    echo "🔧 电机转速命令："
    MOTORS=$(timeout 0.5 ros2 topic echo /rotor_speed_cmds --field angular_velocities --once 2>/dev/null)
    if [ -n "$MOTORS" ]; then
        echo "   $MOTORS"
    else
        echo "   ⚠️  无法获取电机数据"
    fi
    echo ""
    
    # 话题频率
    echo "📊 数据流频率："
    HZ_ODOM=$(timeout 1.5 ros2 topic hz /current_state_est 2>&1 | grep "average rate" | awk '{print $3}')
    HZ_TRAJ=$(timeout 1.5 ros2 topic hz /command/trajectory 2>&1 | grep "average rate" | awk '{print $3}')
    HZ_MOTOR=$(timeout 1.5 ros2 topic hz /rotor_speed_cmds 2>&1 | grep "average rate" | awk '{print $3}')
    
    [ -n "$HZ_ODOM" ] && echo "   里程计: ${HZ_ODOM} Hz" || echo "   里程计: ⚠️  无数据"
    [ -n "$HZ_TRAJ" ] && echo "   轨迹命令: ${HZ_TRAJ} Hz" || echo "   轨迹命令: ⚠️  无数据"
    [ -n "$HZ_MOTOR" ] && echo "   电机命令: ${HZ_MOTOR} Hz" || echo "   电机命令: ⚠️  无数据"
    echo ""
    
    echo "======================================"
    echo "按 Ctrl+C 退出监控"
    echo "======================================"
    
    sleep 2
done
