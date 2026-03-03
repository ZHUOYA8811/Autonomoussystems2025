# 无人机不飞问题排查指南 🚁

## 快速诊断

运行诊断脚本：
```bash
cd ~/Desktop/as\ project/autonomoussystems2025/ros2_ws
source install/setup.bash
./check_system.sh
```

---

## 常见问题和解决方案

### 问题 1: Unity 仿真未启动 ❌

**症状**：
- 没有 `/current_state_est` 话题
- `check_system.sh` 显示 Unity 未运行

**解决方案**：
```bash
# 确保 Unity 仿真已启动
cd ~/Desktop/as\ project/autonomoussystems2025/ros2_ws
source install/setup.bash
ros2 launch simulation full_system.launch.py
```

或者使用完整系统启动：
```bash
ros2 launch full_system.launch.py
```

---

### 问题 2: 控制器未收到状态 ⚠️

**症状**：
- 控制器日志显示："Waiting for states: current=NO, desired=NO"
- 无人机不移动

**检查步骤**：

1. **检查里程计话题**：
```bash
ros2 topic echo /current_state_est
```
如果没有输出，说明 Unity 或状态估计节点有问题。

2. **检查轨迹话题**：
```bash
ros2 topic echo /command/trajectory
```
如果没有输出，说明 waypoint_node 没有发布轨迹。

**解决方案**：
- 重启整个系统
- 检查所有节点是否正常运行：`ros2 node list`

---

### 问题 3: Waypoint Node 未收到里程计 📡

**症状**：
- waypoint_node 日志显示："Waiting for odometry..."
- 没有 "Received first odometry" 消息

**解决方案**：

1. 确认话题重映射正确：
```bash
ros2 topic info /current_state_est
```

2. 检查 state_estimate_corruptor 是否在运行：
```bash
ros2 node info /state_estimate_corruptor
```

3. 如果需要，手动启动 waypoint_node：
```bash
ros2 run waypoint_pkg waypoint_node --ros-args \
  -p odom_topic:=current_state_est \
  -p trajectory_topic:=command/trajectory \
  -p takeoff_height:=5.0 \
  -p cruise_speed:=2.0
```

---

### 问题 4: 电机命令为零 🔇

**症状**：
- `/rotor_speed_cmds` 话题显示全是 0
- 无人机不动

**检查**：
```bash
ros2 topic echo /rotor_speed_cmds
```

**可能原因**：
1. 控制器未收到期望状态或当前状态
2. 控制器增益太小
3. 轨迹命令和当前位置完全相同（已到达目标）

**解决方案**：
- 查看控制器日志
- 检查控制器参数：
```bash
ros2 param list /controller_node
ros2 param get /controller_node kx
ros2 param get /controller_node kv
```

---

### 问题 5: 轨迹生成后立即停止 🛑

**症状**：
- 看到 "All waypoints reached! Hovering at final position."
- 但无人机还在地面

**原因**：
- ✅ 已修复：现在到达最后航点后会继续发布悬停命令

**如果仍有问题**：
```bash
# 检查航点距离阈值
ros2 param get /waypoint_node waypoint_threshold
# 如果太大，减小它
ros2 param set /waypoint_node waypoint_threshold 0.3
```

---

## 完整的启动顺序

### 方法 1: 使用 launch 文件（推荐）

```bash
cd ~/Desktop/as\ project/autonomoussystems2025/ros2_ws
source install/setup.bash
ros2 launch full_system.launch.py
```

### 方法 2: 分步启动（调试用）

1. **启动仿真**：
```bash
# 终端 1
cd ~/Desktop/as\ project/autonomoussystems2025/ros2_ws
source install/setup.bash
ros2 launch simulation simulation.launch.py
```

2. **启动感知**（可选）：
```bash
# 终端 2
source install/setup.bash
ros2 launch perception perception.launch.py
```

3. **启动航点导航**：
```bash
# 终端 3
source install/setup.bash
ros2 run waypoint_pkg waypoint_node
```

---

## 实时监控

### 查看所有话题和频率
```bash
ros2 topic list
ros2 topic hz /current_state_est
ros2 topic hz /command/trajectory
ros2 topic hz /rotor_speed_cmds
```

### 查看无人机位置
```bash
ros2 topic echo /current_state_est --field pose.pose.position
```

### 查看目标位置
```bash
ros2 topic echo /command/trajectory --field points[0].transforms[0].translation
```

### 查看电机命令
```bash
ros2 topic echo /rotor_speed_cmds --field angular_velocities
```

---

## 最新修复 ✅

### 2026-03-02 修复

1. **悬停问题**：到达最后航点后继续发布悬停命令
2. **调试信息**：添加了更多状态日志
3. **控制器警告**：增加了等待状态的提示信息

### 修改的文件：
- `src/waypoint_pkg/src/waypoint_node.cpp`
- `src/controller_pkg/src/controller_node.cpp`

---

## 需要帮助？

如果以上步骤都无法解决问题，请提供以下信息：

1. `ros2 node list` 的输出
2. `ros2 topic list` 的输出
3. `./check_system.sh` 的完整输出
4. 控制器和 waypoint_node 的日志

```bash
# 获取节点日志
ros2 run rqt_console rqt_console
```

或者：
```bash
# 查看特定节点的输出
ros2 node info /controller_node
ros2 node info /waypoint_node
```
