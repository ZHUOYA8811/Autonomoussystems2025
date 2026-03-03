# 自主无人机探索系统 - 完整指南

## 🎯 系统概述

这是一个完整的ROS2自主无人机探索系统，实现了：
- ✅ 前沿探索（Frontier-based Exploration）
- ✅ 3D地图构建（Octomap）
- ✅ 目标检测（灯笼识别）
- ✅ 轨迹规划与跟踪
- ✅ 状态机任务管理

## 📦 包结构

```
ros2_ws/src/
├── state_machine/          # 任务状态机（总控）
├── controller_pkg/         # 飞行控制器
├── planner_pkg/           # 探索规划器（新增）
│   ├── explorer_node           → 前沿检测
│   └── trajectory_planner_node → 轨迹生成
├── mav_trajectory_generation/  # 轨迹采样器
├── perception/            # 视觉感知
│   ├── light_detection_node  → 灯笼检测
│   └── octomap_server       → 地图构建
└── simulation/            # Unity仿真桥接
```

## 🚀 快速启动

### 方法1: 一键启动完整系统（推荐）

```bash
# 1. 启动仿真（如果需要）
ros2 launch simulation simulation.launch.py

# 2. 在另一个终端启动完整自主系统
cd ~/ros2_ws
source install/setup.bash
ros2 launch state_machine full_system.launch.py
```

这将自动启动：
- 感知系统（Octomap + 灯笼检测）
- 控制器
- 轨迹采样器
- 探索系统（前沿检测 + 轨迹规划）
- 状态机

### 方法2: 分步骤启动（调试用）

```bash
# 终端1: 仿真
ros2 launch simulation simulation.launch.py

# 终端2: 感知
ros2 launch perception perception.launch.py

# 终端3: 控制器
ros2 run controller_pkg controller_wrapper

# 终端4: 采样器
ros2 run mav_trajectory_generation trajectory_sampler_node

# 终端5: 探索系统
ros2 launch planner_pkg exploration_system.launch.py

# 终端6: 状态机
ros2 launch state_machine state_machine.launch.py
```

## 🔄 系统工作流程

```
[WAITING] 等待所有节点上线
    ↓
[TAKEOFF] 垂直起飞到5米
    ↓
[TRAVELLING] 飞往任务区域 (-320, 10, 15)
    ↓
[EXPLORING] 🆕 自主探索搜索灯笼
    │
    ├─→ Frontier Explorer 检测前沿
    ├─→ Trajectory Planner 生成轨迹
    ├─→ Sampler 采样轨迹
    ├─→ Controller 执行飞行
    └─→ 持续直到检测到5个灯笼
    ↓
[RETURN_HOME] 返回起点 (-38, 10, 8)
    ↓
[LAND] 降落
    ↓
[DONE] 任务完成
```

## 📡 关键话题

### 输入
- `/current_state_est` - 无人机位置（Odometry）
- `/octomap_full` - 3D占据栅格地图
- `/statemachine/cmd` - 状态机指令

### 输出
- `/frontier_point` - 探索目标点
- `/path_segments_4D` - 多项式轨迹
- `/command_trajectory` - 控制指令
- `/detected_points` - 检测到的灯笼位置

### 可视化
- `/frontier_point_marker` - 探索目标（紫红球）
- `/lantern_marker` - 灯笼位置（橙球）
- `/octomap_full` - 3D地图

## 🛠️ 编译系统

```bash
cd ~/ros2_ws

# 首次编译或修改后
colcon build

# 只编译特定包
colcon build --packages-select planner_pkg state_machine

# 清理重新编译
rm -rf build/ install/ log/
colcon build

# 记得source
source install/setup.bash
```

## 🔍 调试与监控

### 查看系统状态
```bash
# 列出所有节点
ros2 node list

# 列出所有话题
ros2 topic list

# 查看话题频率
ros2 topic hz /frontier_point

# 查看话题内容
ros2 topic echo /frontier_point

# 查看节点健康
ros2 topic echo /statemachine/node_health
```

### RViz可视化
```bash
rviz2

# 添加以下显示:
# - Marker: /frontier_point_marker (探索目标)
# - Marker: /lantern_marker (灯笼)
# - OccupancyGrid: /octomap_full (3D地图)
# - Odometry: /current_state_est (无人机位置)
```

### 查看日志
```bash
# 查看特定节点日志
ros2 run rqt_console rqt_console

# 或直接查看
ros2 node info /frontier_explorer
```

## 🎮 手动控制

### 手动触发探索
```bash
# 启动探索
ros2 topic pub --once /statemachine/cmd state_machine/msg/Command \
  "{target: 'planner', command: 2}"

# 停止探索
ros2 topic pub --once /statemachine/cmd state_machine/msg/Command \
  "{target: 'planner', command: 3}"
```

### 手动设置探索目标
```bash
# 发送自定义探索点
ros2 topic pub --once /frontier_point geometry_msgs/msg/Point \
  "{x: -400.0, y: 10.0, z: 15.0}"
```

## ⚙️ 参数配置

### 探索器参数 (explorer_node)
```yaml
octomap_resolution: 0.4       # 地图分辨率
max_distance: 30              # 初始搜索半径（米）
max_search_distance: 400      # 最大搜索半径（米）
exploration_rate: 0.3         # 更新频率（Hz）
movement_threshold: 3.0       # 移动检测阈值（米）
stall_time_threshold: 10.0    # 卡住检测阈值（秒）
```

### 轨迹规划器参数 (trajectory_planner_node)
```yaml
max_velocity: 3.0            # 最大速度（m/s）
max_acceleration: 2.0        # 最大加速度（m/s²）
```

### 修改参数示例
```bash
ros2 launch planner_pkg exploration_system.launch.py \
  max_distance:=50 \
  max_velocity:=4.0 \
  exploration_rate:=0.5
```

## 📊 性能优化

### 降低CPU负载
```bash
# 降低探索频率
exploration_rate: 0.2  # 从0.3降到0.2

# 减小搜索范围
max_distance: 20  # 从30降到20
```

### 提高探索速度
```bash
# 提高飞行速度
max_velocity: 4.0  # 从3.0提到4.0
max_acceleration: 3.0  # 从2.0提到3.0

# 增加探索频率
exploration_rate: 0.5  # 从0.3提到0.5
```

## ❓ 故障排查

### 问题1: "No frontier points detected"
**原因**: Octomap未生成或质量差
**解决**:
```bash
# 检查octomap
ros2 topic hz /octomap_full
ros2 topic echo /octomap_full --once

# 确保感知系统运行
ros2 node list | grep perception
```

### 问题2: "Planner not starting"
**原因**: 未收到START指令
**解决**:
```bash
# 检查状态机状态
ros2 topic echo /statemachine/state

# 手动发送START
ros2 topic pub --once /statemachine/cmd state_machine/msg/Command \
  "{target: 'planner', command: 2}"
```

### 问题3: "Trajectory generation failed"
**原因**: mav_trajectory_generation未正确编译
**解决**:
```bash
cd ~/ros2_ws
colcon build --packages-select mav_trajectory_generation
source install/setup.bash
```

### 问题4: "Cannot find package 'planner_pkg'"
**原因**: 包未编译或未source
**解决**:
```bash
cd ~/ros2_ws
colcon build --packages-select planner_pkg
source install/setup.bash
```

### 问题5: 节点健康检查失败
**原因**: state_machine未监控planner
**解决**: 确认 `state_machine.launch.py` 包含:
```python
'monitored_node_list': ['controller', 'sampler', 'planner']
```

## 📚 详细文档

- **探索包详情**: `planner_pkg/README.md`
- **系统集成指南**: `planner_pkg/INTEGRATION.md`
- **状态机说明**: `state_machine/README.md`
- **感知系统**: `perception/README.md`

## 🧪 测试

### 基础功能测试
```bash
# 1. 测试节点启动
ros2 run planner_pkg explorer_node
ros2 run planner_pkg trajectory_planner_node

# 2. 测试话题通信
ros2 topic pub --once /frontier_point geometry_msgs/msg/Point "{x: -400, y: 10, z: 15}"

# 3. 检查健康报告
ros2 topic echo /statemachine/node_health
```

### 完整系统测试
```bash
# 1. 启动完整系统
ros2 launch state_machine full_system.launch.py

# 2. 等待EXPLORING状态
ros2 topic echo /statemachine/state

# 3. 观察探索点发布
ros2 topic echo /frontier_point

# 4. 验证轨迹生成
ros2 topic echo /path_segments_4D
```

## 📝 开发者注意事项

### 添加新功能
1. 修改 `planner_pkg` 源代码
2. 重新编译: `colcon build --packages-select planner_pkg`
3. Source: `source install/setup.bash`
4. 测试功能
5. 更新文档

### 代码规范
- 使用有意义的变量名
- 添加注释说明复杂逻辑
- 遵循ROS2编码规范
- 更新相关文档

### 性能分析
```bash
# 查看CPU使用
top -p $(pgrep -f explorer_node)

# 查看内存使用
ros2 run rqt_top rqt_top

# 分析话题带宽
ros2 topic bw /octomap_full
```

## 🔗 相关资源

- [ROS2 Documentation](https://docs.ros.org/en/jazzy/)
- [Octomap](https://octomap.github.io/)
- [PCL Library](https://pointclouds.org/)
- [MAV Trajectory Generation](https://github.com/ethz-asl/mav_trajectory_generation)

## 📄 许可证

MIT License

## 👥 维护者

- State Machine & Integration: Your Team
- Exploration Package: Based on ROS1 implementation, ported to ROS2

## 🎉 致谢

基于 autonomoussystems2024_team8 的 ROS1 探索代码移植而来。
