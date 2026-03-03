# 自主探索系统集成指南

## 系统架构

```
┌─────────────────┐
│ State Machine   │  任务总控制器
│ (state_machine) │
└────────┬────────┘
         │ 发送 START/HOLD 指令
         ↓
┌─────────────────┐      ┌──────────────────┐
│ Frontier        │ ---> │ Trajectory       │  将前沿点转换为轨迹
│ Explorer        │      │ Planner          │
│ (explorer_node) │      │ (traj_planner)   │
└─────────────────┘      └────────┬─────────┘
         ↑                        │ path_segments_4D
         │ octomap_full          ↓
┌─────────────────┐      ┌──────────────────┐
│ Perception      │      │ Trajectory       │  轨迹采样
│ (octomap)       │      │ Sampler          │
└─────────────────┘      └────────┬─────────┘
                                  │ command_trajectory
                                  ↓
                         ┌──────────────────┐
                         │ Controller       │  姿态控制
                         └──────────────────┘
```

## 话题通信

### 核心数据流

1. **感知 → 探索器**
   - 话题: `/octomap_full`
   - 类型: `octomap_msgs/msg/Octomap`
   - 功能: 3D占据栅格地图

2. **探索器 → 轨迹规划器**
   - 话题: `/frontier_point`
   - 类型: `geometry_msgs/msg/Point`
   - 功能: 探索目标点

3. **轨迹规划器 → 采样器**
   - 话题: `/path_segments_4D`
   - 类型: `mav_planning_msgs/msg/PolynomialTrajectory4D`
   - 功能: 多项式轨迹段

4. **采样器 → 控制器**
   - 话题: `/command_trajectory`
   - 类型: `trajectory_msgs/msg/MultiDOFJointTrajectory`
   - 功能: 实时轨迹指令

5. **状态机 → 所有节点**
   - 话题: `/statemachine/cmd`
   - 类型: `state_machine/msg/Command`
   - 功能: 启动/停止指令

6. **所有节点 → 状态机**
   - 话题: `/statemachine/node_health`
   - 类型: `state_machine/msg/Answer`
   - 功能: 健康心跳报告

## 节点清单

| 节点名称 | 包名 | 可执行文件 | 功能 |
|---------|------|-----------|------|
| frontier_explorer | planner_pkg | explorer_node | 前沿检测 |
| trajectory_planner | planner_pkg | trajectory_planner_node | 轨迹生成 |
| trajectory_sampler | mav_trajectory_generation | trajectory_sampler_node | 轨迹采样 |
| controller | controller_pkg | controller_wrapper | 飞行控制 |
| state_machine | state_machine | state_machine_node | 任务调度 |
| light_detection | perception | light_detection_node | 灯笼检测 |
| octomap_server | perception | octomap_server | 地图构建 |

## 启动顺序

### 1. 启动仿真环境
```bash
ros2 launch simulation simulation.launch.py
```

### 2. 启动感知系统
```bash
ros2 launch perception perception.launch.py
```

### 3. 启动控制器
```bash
ros2 launch controller_pkg controller.launch.py
```

### 4. 启动轨迹采样器
```bash
ros2 run mav_trajectory_generation trajectory_sampler_node
```

### 5. 启动探索系统（新增）
```bash
ros2 launch planner_pkg exploration_system.launch.py
```

### 6. 启动状态机
```bash
ros2 launch state_machine state_machine.launch.py
```

## 集成到现有系统

### 方法1: 完整系统启动（推荐）

创建主启动文件 `main_system.launch.py`:

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    return LaunchDescription([
        # 1. 仿真
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('simulation'), '/launch/simulation.launch.py'
            ])
        ),
        
        # 2. 感知
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('perception'), '/launch/perception.launch.py'
            ])
        ),
        
        # 3. 控制器
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('controller_pkg'), '/launch/controller.launch.py'
            ])
        ),
        
        # 4. 采样器（轨迹生成）
        Node(
            package='mav_trajectory_generation',
            executable='trajectory_sampler_node',
            name='trajectory_sampler',
            output='screen'
        ),
        
        # 5. 探索系统（新增）
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('planner_pkg'), '/launch/exploration_system.launch.py'
            ])
        ),
        
        # 6. 状态机
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('state_machine'), '/launch/state_machine.launch.py'
            ])
        ),
    ])
```

### 方法2: 分步启动

适合调试，按顺序在不同终端启动各个组件。

## 验证系统运行

### 1. 检查所有节点在线
```bash
ros2 node list
```

应该看到:
- `/frontier_explorer`
- `/trajectory_planner`
- `/trajectory_sampler`
- `/state_machine_manager`
- `/controller_node`
- 等等...

### 2. 检查话题通信
```bash
# 查看前沿点
ros2 topic echo /frontier_point

# 查看生成的轨迹
ros2 topic echo /path_segments_4D

# 查看状态机指令
ros2 topic echo /statemachine/cmd

# 查看节点健康
ros2 topic echo /statemachine/node_health
```

### 3. 手动触发探索
```bash
# 发送 START 指令给 planner
ros2 topic pub --once /statemachine/cmd state_machine/msg/Command \
  "{target: 'planner', command: 2}"
```

### 4. RViz 可视化

添加以下显示:
- **Marker** → `/frontier_point_marker` (探索目标点，紫红色球)
- **OccupancyGrid** → `/octomap_full` (3D地图)
- **Marker** → `/lantern_marker` (已检测灯笼，橙色球)

## 状态机集成修改

`state_machine.launch.py` 已更新监控列表:
```python
'monitored_node_list': ['controller', 'sampler', 'planner']  # 新增 planner
```

状态机在 `EXPLORING` 状态会:
1. 发送 `START` 指令给 `planner`
2. planner 开始检测前沿并发布目标点
3. trajectory_planner 生成轨迹
4. sampler 采样并发给 controller
5. controller 执行飞行

## 参数调优

### 探索器参数 (explorer_node)
- `max_distance`: 初始搜索半径 (默认30m)
- `max_search_distance`: 卡住时最大搜索半径 (默认400m)
- `exploration_rate`: 更新频率 (默认0.3Hz)

### 轨迹规划器参数 (trajectory_planner_node)
- `max_velocity`: 最大速度 (默认3.0 m/s)
- `max_acceleration`: 最大加速度 (默认2.0 m/s²)

### 示例: 提高探索激进度
```bash
ros2 launch planner_pkg exploration_system.launch.py \
  max_velocity:=5.0 \
  max_acceleration:=3.0 \
  exploration_rate:=0.5
```

## 故障排查

### 问题: 没有检测到前沿点
**原因**: Octomap未生成或分辨率不匹配
**解决**:
```bash
# 检查octomap
ros2 topic hz /octomap_full

# 调整分辨率参数使其与octomap_server一致
ros2 param get /octomap_server resolution
```

### 问题: 轨迹规划器不工作
**原因**: 未安装 mav_trajectory_generation
**解决**:
```bash
cd ~/ros2_ws
colcon build --packages-select mav_trajectory_generation
source install/setup.bash
```

### 问题: 探索器不启动
**原因**: 未收到 START 指令
**解决**:
```bash
# 检查状态机状态
ros2 topic echo /statemachine/state

# 手动发送启动指令
ros2 topic pub --once /statemachine/cmd state_machine/msg/Command \
  "{target: 'planner', command: 2}"
```

### 问题: 节点健康报告失败
**原因**: state_machine 包未找到
**解决**:
```bash
cd ~/ros2_ws
colcon build --packages-select state_machine planner_pkg
source install/setup.bash
```

## 性能优化

1. **降低探索频率** (减少CPU负载):
   ```
   exploration_rate: 0.2  # 从0.3降到0.2
   ```

2. **减小初始搜索范围** (加快计算):
   ```
   max_distance: 20  # 从30降到20
   ```

3. **调整OPTICS聚类参数** (代码中硬编码):
   - `minPts`: 5 → 3 (更小的簇)
   - `epsilon`: 10.0 → 15.0 (更大的搜索半径)

## 下一步开发建议

1. **动态障碍物回避**: 在轨迹规划器中加入障碍物检查
2. **多目标规划**: 一次规划多个前沿点
3. **优先级探索**: 根据信息增益选择最优前沿
4. **回环检测**: 避免重复探索已知区域
5. **协同探索**: 多无人机协同

## 测试命令集合

```bash
# 编译整个工作空间
cd ~/ros2_ws
colcon build
source install/setup.bash

# 启动完整系统
ros2 launch planner_pkg exploration_system.launch.py

# 监控系统
ros2 topic list
ros2 node list

# 查看日志
ros2 run rqt_console rqt_console

# 录制数据
ros2 bag record -a -o exploration_test

# 播放录制数据
ros2 bag play exploration_test
```
