# path_planner_pkg

**UAV 洞穴自主探索路径规划包**

本包为 TUM Autonomous Aerial Systems 2025 课程项目实现，提供基于 **A\* 搜索算法**和 **RRT 算法**的 UAV 洞穴自主探索路径规划功能。

---

## 目录结构

```
path_planner_pkg/
├── include/path_planner_pkg/
│   └── path_planner_node.hpp       # 所有类的声明
├── src/
│   ├── occupancy_map.cpp           # 3D 占用栅格地图实现
│   ├── astar_planner.cpp           # A* 路径规划算法
│   ├── rrt_explorer.cpp            # RRT 探索算法
│   └── path_planner_node.cpp       # ROS2 主节点
├── simulation/
│   └── path_planning_simulation.py # 独立 Python 仿真（无需 ROS2）
├── launch/
│   └── path_planner.launch.py      # ROS2 启动文件
├── config/
│   └── path_planner_params.yaml    # 参数配置文件
├── CMakeLists.txt
├── package.xml
└── README.md
```

---

## 系统架构

### 与现有系统的集成

```
Unity 仿真
    │
    ├─ current_state (Odometry) ──────────────────────────────┐
    ├─ /Quadrotor/Sensors/DepthCamera/point_cloud (PointCloud2)│
    │                                                          ▼
state_machine ──── statemachine/cmd (Command) ──► path_planner_node
    ▲                                                    │
    └──── statemachine/node_health (Answer) ◄────────────┤
                                                         │
controller_pkg ◄── command/trajectory (MultiDOFJointTrajectory)
    │
    ▼
Unity (rotor_speed_cmds)
```

### 话题接口

| 话题名称 | 消息类型 | 方向 | 说明 |
|---------|---------|------|------|
| `current_state_est` | `nav_msgs/Odometry` | 订阅 | UAV 位置和速度 |
| `/Quadrotor/Sensors/DepthCamera/point_cloud` | `sensor_msgs/PointCloud2` | 订阅 | 深度相机点云 |
| `statemachine/cmd` | `state_machine/Command` | 订阅 | 来自状态机的命令 |
| `command/trajectory` | `trajectory_msgs/MultiDOFJointTrajectory` | 发布 | 轨迹命令 |
| `statemachine/node_health` | `state_machine/Answer` | 发布 | 节点心跳 |
| `path_planner/path` | `nav_msgs/Path` | 发布 | 规划路径（可视化） |
| `path_planner/map` | `nav_msgs/OccupancyGrid` | 发布 | 2D 地图投影（可视化） |
| `path_planner/markers` | `visualization_msgs/MarkerArray` | 发布 | 航点标记（可视化） |

---

## 算法说明

### A\* 搜索算法

A\* 是一种**最优启发式搜索算法**，在 3D 占用栅格地图上寻找从起点到终点的最短无碰撞路径。

**核心特性：**
- **启发函数**：欧氏距离（admissible，保证最优性）
- **连通性**：26-连通（考虑对角移动）
- **碰撞检测**：机器人半径膨胀（inflate）
- **代价函数**：移动距离 + 未知区域惩罚（×1.5）
- **路径平滑**：去除冗余中间点（line-of-sight 检查）

**适用场景：** 已知地图或部分已知地图中的最优路径规划。

### RRT 算法

RRT (Rapidly-exploring Random Tree) 是一种**基于随机采样的探索算法**，通过快速构建随机树来探索未知空间。

**核心特性：**
- **Goal-biased 采样**：以 10% 概率直接采样目标点
- **步长控制**：每步延伸 1.5 米
- **碰撞检测**：路径段逐点检查
- **最大迭代**：8000 次

**适用场景：** 未知环境的快速探索，计算速度快（比 A\* 快约 20×）。

### 算法比较

| 指标 | A\* | RRT |
|------|-----|-----|
| 路径最优性 | **最优** | 次优 |
| 计算速度 | 较慢（~55ms） | **极快（~2.6ms）** |
| 路径平滑度 | **平滑** | 较曲折 |
| 内存占用 | 较高 | **较低** |
| 适用场景 | 已知地图精确规划 | 未知环境快速探索 |

---

## 仿真结果

### A\* 算法结果

- **总飞行距离**：89.2 米（比 RRT 短 21%）
- **发现灯笼**：4/4（100%）
- **平均规划时间**：54.2 ms
- **路径特点**：平滑、接近最优

### RRT 算法结果

- **总飞行距离**：112.7 米
- **发现灯笼**：4/4（100%）
- **平均规划时间**：2.6 ms（比 A\* 快 20×）
- **路径特点**：曲折，但计算极快

---

## 安装与使用

### 前置条件

```bash
# ROS2 Humble
sudo apt install ros-humble-desktop

# 依赖包
sudo apt install ros-humble-nav-msgs ros-humble-sensor-msgs \
    ros-humble-visualization-msgs ros-humble-trajectory-msgs \
    ros-humble-tf2-eigen libeigen3-dev
```

### 编译

将本包放置在 ROS2 工作空间的 `src` 目录下，与 `state_machine` 包并列：

```bash
# 工作空间结构
ros2_ws/src/
├── simulation/          # Unity 仿真桥接
├── controller_pkg/      # 位置控制器
├── state_machine/       # 任务状态机
├── mav_trajectory_generation/  # 轨迹生成
└── path_planner_pkg/    # 本包（路径规划）

# 编译
cd ros2_ws
colcon build --packages-select path_planner_pkg
source install/setup.bash
```

### 运行

```bash
# 方式1：使用 A* 算法（默认）
ros2 launch path_planner_pkg path_planner.launch.py

# 方式2：使用 RRT 算法
ros2 launch path_planner_pkg path_planner.launch.py use_astar:=false

# 方式3：直接运行节点
ros2 run path_planner_pkg path_planner_node \
    --ros-args -p use_astar:=true -p robot_radius:=0.8
```

### 独立 Python 仿真（无需 ROS2）

```bash
# 安装依赖
pip3 install numpy matplotlib

# 运行 A* 仿真
python3 simulation/path_planning_simulation.py --algorithm astar

# 运行 RRT 仿真
python3 simulation/path_planning_simulation.py --algorithm rrt

# 比较两种算法
python3 simulation/path_planning_simulation.py --algorithm compare
```

---

## 参数说明

| 参数名 | 默认值 | 说明 |
|--------|--------|------|
| `map_resolution` | 0.5 | 占用栅格分辨率（米/格） |
| `map_origin_x/y/z` | -500/-100/-10 | 地图原点（世界坐标） |
| `map_size_x/y/z` | 1200/400/100 | 地图尺寸（格子数） |
| `robot_radius` | 0.8 | 碰撞检测半径（米） |
| `max_speed` | 2.0 | 最大飞行速度（m/s） |
| `waypoint_reach_dist` | 2.0 | 到达航点距离阈值（米） |
| `explore_height` | 5.0 | 首选探索高度（米） |
| `use_astar` | true | 使用 A\*（true）或 RRT（false） |
| `node_name_in_sm` | "navigator" | 在 state_machine 中的节点名 |

---

## 状态机集成

本节点响应来自 `state_machine` 的以下命令：

| 命令 | 行为 |
|------|------|
| `START`（无目标） | 进入自主探索模式（frontier-based） |
| `START`（有目标） | 导航到指定坐标 |
| `HOLD` | 悬停在当前位置 |
| `LAND`（有目标） | 导航到降落点 |
| `ABORT` | 停止所有规划，进入 IDLE |

---

## 设计说明

### 3D 占用栅格地图

地图使用三值编码：
- `0`：自由空间（已探索且无障碍）
- `1`（`100`）：占用（障碍物）
- `-1`：未知（未探索）

点云更新采用**光线投射（Ray Casting）**：
- 终点标记为占用
- 沿射线的中间点标记为自由

### Frontier-Based 探索

Frontier 点定义为**自由空间与未知空间的边界格子**，UAV 优先飞向这些点以最大化地图覆盖率。

Frontier 选择策略：偏好距当前位置约 10 米、高度接近 `explore_height` 的点。
