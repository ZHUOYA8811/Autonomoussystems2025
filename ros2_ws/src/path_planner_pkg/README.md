# path_planner_pkg

**UAV Cave Autonomous Exploration Path Planning Package**

This package is developed for the TUM Autonomous Aerial Systems 2025 course project. It provides UAV cave exploration path planning based on the **A\* search algorithm** and the **RRT algorithm**.


---

## Directory Structure

```
path_planner_pkg/
├── include/path_planner_pkg/
│   └── path_planner_node.hpp       # Class declarations
├── src/
│   ├── occupancy_map.cpp           # 3D occupancy grid map implementation
│   ├── astar_planner.cpp           # A* path planning algorithm
│   ├── rrt_explorer.cpp            # exploration algorithm
│   └── path_planner_node.cpp       # ROS2 main node
├── simulation/
│   └── path_planning_simulation.py # Standalone Python simulation
├── launch/
│   └── path_planner.launch.py      # ROS2 launch file
├── config/
│   └── path_planner_params.yaml    # Parameter configuration file
├── CMakeLists.txt
├── package.xml
└── README.md
```

---

## System Architecture

### Integration with Existing System

```
Unity Simulation
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

### Topic Interfaces


| Topic Name | Message Type | Direction | Description |
|------------|--------------|-----------|-------------|
| `current_state_est` | `nav_msgs/Odometry` | Subscribe | UAV position and velocity |
| `/Quadrotor/Sensors/DepthCamera/point_cloud` | `sensor_msgs/PointCloud2` | Subscribe | Depth camera point cloud |
| `statemachine/cmd` | `state_machine/Command` | Subscribe | Commands from state machine |
| `command/trajectory` | `trajectory_msgs/MultiDOFJointTrajectory` | Publish | Trajectory command |
| `statemachine/node_health` | `state_machine/Answer` | Publish | Node health |
| `path_planner/path` | `nav_msgs/Path` | Publish | Planned path (visualization) |
| `path_planner/map` | `nav_msgs/OccupancyGrid` | Publish | 2D projected map (visualization) |
| `path_planner/markers` | `visualization_msgs/MarkerArray` | Publish | Waypoint markers (visualization) |

---

## Algorithms

### A\* Search Algorithm

A\* is an **optimal heuristic search algorithm** that finds the shortest collision-free path in a 3D occupancy grid map.

**Key Features:**

- Heuristic function: Euclidean distance (admissible, guarantees optimality)  
- Connectivity: 26-connected (including diagonal moves)  
- Collision checking: Robot radius inflation  
- Cost function: Travel distance + unknown space penalty (×1.5)  
- Path smoothing: Line-of-sight based redundancy removal  

**Use Case:** Optimal planning in known or partially known environments.

---

### RRT Algorithm

RRT (Rapidly-exploring Random Tree) is a **sampling-based exploration algorithm** that rapidly builds a tree to explore unknown space.

**Key Features:**

- Goal-biased sampling (10%)  
- Step size: 1.5 meters  
- Incremental collision checking  
- Maximum 8000 iterations  

**Use Case:** Fast exploration in unknown environments (≈20× faster than A\*).

---

### Algorithm Comparison

| Metric | A\* | RRT |
|--------|-----|-----|
| Path Optimality | Optimal | Suboptimal |
| Computation Speed | ~55 ms | ~2.6 ms |
| Path Smoothness | Smooth | Irregular |
| Memory Usage | Higher | Lower |
| Suitable Scenario | Known map | Unknown exploration |

---

## Simulation Results

### A\*

- Total flight distance: 89.2 m  
- Lanterns detected: 4/4 (100%)  
- Average planning time: 54.2 ms  
- Smooth and near-optimal path  

### RRT

- Total flight distance: 112.7 m  
- Lanterns detected: 4/4 (100%)  
- Average planning time: 2.6 ms  
- Fast but more irregular path  

---

## Installation and Usage

### Prerequisites

```bash
sudo apt install ros-humble-desktop
sudo apt install ros-humble-nav-msgs ros-humble-sensor-msgs \
    ros-humble-visualization-msgs ros-humble-trajectory-msgs \
    ros-humble-tf2-eigen libeigen3-dev
``````



### Build

Workspace structure:

```bash
ros2_ws/src/
├── simulation/          
├── controller_pkg/     
├── state_machine/       
├── mav_trajectory_generation/  
└── path_planner_pkg/    
``````

# Build:
```bash
cd ros2_ws
colcon build --packages-select path_planner_pkg
source install/setup.bash
``````

### Run

```bash
# A* (default)
ros2 launch path_planner_pkg path_planner.launch.py

#  RRT 
ros2 launch path_planner_pkg path_planner.launch.py use_astar:=false

# Run directly
ros2 run path_planner_pkg path_planner_node \
    --ros-args -p use_astar:=true -p robot_radius:=0.8
```

### Standalone Python Simulation

```bash
pip3 install numpy matplotlib

python3 simulation/path_planning_simulation.py --algorithm astar

python3 simulation/path_planning_simulation.py --algorithm rrt

python3 simulation/path_planning_simulation.py --algorithm compare
```

---

## Parameters

| Parameters | Default | Description |
|--------|--------|------|
| `map_resolution` | 0.5 | Grid resolution (m/cell) |
| `map_origin_x/y/z` | -500/-100/-10 | Map origin |
| `map_size_x/y/z` | 1200/400/100 | Map size (cells) |
| `robot_radius` | 0.8 | Collision radius (m) |
| `max_speed` | 2.0 | Max flight speed (m/s) |
| `waypoint_reach_dist` | 2.0 | Waypoint reach threshold (m) |
| `explore_height` | 5.0 | Preferred exploration height |
| `use_astar` | true | A* or RRT |
| `node_name_in_sm` | "navigator" | Name in state_machine |

---

## State Machine Integration


| Command | Behavior |
|------|------|
| `START`(no target) | Autonomous frontier exploration |
| `START`(with target) | Navigate to coordinate |
| `HOLD` | Hover |
| `LAND`(with target) | Navigate to landing point |
| `ABORT` | Stop planning and switch to IDLE |

---

## Design Details

### 3D Occupancy Grid

State encoding:
- `0`：Free
- `1`: Occupied
- `-1`：Unknown

**Ray casting update:**：
- Endpoints → Occupied
- Intermediate cells → Free

### Frontier-Based Exploration

A frontier cell is the boundary between free and unknown space.

The UAV prefers frontiers: around 10 meters away, close to explore_height
