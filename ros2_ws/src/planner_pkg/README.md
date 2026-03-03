# Planner Package - Frontier Exploration

## 🎯 Overview

The `planner_pkg` implements **Frontier-based autonomous exploration** for the drone system. It consists of two main components:

1. **Frontier Explorer** (`explorer_node`): Detects and clusters frontier points from octomap
2. **Trajectory Planner** (`trajectory_planner_node`): Generates smooth polynomial trajectories to exploration goals

This package is fully integrated with the existing state machine, controller, and perception systems.

## 📦 Package Contents

## 📦 Package Contents

### Nodes

1. **explorer_node** - Frontier detection and clustering
2. **trajectory_planner_node** - Polynomial trajectory generation

### Headers

- `Optics.hpp` - OPTICS clustering algorithm implementation
- `explorer_node.hpp` - Frontier explorer interface
- `simple_trajectory_planner.hpp` - Trajectory planner interface

### Launch Files

- `planner.launch.py` - Launch explorer only
- `exploration_system.launch.py` - Launch complete exploration system (explorer + planner)

## 🚀 Quick Start

### Build
```bash
cd ~/ros2_ws
colcon build --packages-select planner_pkg
source install/setup.bash
```

### Launch Complete System
```bash
# Launch exploration system (both nodes)
ros2 launch planner_pkg exploration_system.launch.py
```

### Manual Control
```bash
# Start exploration
ros2 topic pub --once /statemachine/cmd state_machine/msg/Command \
  "Node: `trajectory_planner_node`

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `frontier_point` | `geometry_msgs/msg/Point` | Exploration goal from explorer |
| `current_state_est` | `nav_msgs/msg/Odometry` | Current drone state |
| `statemachine/cmd` | `state_machine/msg/Command` | Commands from state machine |

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `path_segments_4D` | `mav_planning_msgs/msg/PolynomialTrajectory4D` | Polynomial trajectory segments |
| `statemachine/node_health` | `state_machine/msg/Answer` | Node health status |

### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `max_velocity` | double | 3.0 | Maximum velocity for trajectory (m/s) |
| `max_acceleration` | double | 2.0 | Maximum acceleration for trajectory (m/s²) |

---

## {target: 'planner', command: 2}"

# Stop exploration
ros2 topic pub --once /statemachine/cmd state_machine/msg/Command \
  "{target: 'planner', command: 3}"
```

## Features

- **Frontier Detection**: Identifies boundary points between known and unknown regions in the octomap
- **OPTICS Clustering**: Groups frontier points into meaningful clusters
- **Dynamic Search Range**: Adapts search distance based on robot movement (30m → 400m when stalled)
- **State Machine Integration**: Responds to START/HOLD/ABORT commands from the state machine
- **Health Monitoring**: Reports node status to the state machine

## Node: `explorer_node`

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `current_state_est` | `nav_msgs/msg/Odometry` | Current drone position |
| `octomap_full` | `octomap_msgs/msg/Octomap` | Full 3D occupancy map |
| `statemachine/cmd` | `state_machine/msg/Command` | Commands from state machine |

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `frontier_point` | `geometry_msgs/msg/Point` | Next exploration goal point |
| `frontier_point_marker` | `visualization_msgs/msg/Marker` | RViz visualization of goal |
| `statemachine/node_health` | `state_machine/msg/Answer` | Node health status |

### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `octomap_resolution` | double | 0.4 | Resolution of octomap in meters |
| `movement_threshold` | double | 3.0 | Distance threshold to detect movement (m) |
| `stall_time_threshold` | double | 10.0 | Time threshold to detect stall (s) |
| `max_distance` | int | 30 | Initial search radius (m) |
| `max_search_distance` | int | 400 | Maximum search radius when stalled (m) |
| `exploration_rate` | double | 0.3 | Exploration update frequency (Hz) |

## Integration with State Machine

The planner responds to commands from `state_machine`:
- **START (command=2)**: Begin exploration
- **HOLD (command=3)**: Pause exploration
- **ABORT (command=6)**: Stop exploration

The state machine sends `START` command when entering `EXPLORING` state.

## Algorithm

1. **Frontier Detection**:
   - Scan octomap in a bounding box around the drone
   - For each free voxel, check its 26 neighbors
   - If any neighbor is unknown → mark as frontier point

2. **Clustering**:
   - Use complete exploration system (recommended)
ros2 launch planner_pkg exploration_system.launch.py

# Launch explorer only
ros2 launch planner_pkg planner.launch.py

# With custom parameters
ros2 launch planner_pkg exploration_system.launch.py \
  max_distance:=50 \
  exploration_rate:=0.5 \
  max_velocity:=4.0
```

### System Integration

See [`INTEGRATION.md`](INTEGRATION.md) for complete system integration guide including:
- Full system architecture
- Launch sequences
- Topic communication flow
- Troubleshooting guide- Calculate centroid as exploration goal
   - Only publish goals with x ≤ -340 (mission boundary)

4. **Adaptive Search**:
   - Start with 30m search radius
   - If robot stalls for >10s in x < -600 region → expand to 400m
   - Reset to 30m when robot starts moving again

## Usage

### Build

```bash
cd ~/ros2_ws
colcon build --packages-select planner_pkg
source install/setup.bash
```

### Launch

```bash
# Launch planner node
ros2 launch planner_pkg planner.launch.py

# With custom parameters
ros2 launch planner_pkg planner.launch.py max_distance:=50 exploration_rate:=0.5
```

### Manual Testing

```bash
# Start exploration
ros2 topic pub --once /statemachine/cmd state_machine/msg/Command "{target: 'planner', command: 2}"

# Stop exploration
ros2 topic pub --once /statemachine/cmd state_machine/msg/Command "{target: 'planner', command: 3}"

# View frontier goal
ros2 topic echo /frontier_point
```

## Dependencies

- **ROS2 Jazzy** (or compatible)
- **PCL** (Point Cloud Library)
- **Octomap** & **octomap_msgs**
- **state_machine** package
- **perception** package (for octomap generation)

## RViz Visualization

Add these to RViz:
- **Marker** → Topic: `/frontier_point_marker` (magenta sphere showing exploration goal)
- **OccupancyGrid** → Topic: `/octomap_full` (3D occupancy map)

## Notes

- The planner requires a valid octomap to function
- Ensure `perception` package is running to generate octomap
- Goal points are only published if x ≤ -340 (inside mission area)
- Health reports are sent every 1 second to state machine

## Troubleshooting

**No frontier points detected:**
- Check if octomap is being published: `ros2 topic echo /octomap_full`
- Verify drone position is being received: `ros2 topic echo /current_state_est`
- Increase `max_distance` parameter

**Planner not starting:**
- Ensure state machine sends START command: check `/statemachine/cmd`
- Verify planner is receiving command: `ros2 topic echo /statemachine/cmd`

**Goal not being followed:**
- Check if `sampler` or trajectory generator is subscribed to `/frontier_point`
- Verify goal is valid (x ≤ -340)
