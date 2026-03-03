# AutonomousSystems2025 — Group 05

## SubTerrain Challenge

![Schematics of the Drone](DroneforCave.png)

Autonomous drone exploration of a cave environment. The drone takes off, flies to a target region, then autonomously explores using frontier-based exploration while detecting lanterns.

---

## Architecture

```
Unity (Simulation)
  └─ /realsense/depth/image  ──► perception.launch.py
                                    ├─ depth_to_points  ──► /camera/depth/points
                                    └─ octomap_server   ──► /octomap_full

/octomap_full ──► planner.launch.py
                    ├─ explorer_node      (frontier detection → /frontier_point)
                    └─ trajectory_planner (/frontier_point → /path_segments_4D)

/path_segments_4D ──► waypoint_mission.launch.py
                         └─ sampler  ──► /command/trajectory

/command/trajectory ──► simulation.launch.py
                           └─ controller_node  ──► /rotor_speed_cmds

state_machine.launch.py  ──► orchestrates all of the above via /statemachine/cmd
```

**Mission states:** `WAITING → TAKEOFF → TRAVELLING → EXPLORING → RETURN_HOME → LAND → DONE`

---

## Environment Setup

### Docker (recommended)

```bash
cd ~/AS/autonomous_system-group05
./as3.sh          # enters jazzy-as3 container
```

Inside the container, the workspace is at `/workspace`. Source before running:

```bash
source /workspace/install/setup.bash
```

### Build (only needed after source changes)

```bash
cd /workspace/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

Build a single package:

```bash
colcon build --packages-select <pkg_name>
```

---

## Launch Sequence


The entire UAV system is integrated into a unified entry point within the system_bringup package. This allows for a stable, single-command startup of the simulation, perception, planning, and control modules.

```bash
ros2 launch system_bringup bringup.launch.py
```


---

## Key Topics

| Topic | Type | Description |
|---|---|---|
| `/statemachine/cmd` | `state_machine/msg/Command` | State machine → nodes commands |
| `/statemachine/node_health` | `state_machine/msg/Answer` | Node heartbeats → state machine |
| `/statemachine/status` | `std_msgs/msg/String` | Current mission state string |
| `/frontier_point` | `geometry_msgs/msg/Point` | Explorer goal output |
| `/path_segments_4D` | `mav_planning_msgs/msg/PolynomialTrajectory4D` | Planned trajectory |
| `/command/trajectory` | `trajectory_msgs/msg/MultiDOFJointTrajectory` | Sampler → controller |
| `/octomap_full` | `octomap_msgs/msg/Octomap` | Full 3D map |
| `/octomap_point_cloud_centers` | `sensor_msgs/msg/PointCloud2` | Map visualization |

---

## Monitoring

```bash
# Check heartbeats (should see both controller and sampler)
ros2 topic echo /statemachine/node_health

# Check mission state
ros2 topic echo /statemachine/status

# Check frontier detection rate
ros2 topic hz /frontier_point

# Check octomap update rate
ros2 topic hz /octomap_full
```

---

## Visualize in Foxglove

1. Open Foxglove Studio and connect to `ws://localhost:8765`
2. Add a **3D panel**, set **Fixed frame** to `world`
3. Useful topics to add:
   - `/octomap_point_cloud_centers` — occupied voxels (point cloud)
   - `/frontier_point_marker` — current exploration goal (sphere)
   - `/occupied_cells_vis_array` — 3D voxel markers

---

## Packages

| Package | Description |
|---|---|
| `simulation` | Unity bridge, controller node, state estimator |
| `perception` | depth→pointcloud→octomap pipeline |
| `basic_waypoint_pkg` | Polynomial trajectory planner + trajectory sampler |
| `planner_pkg` | Frontier explorer + simple trajectory planner |
| `state_machine` | Mission state machine |
| `mav_msgs` / `mav_planning_msgs` / `mav_trajectory_generation` | Third-party MAV libraries |
