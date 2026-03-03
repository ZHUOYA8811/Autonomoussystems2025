# Perception Package

## Overview

The perception package implements the visual perception pipeline for the Autonomous Systems project.

It provides:

- Depth image → PointCloud2 conversion
- PointCloud2 → OctoMap 3D occupancy mapping
- Lantern target detection using semantic camera + depth
- 3D localization in world frame
- RViz visualization (PointCloud2, MarkerArray, Marker)

---

## System Architecture

### Semantic Detection Pipeline

~~~
/Quadrotor/Sensors/SemanticCamera/image_raw
                ↓
        light_detection_node
           ├── /detected_points        (PointStamped)
           └── /lantern_marker         (Marker)
~~~

### Mapping Pipeline

~~~
/realsense/depth/image
/realsense/depth/camera_info
                ↓
     depth_image_proc (point_cloud_xyz_node)
                ↓
        /camera/depth/points (PointCloud2)
                ↓
           octomap_server
           ├── /octomap_binary
           └── /occupied_cells_vis_array
~~~

---

## Dependencies (Docker)

### Start container

~~~bash
docker start -ai as_container
docker exec -it as_container bash
~~~

### Install required packages

~~~bash
apt update
apt install -y ros-jazzy-depth-image-proc
apt install -y ros-jazzy-octomap-server
apt install -y ros-jazzy-cv-bridge
apt install -y ros-jazzy-image-transport
apt install -y ros-jazzy-tf2-ros
apt install -y ros-jazzy-tf2-geometry-msgs
apt install -y ros-jazzy-vision-msgs
apt install -y python3-opencv
~~~

---

## Build

~~~bash
cd /workspace/autonomoussystems2025/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
~~~

---

## Run Instructions

### 1️⃣ Start Simulation

~~~bash
ros2 launch simulation simulation.launch.py
~~~

### 2️⃣ Start Mapping (PointCloud + Octomap)

~~~bash
ros2 launch perception perception.launch.py
~~~

This launches:

- static TF publisher
- point_cloud_xyz_node
- octomap_server

### 3️⃣ Start Lantern Detection

~~~bash
ros2 run perception light_detection_node
~~~

---

## Topics

### Input Topics

- /Quadrotor/Sensors/SemanticCamera/image_raw
- /realsense/depth/image
- /realsense/depth/camera_info

### Output Topics

- /camera/depth/points
- /octomap_binary
- /occupied_cells_vis_array
- /detected_points
- /lantern_marker

---

## RViz Visualization

Start RViz:

~~~bash
rviz2
~~~

Set:

- Fixed Frame → world

Add the following displays:

- PointCloud2 → /camera/depth/points
- MarkerArray → /occupied_cells_vis_array
- Marker → /lantern_marker

---

## Lantern Detection Algorithm

1. Convert semantic image (rgb8) to BGR8  
2. Apply color threshold using cv::inRange  
3. Target color ≈ (4, 235, 255)  
4. Morphological filtering  
5. Extract largest contour  
6. Compute centroid (u, v)  
7. Retrieve depth value (median filtering)  
8. Back-project to 3D:

~~~
x = (u - cx) * z / fx
y = (v - cy) * z / fy
~~~

9. Transform to world frame using TF  
10. Publish detected point and visualization marker  

---

## Notes

- Semantic and depth cameras are not perfectly aligned.
- For precise alignment, use depth registration (depth_image_proc register node).
- Current implementation assumes approximate alignment for project simplicity.