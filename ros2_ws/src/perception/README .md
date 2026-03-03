# Perception Package

## Overview

The `perception` package implements the visual perception pipeline for
the Autonomous Systems project.

It provides:

-   Depth image → PointCloud2 conversion\
-   PointCloud2 → OctoMap 3D occupancy mapping\
-   Lantern target detection using semantic camera + depth\
-   3D localization in world frame\
-   RViz visualization (PointCloud2, MarkerArray, Marker)

------------------------------------------------------------------------

## System Architecture

Semantic RGB (rgb8) │ ▼ Light Detection Node │ ├── /detected_points
(PointStamped) └── /lantern_marker (Marker)

Depth Image │ ▼ depth_image_proc (point_cloud_xyz_node) │ ▼
/camera/depth/points (PointCloud2) │ ▼ octomap_server │ ├──
/octomap_binary └── /occupied_cells_vis_array

------------------------------------------------------------------------

## Dependencies (Docker)

Start container:

docker start -ai as_container docker exec -it as_container bash

Install required packages:

apt update apt install -y ros-jazzy-depth-image-proc apt install -y
ros-jazzy-octomap-server apt install -y ros-jazzy-cv-bridge
ros-jazzy-image-transport ros-jazzy-tf2-ros ros-jazzy-tf2-geometry-msgs
ros-jazzy-vision-msgs python3-opencv

------------------------------------------------------------------------

## Build

cd /workspace/autonomoussystems2025/ros2_ws source
/opt/ros/jazzy/setup.bash colcon build source install/setup.bash

------------------------------------------------------------------------

## Run Instructions

### 1️⃣ Start Simulation

ros2 launch simulation simulation.launch.py

### 2️⃣ Start Mapping (PointCloud + Octomap)

ros2 launch perception perception.launch.py

This launches:

-   static TF publisher\
-   point_cloud_xyz_node\
-   octomap_server

### 3️⃣ Start Lantern Detection

ros2 run perception light_detection_node

Optional debug mode:

ros2 run perception light_detection_node --ros-args -p
publish_debug_mask:=true

------------------------------------------------------------------------

## Topics

### Input Topics

-   /Quadrotor/Sensors/SemanticCamera/image_raw\
-   /realsense/depth/image\
-   /realsense/depth/camera_info

### Output Topics

-   /camera/depth/points\
-   /octomap_binary\
-   /occupied_cells_vis_array\
-   /detected_points\
-   /lantern_marker

------------------------------------------------------------------------

## RViz Visualization

Start:

rviz2

Set Fixed Frame:

world

Add Displays:

-   PointCloud2 → /camera/depth/points\
-   MarkerArray → /occupied_cells_vis_array\
-   Marker → /lantern_marker

------------------------------------------------------------------------

## Lantern Detection Algorithm

1.  Convert semantic image (rgb8) to BGR8\

2.  Apply color threshold using cv::inRange\
    Target color ≈ (4, 235, 255)\

3.  Morphological filtering\

4.  Extract largest contour\

5.  Compute centroid (u, v)\

6.  Retrieve depth (median filtering)\

7.  Back-project to 3D:

    x = (u - cx) \* z / fx\
    y = (v - cy) \* z / fy

8.  Transform to world frame via TF\

9.  Publish detected point and visualization marker

------------------------------------------------------------------------

## Notes

-   Semantic and depth cameras are not perfectly aligned.\
-   For precise alignment, use depth registration (depth_image_proc
    register node).\
-   Current implementation assumes approximate alignment for project
    simplicity.

------------------------------------------------------------------------

Perception module for Autonomous Systems 2025.
