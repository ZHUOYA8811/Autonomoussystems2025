#!/usr/bin/env python3
"""
Full System Launch File
-----------------------
Launches the complete UAV system:
1. Simulation (Unity + Controller)
2. Perception (Depth→PointCloud, OctoMap, Light Detection)
3. Navigation (Waypoint following)

Usage:
  ros2 launch simulation full_system.launch.py
  ros2 launch simulation full_system.launch.py takeoff_height:=3.0 cruise_speed:=1.5
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directories
    simulation_pkg = get_package_share_directory('simulation')
    
    # Declare arguments
    takeoff_height_arg = DeclareLaunchArgument(
        'takeoff_height',
        default_value='5.0',
        description='Takeoff height in meters'
    )
    
    cruise_speed_arg = DeclareLaunchArgument(
        'cruise_speed',
        default_value='2.0',
        description='Cruise speed in m/s'
    )
    
    enable_perception_arg = DeclareLaunchArgument(
        'enable_perception',
        default_value='true',
        description='Enable perception pipeline (depth→octomap, light detection)'
    )
    
    enable_waypoint_arg = DeclareLaunchArgument(
        'enable_waypoint',
        default_value='true',
        description='Enable waypoint navigation'
    )
    
    # ============================================================
    # 1. Simulation launch (includes Unity, controller, TF, etc.)
    # ============================================================
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(simulation_pkg, 'launch', 'simulation.launch.py')
        ),
    )
    
    # ============================================================
    # 2. Perception launch (delayed 3 seconds to wait for sim)
    # ============================================================
    perception_launch = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare("perception"), "launch", "perception.launch.py"])
                ),
            )
        ]
    )
    
    # Light detection node (delayed 4 seconds)
    light_detection_node = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='perception',
                executable='light_detection_node',
                name='light_detection_node',
                output='screen',
                emulate_tty=True
            )
        ]
    )
    
    # ============================================================
    # 3. Waypoint navigation node (delayed 5 seconds)
    # ============================================================
    waypoint_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='waypoint_pkg',
                executable='waypoint_node',
                name='waypoint_node',
                output='screen',
                parameters=[{
                    'takeoff_height': LaunchConfiguration('takeoff_height'),
                    'cruise_speed': LaunchConfiguration('cruise_speed'),
                    'waypoint_threshold': 0.5,
                    'odom_topic': 'current_state_est',
                    'trajectory_topic': 'command/trajectory',
                }],
                emulate_tty=True
            )
        ]
    )
    
    return LaunchDescription([
        # Arguments
        takeoff_height_arg,
        cruise_speed_arg,
        enable_perception_arg,
        enable_waypoint_arg,
        
        # Launch components
        simulation_launch,
        perception_launch,
        light_detection_node,
        waypoint_node,
    ])
