#!/usr/bin/env python3
"""
Test Flight Launch File
========================
A simplified launch file for testing Unity graphics and UAV flight.
This version allows easy switching between different Unity rendering modes.

Usage:
  # Default (no graphics argument - Unity auto-selects)
  ros2 launch simulation test_flight.launch.py

  # With Vulkan (better performance)
  ros2 launch simulation test_flight.launch.py unity_graphics:=vulkan

  # With OpenGL ES (compatibility mode)
  ros2 launch simulation test_flight.launch.py unity_graphics:=gles

  # With OpenGL Core (original)
  ros2 launch simulation test_flight.launch.py unity_graphics:=glcore
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    # Launch arguments
    unity_graphics = LaunchConfiguration("unity_graphics")
    corrupt_state_estimate = LaunchConfiguration("corrupt_state_estimate")
    use_waypoint = LaunchConfiguration("use_waypoint")
    use_takeoff = LaunchConfiguration("use_takeoff")

    declared_args = [
        DeclareLaunchArgument(
            "unity_graphics", 
            default_value="auto",
            description="Unity graphics mode: auto, glcore, vulkan, gles"
        ),
        DeclareLaunchArgument(
            "corrupt_state_estimate", 
            default_value="true",
            description="Add noise to state estimate"
        ),
        DeclareLaunchArgument(
            "use_waypoint", 
            default_value="false",
            description="Use waypoint navigation instead of takeoff"
        ),
        DeclareLaunchArgument(
            "use_takeoff", 
            default_value="true",
            description="Use simple takeoff node"
        ),
    ]

    # Include unity_ros.launch.py for all the Unity communication nodes
    unity_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("simulation"), "launch", "unity_ros.launch.py"])
        ),
    )

    # Unity executable - uses auto mode (no force graphics argument)
    # This lets Unity choose the best renderer for your system
    simulation_node = Node(
        package="simulation",
        executable="Simulation.x86_64",
        name="Simulation",
        output="screen",
        arguments=[],  # Auto mode - Unity selects best renderer
    )

    # State estimate corruptor (with noise)
    state_estimate_corruptor = Node(
        package="simulation",
        executable="state_estimate_corruptor_node",
        name="state_estimate_corruptor",
        output="screen",
        parameters=[
            {"drift_rw_factor": 0.03},
            {"pos_white_sig": 0.005},
            {"jump_seconds": 20.0},
        ],
        condition=IfCondition(corrupt_state_estimate),
    )

    # State estimate corruptor (without noise)
    state_estimate_corruptor_disabled = Node(
        package="simulation",
        executable="state_estimate_corruptor_node",
        name="state_estimate_corruptor",
        output="screen",
        parameters=[
            {"drift_rw_factor": 0.0},
            {"pos_white_sig": 0.0},
            {"jump_seconds": -1.0},
        ],
        condition=UnlessCondition(corrupt_state_estimate),
    )

    # Motor command to Unity bridge
    w_to_unity = Node(
        package="simulation",
        executable="w_to_unity",
        name="w_to_unity",
        output="screen",
        parameters=[
            {"ip_address": "127.0.0.1"},
            {"port": "12346"},
        ],
    )

    # Controller
    controller = Node(
        package="controller_pkg",
        executable="controller_node",
        name="controller_node",
        output="screen",
    )

    # Takeoff node (simple)
    takeoff_node = Node(
        package="takeoff_pkg",
        executable="takeoff_node",
        name="takeoff_node",
        output="screen",
        parameters=[{
            'takeoff_height': 5.0,
            'odom_topic': 'current_state_est',
            'trajectory_topic': 'command/trajectory',
        }],
        condition=IfCondition(use_takeoff),
    )

    # Waypoint node (mission)
    waypoint_node = Node(
        package="waypoint_pkg",
        executable="waypoint_node",
        name="waypoint_node",
        output="screen",
        parameters=[{
            'takeoff_height': 5.0,
            'cruise_speed': 2.0,
            'waypoint_threshold': 0.5,
            'odom_topic': 'current_state_est',
            'trajectory_topic': 'command/trajectory',
        }],
        condition=IfCondition(use_waypoint),
    )

    return LaunchDescription(
        declared_args
        + [
            unity_launch,
            simulation_node,
            state_estimate_corruptor,
            state_estimate_corruptor_disabled,
            w_to_unity,
            controller,
            takeoff_node,
            waypoint_node,
        ]
    )
