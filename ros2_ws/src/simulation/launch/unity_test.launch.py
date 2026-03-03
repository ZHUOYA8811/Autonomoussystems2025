#!/usr/bin/env python3
"""
Minimal Test Launch - Unity Only
==================================
Only starts Unity simulation for graphics testing.

Usage:
  ros2 launch simulation unity_test.launch.py
  
Graphics modes:
  ros2 launch simulation unity_test.launch.py graphics:=auto     # Default
  ros2 launch simulation unity_test.launch.py graphics:=vulkan   # Vulkan
  ros2 launch simulation unity_test.launch.py graphics:=gles     # OpenGL ES
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    graphics_arg = DeclareLaunchArgument(
        'graphics',
        default_value='auto',
        description='Graphics mode: auto, vulkan, gles, glcore'
    )

    # Just Unity executable - no arguments for auto mode
    simulation_node = Node(
        package="simulation",
        executable="Simulation.x86_64",
        name="Simulation",
        output="screen",
        arguments=[],
    )

    return LaunchDescription([
        graphics_arg,
        simulation_node,
    ])
