"""
System Bringup - One-Step Launch
================================
启动完整 UAV 洞穴探索任务的所有组件。

使用方法:
  # 终端1: 先启动 Unity
  cd ~/Desktop/as\ project/autonomoussystems2025/ros2_ws/install/simulation/lib/simulation/
  ./Simulation.x86_64

  # 终端2: 等 Unity 窗口出现后
  source install/setup.bash
  ros2 launch system_bringup bringup.launch.py
"""

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    TimerAction,
    LogInfo,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # ---- Launch arguments ----
    takeoff_height_arg = DeclareLaunchArgument(
        'takeoff_height', default_value='5.0',
        description='Takeoff height (meters)')

    use_astar_arg = DeclareLaunchArgument(
        'use_astar', default_value='true',
        description='Use A* (true) or RRT (false)')

    # ============================================================
    # 1. Simulation (Unity bridge + controller + TF)
    # ============================================================
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('simulation'),
                'launch', 'simulation.launch.py'
            )
        )
    )

    # ============================================================
    # 2. Perception pipeline (delay 3s)
    # ============================================================
    perception = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('perception'),
                        'launch', 'perception.launch.py'
                    )
                )
            )
        ]
    )

    # ============================================================
    # 3. Light/Lantern detection (delay 4s)
    # ============================================================
    light_detection = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='perception',
                executable='light_detection_node',
                name='light_detection_node',
                output='screen',
                emulate_tty=True,
            )
        ]
    )

    # ============================================================
    # 4. Path planner (delay 5s)
    # ============================================================
    planner = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='path_planner_pkg',
                executable='path_planner_node',
                name='path_planner',
                output='screen',
                emulate_tty=True,
                parameters=[{
                    'map_resolution': 0.5,
                    'robot_radius': 0.8,
                    'max_speed': 2.0,
                    'transit_speed': 5.0,
                    'waypoint_reach_dist': 2.0,
                    'replan_dist': 5.0,
                    'explore_height': 15.0,
                    'movement_threshold': 3.0,
                    'stall_time_threshold': 10.0,
                    'max_distance': 30,
                    'max_search_distance': 400,
                    'exploration_rate': 0.3,
                    'enforce_x_limit': False,
                    'x_limit_max': -700.0,
                    'use_astar': LaunchConfiguration('use_astar'),
                    'node_name_in_sm': 'navigator',
                }],
            )
        ]
    )

    # ============================================================
    # 5. State machine (delay 6s)
    # ============================================================
    state_machine = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='state_machine',
                executable='state_machine_node',
                name='state_machine_manager',
                output='screen',
                emulate_tty=True,
                parameters=[{
                    'monitored_node_list': ['controller', 'navigator'],
                    'takeoff_height': LaunchConfiguration('takeoff_height'),
                    'checkpoint_reach_dist_m': 2.0,
                }],
            )
        ]
    )

    # ============================================================
    startup_info = LogInfo(
        msg="\n"
            "============================================================\n"
            "  System Bringup: All nodes launching...\n"
            "  Make sure Unity Simulation is already running!\n"
            "  \n"
            "  Flow: WAITING -> TAKEOFF -> TRAVELLING -> EXPLORING\n"
            "        -> RETURN_HOME (5 lanterns) -> LAND -> DONE\n"
            "============================================================\n"
    )

    return LaunchDescription([
        takeoff_height_arg,
        use_astar_arg,
        startup_info,
        simulation,
        perception,
        light_detection,
        planner,
        state_machine,
    ])
