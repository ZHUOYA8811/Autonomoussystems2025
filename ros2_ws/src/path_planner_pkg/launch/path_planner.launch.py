"""
Path Planner Launch File
========================
启动UAV路径规划节点，用于洞穴自主探索任务。

支持两种规划算法：
- A* (use_astar=True): 在已知占用栅格地图上进行最优路径搜索
- RRT (use_astar=False): 用于未知环境的快速探索

使用方法：
  ros2 launch path_planner_pkg path_planner.launch.py
  ros2 launch path_planner_pkg path_planner.launch.py use_astar:=false
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ---- 声明启动参数 ----
    use_astar_arg = DeclareLaunchArgument(
        'use_astar',
        default_value='true',
        description='Use A* algorithm (true) or RRT (false) for path planning'
    )

    map_resolution_arg = DeclareLaunchArgument(
        'map_resolution',
        default_value='0.5',
        description='Occupancy map resolution in meters'
    )

    robot_radius_arg = DeclareLaunchArgument(
        'robot_radius',
        default_value='0.8',
        description='Robot collision radius in meters'
    )

    max_speed_arg = DeclareLaunchArgument(
        'max_speed',
        default_value='2.0',
        description='Exploration mode UAV speed in m/s'
    )

    transit_speed_arg = DeclareLaunchArgument(
        'transit_speed',
        default_value='5.0',
        description='Transit/navigation mode UAV speed in m/s'
    )

    explore_height_arg = DeclareLaunchArgument(
        'explore_height',
        default_value='5.0',
        description='Preferred exploration height in meters'
    )

    # ---- 路径规划节点 ----
    path_planner_node = Node(
        package='path_planner_pkg',
        executable='path_planner_node',
        name='path_planner',
        output='screen',
        parameters=[{
            # 地图分辨率（地图范围由代码自动计算）
            'map_resolution': LaunchConfiguration('map_resolution'),

            # 机器人参数
            'robot_radius': LaunchConfiguration('robot_radius'),
            'max_speed': LaunchConfiguration('max_speed'),
            'transit_speed': LaunchConfiguration('transit_speed'),
            'waypoint_reach_dist': 2.0,
            'replan_dist': 5.0,
            'explore_height': LaunchConfiguration('explore_height'),

            # 探索策略参数
            'movement_threshold': 3.0,
            'stall_time_threshold': 10.0,
            'max_distance': 30,
            'max_search_distance': 400,
            'exploration_rate': 0.3,
            'enforce_x_limit': False,
            'x_limit_max': -700.0,

            # 算法选择
            'use_astar': LaunchConfiguration('use_astar'),

            # state_machine中的节点名称
            'node_name_in_sm': 'navigator',
        }],
        remappings=[
            # 话题重映射（如需要）
            # ('current_state_est', '/current_state_est'),
            # ('command/trajectory', '/command/trajectory'),
        ]
    )

    return LaunchDescription([
        use_astar_arg,
        map_resolution_arg,
        robot_radius_arg,
        max_speed_arg,
        transit_speed_arg,
        explore_height_arg,
        path_planner_node,
    ])
