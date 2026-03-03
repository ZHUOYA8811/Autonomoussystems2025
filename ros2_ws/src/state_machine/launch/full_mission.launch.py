#!/usr/bin/env python3
"""
完整任务 Launch 文件
====================
启动无人机完整探索任务的所有必要组件：

1. Unity 仿真桥接（unity_ros + w_to_unity + state_estimate_corruptor + 静态TF）
2. 控制器（controller_node，带 current_state → current_state_est 重映射）
3. 感知管线（深度图→点云→OctoMap + 灯笼检测）
4. 路径规划（path_planner_node，自建3D地图 + A*/RRT + 直接输出轨迹）
5. 状态机（state_machine_node，协调整个任务流程）

任务流程：
  WAITING → TAKEOFF → TRAVELLING(飞往洞穴入口) → EXPLORING(自主探索) 
  → RETURN_HOME(检测到5个灯笼后返航) → LAND → DONE

使用方法：
  # 终端1：先启动 Unity 仿真器
  cd ~/Desktop/as\ project/autonomoussystems2025/ros2_ws/install/simulation/lib/simulation/
  ./Simulation.x86_64

  # 终端2：等 Unity 窗口出现后
  source install/setup.bash
  ros2 launch state_machine full_mission.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, 
    IncludeLaunchDescription, 
    TimerAction,
    LogInfo,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ============================================================
    # 启动参数
    # ============================================================
    takeoff_height_arg = DeclareLaunchArgument(
        'takeoff_height', default_value='5.0',
        description='起飞高度（米）')
    
    use_astar_arg = DeclareLaunchArgument(
        'use_astar', default_value='true',
        description='使用A*算法(true)或RRT算法(false)')

    # ============================================================
    # 1. 仿真环境（Unity桥接 + 控制器 + 状态估计 + TF）
    #    simulation.launch.py 已包含：
    #    - unity_ros（TCP桥接）
    #    - state_estimate_corruptor（发布 current_state_est）
    #    - w_to_unity（发送电机命令到Unity）
    #    - controller_node（带 current_state → current_state_est 重映射）
    #    - 所有静态TF变换
    # ============================================================
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("simulation"), "launch", "simulation.launch.py"
            ])
        ),
    )

    # ============================================================
    # 2. 感知管线（延迟3秒，等仿真连接稳定）
    #    - depth_image_proc：深度图 → 点云
    #    - octomap_server：点云 → OctoMap（供 planner_pkg 的 explorer 使用）
    # ============================================================
    perception_launch = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare("perception"), "launch", "perception.launch.py"
                    ])
                ),
            )
        ]
    )

    # ============================================================
    # 3. 灯笼检测（延迟4秒）
    #    light_detection_node 发布 /detected_points (PointStamped)
    #    → state_machine 订阅这个话题来计数灯笼
    # ============================================================
    light_detection_node = TimerAction(
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
    # 4. 路径规划节点（延迟5秒）
    #    path_planner_node：自建3D占用栅格 + A*/RRT
    #    - 订阅深度点云建图
    #    - 接收 state_machine 命令（导航/探索/悬停）
    #    - 直接输出 command/trajectory 给控制器
    #    - 心跳上报名称 = "navigator"
    #
    #    重映射：点云话题从 perception 管线的输出连接过来
    # ============================================================
    path_planner_node = TimerAction(
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
                    'explore_height': 15.0,   # 与洞口飞入高度一致，防止进洞后撞地
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
                remappings=[
                    # 关键重映射：perception 输出的点云话题 → path_planner 订阅的话题
                    ('/Quadrotor/Sensors/DepthCamera/point_cloud', '/camera/depth/points'),
                ],
            )
        ]
    )

    # ============================================================
    # 5. 状态机（延迟6秒，等其他节点都就绪）
    #    监控 controller 和 navigator 的心跳
    #    所有心跳收到后自动从 WAITING → TAKEOFF → ...
    # ============================================================
    state_machine_node = TimerAction(
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
    # 启动提示
    # ============================================================
    startup_info = LogInfo(
        msg="\n"
            "============================================================\n"
            "  完整探索任务启动中...\n"
            "  请确保 Unity 仿真器已经在运行！\n"
            "  \n"
            "  任务流程:\n"
            "    WAITING → TAKEOFF → TRAVELLING → EXPLORING\n"
            "    → RETURN_HOME (5个灯笼) → LAND → DONE\n"
            "============================================================\n"
    )

    return LaunchDescription([
        # 参数声明
        takeoff_height_arg,
        use_astar_arg,
        # 启动提示
        startup_info,
        # 按顺序启动各组件
        simulation_launch,          # 立即启动仿真桥接
        perception_launch,          # 3秒后启动感知
        light_detection_node,       # 4秒后启动灯笼检测
        path_planner_node,          # 5秒后启动路径规划
        state_machine_node,         # 6秒后启动状态机
    ])
