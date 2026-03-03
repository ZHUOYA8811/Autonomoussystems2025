from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """
    Complete launch file for exploration system:
    1. Frontier Explorer - detects frontiers and publishes exploration goals
    2. Trajectory Planner - generates smooth trajectories to frontier goals
    """
    
    # ========== Frontier Explorer Parameters ==========
    octomap_resolution_arg = DeclareLaunchArgument(
        'octomap_resolution',
        default_value='0.4',
        description='Resolution of the octomap'
    )
    
    movement_threshold_arg = DeclareLaunchArgument(
        'movement_threshold',
        default_value='3.0',
        description='Movement threshold in meters'
    )
    
    stall_time_threshold_arg = DeclareLaunchArgument(
        'stall_time_threshold',
        default_value='10.0',
        description='Stall time threshold in seconds'
    )
    
    max_distance_arg = DeclareLaunchArgument(
        'max_distance',
        default_value='30',
        description='Initial search distance for frontiers'
    )
    
    max_search_distance_arg = DeclareLaunchArgument(
        'max_search_distance',
        default_value='400',
        description='Maximum search distance when stalled'
    )
    
    exploration_rate_arg = DeclareLaunchArgument(
        'exploration_rate',
        default_value='0.3',
        description='Exploration update rate in Hz'
    )

    enforce_x_limit_arg = DeclareLaunchArgument(
        'enforce_x_limit',
        default_value='false',
        description='Whether to enforce max x limit for frontier detection/publishing'
    )

    x_limit_max_arg = DeclareLaunchArgument(
        'x_limit_max',
        default_value='-340.0',
        description='Maximum x allowed when enforce_x_limit is true'
    )

    # ========== Trajectory Planner Parameters ==========
    max_velocity_arg = DeclareLaunchArgument(
        'max_velocity',
        default_value='3.0',
        description='Maximum velocity for trajectory generation (m/s)'
    )
    
    max_acceleration_arg = DeclareLaunchArgument(
        'max_acceleration',
        default_value='2.0',
        description='Maximum acceleration for trajectory generation (m/s^2)'
    )

    # ========== Node Definitions ==========
    
    # Frontier Explorer Node
    explorer_node = Node(
        package='planner_pkg',
        executable='explorer_node',
        name='frontier_explorer',
        output='screen',
        parameters=[{
            'octomap_resolution': LaunchConfiguration('octomap_resolution'),
            'movement_threshold': LaunchConfiguration('movement_threshold'),
            'stall_time_threshold': LaunchConfiguration('stall_time_threshold'),
            'max_distance': LaunchConfiguration('max_distance'),
            'max_search_distance': LaunchConfiguration('max_search_distance'),
            'exploration_rate': LaunchConfiguration('exploration_rate'),
            'health_report_period': 1.0,
            'enforce_x_limit': LaunchConfiguration('enforce_x_limit'),
            'x_limit_max': LaunchConfiguration('x_limit_max'),
        }],
        remappings=[
            ('current_state_est', '/current_state_est'),
            ('octomap_full', '/octomap_full'),
            ('frontier_point', '/frontier_point'),
            ('frontier_point_marker', '/frontier_point_marker'),
            ('statemachine/node_health', '/statemachine/node_health'),
            ('statemachine/cmd', '/statemachine/cmd'),
        ]
    )

    # Trajectory Planner Node
    trajectory_planner_node = Node(
        package='planner_pkg',
        executable='trajectory_planner_node',
        name='trajectory_planner',
        output='screen',
        parameters=[{
            'max_velocity': LaunchConfiguration('max_velocity'),
            'max_acceleration': LaunchConfiguration('max_acceleration'),
            'health_report_period': 1.0,
        }],
        remappings=[
            ('frontier_point', '/frontier_point'),
            ('current_state_est', '/current_state_est'),
            ('path_segments_4D', '/path_segments_4D'),
            ('statemachine/node_health', '/statemachine/node_health'),
            ('statemachine/cmd', '/statemachine/cmd'),
        ]
    )

    return LaunchDescription([
        # Arguments
        octomap_resolution_arg,
        movement_threshold_arg,
        stall_time_threshold_arg,
        max_distance_arg,
        max_search_distance_arg,
        exploration_rate_arg,
        enforce_x_limit_arg,
        x_limit_max_arg,
        max_velocity_arg,
        max_acceleration_arg,
        
        # Nodes
        explorer_node,
        trajectory_planner_node,
    ])
