from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """
    Launch file for frontier exploration planner
    """
    
    # Declare launch arguments
    octomap_resolution_arg = DeclareLaunchArgument(
        'octomap_resolution',
        default_value='0.4',
        description='Resolution of the octomap'
    )
    
    movement_threshold_arg = DeclareLaunchArgument(
        'movement_threshold',
        default_value='3.0',
        description='Movement threshold in meters to consider robot as moving'
    )
    
    stall_time_threshold_arg = DeclareLaunchArgument(
        'stall_time_threshold',
        default_value='10.0',
        description='Time threshold in seconds to consider robot as stalled'
    )
    
    max_distance_arg = DeclareLaunchArgument(
        'max_distance',
        default_value='30',
        description='Initial maximum search distance for frontier detection in meters'
    )
    
    max_search_distance_arg = DeclareLaunchArgument(
        'max_search_distance',
        default_value='400',
        description='Maximum search distance when robot is stalled'
    )
    
    exploration_rate_arg = DeclareLaunchArgument(
        'exploration_rate',
        default_value='0.3',
        description='Exploration update rate in Hz'
    )

    # Explorer node
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

    return LaunchDescription([
        octomap_resolution_arg,
        movement_threshold_arg,
        stall_time_threshold_arg,
        max_distance_arg,
        max_search_distance_arg,
        exploration_rate_arg,
        explorer_node,
    ])
