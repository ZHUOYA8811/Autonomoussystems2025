from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
import os

def generate_launch_description():
    """
    Complete autonomous exploration system launcher
    
    Launch sequence:
    1. Simulation (optional, if not already running)
    2. Perception (octomap generation)
    3. Controller
    4. Trajectory sampler
    5. Exploration system (frontier detection + trajectory planning)
    6. State machine (mission coordinator)
    """
    
    # ========== Perception System ==========
    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('perception'), '/launch/perception.launch.py'
        ])
    )
    
    # ========== Controller ==========
    # Assuming controller has a launch file, adjust if using different launch method
    controller_node = Node(
        package='controller_pkg',
        executable='controller_node',
        name='controller_node',
        output='screen'
    )
    
    # ========== Trajectory Sampler ==========
    sampler_node = Node(
        package='mav_trajectory_generation',
        executable='trajectory_sampler_node',
        name='trajectory_sampler',
        output='screen',
        parameters=[{
            'publish_whole_trajectory': False,
            'dt': 0.01,
            'health_report_period_sec': 1.0
        }]
    )
    
    # ========== Exploration System ==========
    exploration_launch = TimerAction(
        period=2.0,  # Wait 2 seconds for other nodes to initialize
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    FindPackageShare('planner_pkg'), '/launch/exploration_system.launch.py'
                ])
            )
        ]
    )
    
    # ========== State Machine (Mission Coordinator) ==========
    state_machine_launch = TimerAction(
        period=3.0,  # Wait 3 seconds to ensure all other nodes are ready
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    FindPackageShare('state_machine'), '/launch/state_machine.launch.py'
                ])
            )
        ]
    )
    
    return LaunchDescription([
        # Launch in sequence with delays
        perception_launch,
        controller_node,
        sampler_node,
        exploration_launch,
        state_machine_launch,
    ])
