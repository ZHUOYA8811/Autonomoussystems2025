from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
import os

def generate_launch_description():
    """
    Complete autonomous exploration system launcher
    
    Launch sequence:
    0. Simulation (optional - Unity must be started separately!)
    1. Perception (octomap generation)
    2. Controller
    3. Trajectory sampler
    4. Exploration system (frontier detection + trajectory planning)
    5. State machine (mission coordinator)
    
    Usage:
        # Without simulation (Unity already running manually):
        ros2 launch state_machine full_system.launch.py
        
        # With simulation launch (still need to start Unity manually):
        ros2 launch state_machine full_system.launch.py launch_simulation:=true
    """
    
    # Declare launch arguments
    launch_simulation_arg = DeclareLaunchArgument(
        'launch_simulation',
        default_value='false',
        description='Launch simulation nodes (Unity must still be started manually)'
    )
    
    # ========== Simulation (Optional) ==========
    # NOTE: Unity executable must be started MANUALLY before running this!
    # This only starts the ROS2 bridge nodes (unity_ros, w_to_unity, etc.)
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('simulation'), '/launch/simulation.launch.py'
        ]),
        condition=IfCondition(LaunchConfiguration('launch_simulation'))
    )
    
    # ========== Perception System (Delayed Start) ==========
    # Wait for simulation/Unity to be ready
    perception_launch = TimerAction(
        period=3.0,  # Wait 3 seconds for Unity connection
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    FindPackageShare('perception'), '/launch/perception.launch.py'
                ])
            )
        ]
    )
    
    # ========== Controller (Delayed Start) ==========
    controller_node = TimerAction(
        period=4.0,  # Wait for perception to initialize
        actions=[
            Node(
                package='controller_pkg',
                executable='controller_node',
                name='controller_node',
                output='screen'
            )
        ]
    )
    
    # ========== Trajectory Sampler (Delayed Start) ==========
    sampler_node = TimerAction(
        period=5.0,  # Wait for controller
        actions=[
            Node(
                package='mav_trajectory_generation',
                executable='trajectory_sampler_node',
                name='trajectory_sampler',
                output='screen',
                parameters=[{
                    'publish_whole_trajectory': False,
                    'dt': 0.01,
                    'health_report_period_sec': 1.0,
                    'max_velocity': 2.0,
                    'max_acceleration': 2.0
                }]
            )
        ]
    )
    
    # ========== Exploration System (Delayed Start) ==========
    exploration_launch = TimerAction(
        period=6.0,  # Wait 6 seconds for sampler to initialize
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    FindPackageShare('planner_pkg'), '/launch/planner.launch.py'
                ])
            )
        ]
    )
    
    # ========== State Machine (Mission Coordinator) ==========
    state_machine_launch = TimerAction(
        period=8.0,  # Wait 8 seconds to ensure all other nodes are ready
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    FindPackageShare('state_machine'), '/launch/state_machine.launch.py'
                ])
            )
        ]
    )
    
    return LaunchDescription([
        # Launch arguments
        launch_simulation_arg,
        
        # Launch in sequence with delays
        simulation_launch,  # Optional, controlled by launch_simulation arg
        perception_launch,
        controller_node,
        sampler_node,
        exploration_launch,
        state_machine_launch,
    ])
