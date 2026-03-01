from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directories
    takeoff_pkg_share = get_package_share_directory('takeoff_pkg')
    controller_pkg_share = get_package_share_directory('controller_pkg')

    # Config files
    takeoff_config = os.path.join(takeoff_pkg_share, 'config', 'takeoff_params.yaml')
    controller_config = os.path.join(controller_pkg_share, 'config', 'controller_params.yaml')

    # Declare launch arguments
    takeoff_height_arg = DeclareLaunchArgument(
        'takeoff_height',
        default_value='5.0',
        description='Takeoff height in meters'
    )

    # Include simulation launch file
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("simulation"), "launch", "simulation.launch.py"])
        ),
    )

    # Controller node
    controller_node = Node(
        package='controller_pkg',
        executable='controller_node',
        name='controller_node',
        output='screen',
        parameters=[controller_config],
        remappings=[
            ('current_state', 'current_state_est'),
        ],
        emulate_tty=True
    )

    # Takeoff node
    takeoff_node = Node(
        package='takeoff_pkg',
        executable='takeoff_node',
        name='takeoff_node',
        output='screen',
        parameters=[takeoff_config],
        emulate_tty=True
    )

    return LaunchDescription([
        takeoff_height_arg,
        simulation_launch,  # Start simulation first
        controller_node,
        takeoff_node,
    ])
