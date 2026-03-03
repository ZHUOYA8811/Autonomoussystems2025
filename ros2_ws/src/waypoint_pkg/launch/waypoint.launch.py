from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package share directory
    waypoint_pkg_share = get_package_share_directory('waypoint_pkg')
    
    # Config file
    config_file = os.path.join(waypoint_pkg_share, 'config', 'waypoint_params.yaml')
    
    # Declare arguments
    takeoff_height_arg = DeclareLaunchArgument(
        'takeoff_height',
        default_value='5.0',
        description='Takeoff height in meters'
    )
    
    cruise_speed_arg = DeclareLaunchArgument(
        'cruise_speed',
        default_value='2.0',
        description='Cruise speed in m/s'
    )
    
    # Waypoint node
    waypoint_node = Node(
        package='waypoint_pkg',
        executable='waypoint_node',
        name='waypoint_node',
        output='screen',
        parameters=[
            config_file,
            {
                'takeoff_height': LaunchConfiguration('takeoff_height'),
                'cruise_speed': LaunchConfiguration('cruise_speed'),
            }
        ],
        emulate_tty=True
    )
    
    return LaunchDescription([
        takeoff_height_arg,
        cruise_speed_arg,
        waypoint_node,
    ])
