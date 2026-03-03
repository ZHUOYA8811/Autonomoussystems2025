from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    takeoff_pkg_share = get_package_share_directory('takeoff_pkg')
    takeoff_config = os.path.join(takeoff_pkg_share, 'config', 'takeoff_params.yaml')

    takeoff_height_arg = DeclareLaunchArgument(
        'takeoff_height',
        default_value='5.0',
        description='Takeoff height in meters'
    )

    # 关键：让你可以从外面指定 odom 来源，默认用 /current_state_est
    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic',
        default_value='/current_state_est',
        description='Odometry topic to use as current_state for takeoff_node'
    )

    takeoff_node = Node(
        package='takeoff_pkg',
        executable='takeoff_node',
        name='takeoff_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            takeoff_config,
            {'takeoff_height': LaunchConfiguration('takeoff_height')},
        ],
        remappings=[
            ('current_state', LaunchConfiguration('odom_topic')),
        ],
    )

    return LaunchDescription([
        takeoff_height_arg,
        odom_topic_arg,
        takeoff_node,
    ])