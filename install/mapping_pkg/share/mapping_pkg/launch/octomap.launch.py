from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    cloud_in = LaunchConfiguration("cloud_in")

    # 默认参数文件：mapping_pkg/config/octomap_params.yaml
    default_params = PathJoinSubstitution([
        FindPackageShare("mapping_pkg"),
        "config",
        "octomap_params.yaml",
    ])
    params_file = LaunchConfiguration("params_file")

    octomap_node = Node(
        package="octomap_server",
        executable="octomap_server_node",
        name="octomap_server",
        output="screen",
        parameters=[params_file],
        remappings=[
            ("cloud_in", cloud_in),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("cloud_in", default_value="/realsense/depth/points"),
        DeclareLaunchArgument("params_file", default_value=default_params),
        octomap_node,
    ])