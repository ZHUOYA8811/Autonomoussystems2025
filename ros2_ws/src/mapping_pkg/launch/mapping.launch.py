import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory("mapping_pkg")

    pointcloud_launch = os.path.join(pkg_dir, "launch", "pointcloud.launch.py")
    octomap_launch = os.path.join(pkg_dir, "launch", "octomap.launch.py")

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="depthcam_alias_tf",
        arguments=[
            "0", "0", "0", "0", "0", "0",
            "depth_camera",
            "Quadrotor/Sensors/DepthCamera"
        ],
        output="screen",
    )

    return LaunchDescription([
        static_tf,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(pointcloud_launch),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(octomap_launch),
        ),
    ])