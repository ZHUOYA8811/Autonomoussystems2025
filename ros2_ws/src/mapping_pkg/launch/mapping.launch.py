import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory("mapping_pkg")

    pointcloud_launch = os.path.join(pkg_dir, "launch", "pointcloud.launch.py")
    octomap_launch = os.path.join(pkg_dir, "launch", "octomap.launch.py")

    cloud_in = LaunchConfiguration("cloud_in")
    cloud_out = LaunchConfiguration("cloud_out")
    enable_cloud_out_relay = LaunchConfiguration("enable_cloud_out_relay")
    tf_parent = LaunchConfiguration("tf_parent")
    tf_child = LaunchConfiguration("tf_child")

    enable_semantic = LaunchConfiguration("enable_semantic")
    sem_in = LaunchConfiguration("sem_in")
    sem_out = LaunchConfiguration("sem_out")

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="depth_camera_static_tf",
        output="screen",
        arguments=[
            "0", "0", "0", "0", "0", "0",
            tf_parent,
            tf_child,
        ],
    )

    semantic_node = Node(
        package="mapping_pkg",
        executable="semantic_camera_node",
        name="semantic_camera",
        output="screen",
        condition=IfCondition(enable_semantic),
        parameters=[{
            "in_topic": sem_in,
            "out_topic": sem_out,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "cloud_out",
            default_value="/realsense/depth/cloud_out",
        ),
        DeclareLaunchArgument(
            "cloud_in",
            default_value="/realsense/depth/points",
        ),
        DeclareLaunchArgument("enable_cloud_out_relay", default_value="false"),
        DeclareLaunchArgument(
            "tf_parent",
            default_value="Quadrotor/DepthCamera",
        ),
        DeclareLaunchArgument(
            "tf_child",
            default_value="Quadrotor/Sensors/DepthCamera",
        ),
        DeclareLaunchArgument("enable_semantic", default_value="false"),
        DeclareLaunchArgument(
            "sem_in",
            default_value="/Quadrotor/Sensors/SemanticCamera/image_raw",
        ),
        DeclareLaunchArgument(
            "sem_out",
            default_value="/realsense/semantic/image",
        ),

        static_tf,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(pointcloud_launch),
            launch_arguments={
                "cloud_out": cloud_out,
                "enable_cloud_out_relay": enable_cloud_out_relay,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(octomap_launch),
            launch_arguments={
                "cloud_in": cloud_in,
            }.items(),
        ),
        semantic_node,
    ])
