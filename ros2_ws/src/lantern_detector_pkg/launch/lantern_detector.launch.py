from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    params = PathJoinSubstitution([FindPackageShare("lantern_detector_pkg"), "config", "lantern_detector.yaml"])

    depthcam_alias_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="depthcam_alias_tf",
        arguments=["0","0","0","0","0","0",
                   "/Quadrotor/DepthCamera",
                   "Quadrotor/Sensors/DepthCamera"],
        output="screen",
    )

    detector = Node(
        package="lantern_detector_pkg",
        executable="lantern_detector_node",
        name="lantern_detector",
        output="screen",
        parameters=[params],
    )

    return LaunchDescription([depthcam_alias_tf, detector])