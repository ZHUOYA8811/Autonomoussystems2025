from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    cloud_in = LaunchConfiguration("cloud_in")
    use_sim_time = LaunchConfiguration("use_sim_time")
    enable_tf_bridge = LaunchConfiguration("enable_tf_bridge")
    bridge_parent = LaunchConfiguration("bridge_parent")
    sensor_frame = LaunchConfiguration("sensor_frame")

    # 默认参数文件：mapping_pkg/config/octomap_params.yaml
    default_params = PathJoinSubstitution([
        FindPackageShare("mapping_pkg"),
        "config",
        "octomap_params.yaml",
    ])
    params_file = LaunchConfiguration("params_file")

    tf_bridge = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="depthcam_bridge_tf",
        output="screen",
        condition=IfCondition(enable_tf_bridge),
        arguments=[
            "0", "0", "0", "0", "0", "0",
            bridge_parent,
            sensor_frame,
        ],
    )

    octomap_node = Node(
        package="octomap_server",
        executable="octomap_server_node",
        name="octomap_server",
        output="screen",
        parameters=[
            params_file,
            {"use_sim_time": use_sim_time},
        ],
        remappings=[
            ("cloud_in", cloud_in),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("enable_tf_bridge", default_value="false"),
        DeclareLaunchArgument("bridge_parent", default_value="true_body"),
        DeclareLaunchArgument(
            "sensor_frame",
            default_value="Quadrotor/Sensors/DepthCamera",
        ),
        DeclareLaunchArgument(
            "cloud_in",
            default_value="/realsense/depth/points",
        ),
        DeclareLaunchArgument("params_file", default_value=default_params),
        tf_bridge,
        octomap_node,
    ])
