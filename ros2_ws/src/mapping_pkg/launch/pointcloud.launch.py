from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.substitutions import PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_voxel = LaunchConfiguration("use_voxel")
    enable_cloud_out_relay = LaunchConfiguration("enable_cloud_out_relay")
    points_topic = LaunchConfiguration("points_topic")
    points_voxel_topic = LaunchConfiguration("points_voxel_topic")
    cloud_out = LaunchConfiguration("cloud_out")

    # 默认参数文件：mapping_pkg/config/voxel_params.yaml
    default_voxel_params = PathJoinSubstitution([
        FindPackageShare("mapping_pkg"),
        "config",
        "voxel_params.yaml",
    ])
    voxel_params = LaunchConfiguration("voxel_params")

    # 1) depth image + camera_info -> PointCloud2
    pc_node = Node(
        package="depth_image_proc",
        executable="point_cloud_xyz_node",
        name="point_cloud_xyz_node",
        output="screen",
        remappings=[
            ("image_rect", "/realsense/depth/image"),
            ("camera_info", "/realsense/depth/camera_info"),
            ("points", points_topic),
        ],
    )

    # 2) voxel filter（可选）
    voxel_node = Node(
        package="pcl_ros",
        executable="filter_voxel_grid_node",
        name="voxel_grid",
        output="screen",
        remappings=[
            ("input", points_topic),
            ("output", points_voxel_topic),
        ],
        parameters=[voxel_params],
        condition=IfCondition(use_voxel),
    )

    relay_voxel = Node(
        package="topic_tools",
        executable="relay",
        name="relay_voxel_to_out",
        output="screen",
        arguments=[points_voxel_topic, cloud_out],
        condition=IfCondition(PythonExpression([
            "'",
            enable_cloud_out_relay,
            "'",
            " == 'true' and ",
            "'",
            use_voxel,
            "'",
            " == 'true'",
        ])),
    )

    relay_raw = Node(
        package="topic_tools",
        executable="relay",
        name="relay_raw_to_out",
        output="screen",
        arguments=[points_topic, cloud_out],
        condition=IfCondition(PythonExpression([
            "'",
            enable_cloud_out_relay,
            "'",
            " == 'true' and ",
            "'",
            use_voxel,
            "'",
            " == 'false'",
        ])),
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_voxel", default_value="false"),
        DeclareLaunchArgument("enable_cloud_out_relay", default_value="false"),
        DeclareLaunchArgument(
            "points_topic",
            default_value="/realsense/depth/points",
        ),
        DeclareLaunchArgument(
            "points_voxel_topic",
            default_value="/realsense/depth/points_voxel",
        ),
        DeclareLaunchArgument(
            "cloud_out",
            default_value="/realsense/depth/cloud_out",
        ),
        # 关键：不要给空字符串，给一个真实默认文件
        DeclareLaunchArgument(
            "voxel_params",
            default_value=default_voxel_params,
        ),

        pc_node,
        voxel_node,
        relay_voxel,
        relay_raw,
    ])
