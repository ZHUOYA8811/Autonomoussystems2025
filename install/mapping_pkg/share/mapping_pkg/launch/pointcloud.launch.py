from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_voxel = LaunchConfiguration("use_voxel")
    points_topic = LaunchConfiguration("points_topic")
    points_voxel_topic = LaunchConfiguration("points_voxel_topic")

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

    return LaunchDescription([
        DeclareLaunchArgument("use_voxel", default_value="true"),
        DeclareLaunchArgument("points_topic", default_value="/realsense/depth/points"),
        DeclareLaunchArgument("points_voxel_topic", default_value="/realsense/depth/points_voxel"),
        # 关键：不要给空字符串，给一个真实默认文件
        DeclareLaunchArgument("voxel_params", default_value=default_voxel_params),

        pc_node,
        voxel_node,
    ])