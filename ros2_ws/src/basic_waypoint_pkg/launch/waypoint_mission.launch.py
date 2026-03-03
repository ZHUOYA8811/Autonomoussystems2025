from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch argument (currently not used further, but kept for parity)
    mav_name_arg = DeclareLaunchArgument(
        "mav_name",
        default_value="firefly",
        description="Name of the MAV"
    )

    auto_start_route_arg = DeclareLaunchArgument(
        "auto_start_route",
        default_value="false",  # 与状态机配合使用时关闭，独立使用时可传 auto_start_route:=true
        description=(
            "If true, planner waits for odom "
            "then auto-plans configured route"
        )
    )
    auto_start_timeout_arg = DeclareLaunchArgument(
        "auto_start_timeout_s",
        default_value="10.0",
        description="Max wait time for odometry before auto route abort"
    )

    # Path to trajectory_config.yaml
    trajectory_config = PathJoinSubstitution([
        FindPackageShare("basic_waypoint_pkg"),
        "config",
        "trajectory_config.yaml"
    ])

    # Trajectory planner node
    planner_node = Node(
        package="basic_waypoint_pkg",
        executable="basic_waypoint_node",   # ROS1: type="basic_waypoint_pkg"
        name="planner",
        output="screen",
        parameters=[
            trajectory_config,
            {
                "auto_start_route": LaunchConfiguration("auto_start_route"),
                "auto_start_timeout_s": LaunchConfiguration(
                    "auto_start_timeout_s"
                ),
            },
        ]
    )

    # Trajectory sampler node
    sampler_node = Node(
        package="mav_trajectory_generation",
        executable="trajectory_sampler_node",
        name="sampler",
        output="screen",
        # No remappings: native path_segments_4D subscription handles both
        # basic_waypoint_pkg and planner_pkg output
    )

    return LaunchDescription([
        mav_name_arg,
        auto_start_route_arg,
        auto_start_timeout_arg,
        planner_node,
        sampler_node,
    ])
