from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    入口 launch：将规划任务委托给 path_planner_pkg（A*/RRT + 直接输出 command/trajectory）。
    用法：
      ros2 launch planner_pkg planner.launch.py               # A*（默认）
      ros2 launch planner_pkg planner.launch.py use_astar:=false  # RRT
    """
    # 声明可透传给 path_planner_pkg 的参数
    use_astar_arg = DeclareLaunchArgument(
        'use_astar', default_value='true',
        description='Use A* (true) or RRT (false)')
    map_resolution_arg = DeclareLaunchArgument(
        'map_resolution', default_value='0.5',
        description='Occupancy map resolution in meters')
    robot_radius_arg = DeclareLaunchArgument(
        'robot_radius', default_value='0.8',
        description='Robot collision radius in meters')
    max_speed_arg = DeclareLaunchArgument(
        'max_speed', default_value='2.0',
        description='Maximum UAV speed in m/s')
    explore_height_arg = DeclareLaunchArgument(
        'explore_height', default_value='5.0',
        description='Preferred exploration height in meters')

    path_planner_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('path_planner_pkg'),
                'launch',
                'path_planner.launch.py'
            ])
        ),
        launch_arguments={
            'use_astar':      LaunchConfiguration('use_astar'),
            'map_resolution': LaunchConfiguration('map_resolution'),
            'robot_radius':   LaunchConfiguration('robot_radius'),
            'max_speed':      LaunchConfiguration('max_speed'),
            'explore_height': LaunchConfiguration('explore_height'),
        }.items()
    )

    return LaunchDescription([
        use_astar_arg,
        map_resolution_arg,
        robot_radius_arg,
        max_speed_arg,
        explore_height_arg,
        path_planner_launch,
    ])
