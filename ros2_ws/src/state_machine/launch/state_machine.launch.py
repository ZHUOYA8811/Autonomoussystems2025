from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 声明可在命令行覆盖的启动参数
    # 用法示例: ros2 launch state_machine state_machine.launch.py takeoff_height:=10.0
    
    declare_monitored_nodes = DeclareLaunchArgument(
        'monitored_node_list',
        default_value="['controller', 'navigator']",
        description='监控的节点列表'
    )
    
    declare_takeoff_height = DeclareLaunchArgument(
        'takeoff_height',
        default_value='5.0',
        description='起飞高度 (米)'
    )
    
    declare_alive_tol = DeclareLaunchArgument(
        'alive_tol_sec',
        default_value='10.0',
        description='节点存活容忍时间 (秒)'
    )
    
    declare_checkpoint_dist = DeclareLaunchArgument(
        'checkpoint_reach_dist_m',
        default_value='2.0',
        description='检查点到达距离阈值 (米)'
    )
    
    declare_log_level = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='日志级别: debug, info, warn, error, fatal'
    )

    # 定义状态机节点
    state_machine_node = Node(
        package='state_machine',
        executable='state_machine_node',
        name='state_machine_manager',
        output='screen',
        parameters=[{
            'monitored_node_list': LaunchConfiguration('monitored_node_list'),
            'takeoff_height': LaunchConfiguration('takeoff_height'),
            'alive_tol_sec': LaunchConfiguration('alive_tol_sec'),
            'checkpoint_reach_dist_m': LaunchConfiguration('checkpoint_reach_dist_m'),
        }],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
    )

    return LaunchDescription([
        declare_monitored_nodes,
        declare_takeoff_height,
        declare_alive_tol,
        declare_checkpoint_dist,
        declare_log_level,
        state_machine_node
    ])