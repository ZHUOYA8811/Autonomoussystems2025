import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 定义节点要加载的参数
    # 这里你可以自定义 checkSystemstate 需要监控的节点名单
    mission_params = {
        'monitored_node_list': ['controller', 'sampler', 'navigator'],
        'takeoff_height': 5.0,
        'alive_tol_sec': 10.0
    }

    # 2. 定义状态机节点
    state_machine_node = Node(
        package='state_machine',           # 你的包名
        executable='state_machine_node',   # CMake 里的 add_executable 名字
        name='state_machine_manager',      # 运行时在 ROS 中的名字
        output='screen',                   # 将日志打印到屏幕
        parameters=[mission_params]        # 加载上面定义的参数
    )

    # 3. 返回启动描述
    return LaunchDescription([
        state_machine_node
    ])