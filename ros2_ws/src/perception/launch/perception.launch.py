from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    depth_image = '/realsense/depth/image'
    camera_info = '/realsense/depth/camera_info'
    points_out  = '/camera/depth/points'
    world_frame = 'world'

    depth_to_points = Node(
        package='depth_image_proc',
        executable='point_cloud_xyz_node',
        name='depth_to_points',
        remappings=[
            ('image_rect', depth_image),
            ('camera_info', camera_info),
            ('points', points_out),
        ],
        output='screen'
    )

    octomap = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server',
        remappings=[
            ('cloud_in', points_out),
        ],
        parameters=[{
            'frame_id': world_frame,
            'resolution': 0.2,
            'latch': True,
        }],
        output='screen'
    )
    
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0','0','0','0','0','0',
                   'Quadrotor/DepthCamera','Quadrotor/Sensors/DepthCamera'],
        output='screen'
    )

    return LaunchDescription([static_tf, depth_to_points, octomap])
