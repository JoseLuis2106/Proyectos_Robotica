from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    image_player = Node(
        package='image_png_player',
        executable='Prueba',
        name='image_player',
        parameters=[{
            'image_folder': '/home/jlmre/dataset/images',
            'frame_id': 'camera_link'
        }]
    )

    # TF estático base_link → camera_link
    tf_base_to_cam = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_cam',
        arguments=['0','0','0.1','0','0','0','base_link','camera_link']
    )

    # RTAB-Map monocular
    rtabmap = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        parameters=[{
            'frame_id': 'base_link',
            'subscribe_depth': False,
            'subscribe_rgbd': False,
            'approx_sync': True,
            'use_sim_time': True,
            'topic_queue_size': 50,
            'sync_queue_size': 50
        }],
        remappings=[
            ('rgb/image', '/camera/image_raw'),
            ('rgb/camera_info', '/camera/camera_info'),
            ('odom', '/odom')
        ]
    )

    return LaunchDescription([image_player, tf_base_to_cam, rtabmap])

