from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Ruta al share del paquete
    pkg_share = os.path.join(os.getenv('COLCON_PREFIX_PATH').split(':')[0], 'share')

    # Nodo de publicación de imágenes
    image_node = Node(
        package='image_png_player',
        executable='image_player',
        name='image_player',
        output='screen'
    )

    # Nodo de odometría visual
    vodom_node = Node(
        package='vodom',
        executable='vodom',
        name='vodom',
        output='screen'
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_share, 'pr_pkg', 'rviz', 'slam_config.rviz')],
        output='screen'
    )

    return LaunchDescription([
        image_node,
        vodom_node,
        rviz_node
    ])

