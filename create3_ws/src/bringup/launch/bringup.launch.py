from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # ======================
        # Image server
        # ======================
        Node(
            package='img_server',
            executable='img_server_node',
            name='img_server',
            output='screen'
        ),

        # ======================
        # Navigation
        # ======================
        Node(
            package='ez_nav',
            executable='ez_nav_node',
            name='ez_nav',
            output='screen'
        ),
        
        
    ])
