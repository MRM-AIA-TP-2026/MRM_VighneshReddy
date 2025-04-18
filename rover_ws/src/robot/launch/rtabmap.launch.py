import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config_dir = os.path.join(
        os.getenv('ROBOT_WS', '/home/username/MRM_VighneshReddy/rover_ws'),
        'src',
        'robot',
        'config'
    )

    return LaunchDescription([
        Node(
            package='rtabmap_ros',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[
                os.path.join(config_dir, 'rtabmap_config.yaml')
            ]
        ),
        Node(
            package='rtabmap_ros',
            executable='rtabmapviz',
            name='rtabmapviz',
            output='screen',
            parameters=[
                os.path.join(config_dir, 'rtabmap_launch.yaml')
            ]
        )
    ])

