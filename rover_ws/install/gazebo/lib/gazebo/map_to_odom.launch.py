from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gazebo',  # Replace with your package name
            executable='map_to_odom_publisher',  # Replace with your executable name
            name='map_to_odom_publisher',
            output='screen',
        ),
    ])

