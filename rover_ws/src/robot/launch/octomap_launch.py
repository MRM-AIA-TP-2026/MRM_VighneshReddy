from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Define the path to the config file relative to the package
    package_share_directory = FindPackageShare('robot').find('robot')
    config_file = os.path.join(package_share_directory, 'config', 'octomap.yaml')

    return LaunchDescription([
        Node(
            package='octomap_server',
            executable='octomap_server_node',
            name='octomap_server',
            output='screen',
            parameters=[{'frame_id': 'base_link'}, {'resolution': 0.1}, config_file],
            remappings=[('/scan', '/lidar/scan')]  # Update the scan topic if needed
        )
    ])

