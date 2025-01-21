from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time if true'),
            
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'max_laser_range': 20.0},
                {'resolution': 0.05},
                {'map_update_interval': 5.0},
                {'transform_timeout': 0.2},
                {'map_frame': 'map'},
                {'base_frame': 'base_link'},  # Adjust this to your robot's base frame
                {'odom_frame': 'odom'}
            ]
        )
    ])
