from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    return LaunchDescription([
        # SLAM Toolbox Node
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                '/opt/ros/humble/share/slam_toolbox/config/mapper_params_online_async.yaml'
            ]
        ),
        
        # Nav2 Bringup Node
        Node(
            package='nav2_bringup',
            executable='bringup_launch.py',
            output='screen',
            parameters=[
                '/opt/ros/humble/share/nav2_bringup/params/nav2_params.yaml'
            ]
        )
    ])

