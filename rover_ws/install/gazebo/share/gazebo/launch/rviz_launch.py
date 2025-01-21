from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    # Define the paths to the necessary files in the package
    package_name = 'gazebo'
    model_dir = os.path.join(os.getenv('AMENT_PREFIX_PATH', '/'), package_name, 'model')
    
    xacro_file = os.path.join(model_dir, 'robot.xacro')  # Replace with your xacro file name
    gazebo_world_file = os.path.join(model_dir, 'default.world')  # Replace with your world file name
    rviz_config_file = os.path.join(model_dir, 'robot_config.rviz')  # Replace with your rviz config file name

    return LaunchDescription([
        # Robot State Publisher (process xacro to publish robot description)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': ExecuteProcess(
                    cmd=['xacro', xacro_file],
                    output='screen'
                )
            }],
        ),
        
        # Launch Gazebo
        ExecuteProcess(
            cmd=['gazebo', '--verbose', gazebo_world_file],
            output='screen'
        ),

        # Launch RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file],
        ),
    ])
