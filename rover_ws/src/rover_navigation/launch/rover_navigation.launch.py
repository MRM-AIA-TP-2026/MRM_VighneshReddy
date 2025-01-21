from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_rover = get_package_share_directory('gazebo')
    
    # Include the robot state publisher launch file
    rover_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_rover, 'launch', 'gazebo_launch.py')
        )
    )
    
    # Launch the navigation node
    navigation_node = Node(
        package='rover_navigation',
        executable='haversine_navigation',
        name='haversine_navigation',
        output='screen'
    )
    
    return LaunchDescription([
        rover_gazebo,
        navigation_node
    ])

