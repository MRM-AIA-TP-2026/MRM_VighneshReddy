from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('robot')
    
    # Include the Gazebo launch file
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'gazebo.launch.py')
        )
    )
    
    # Include the RViz launch file
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'display.launch.py')
        )
    )
    
    # Static transform from map to base_link
    static_transform_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_map_to_base_link',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link']
    )
      
    return LaunchDescription([
        gazebo_launch,
        rviz_launch,
        static_transform_publisher,
    ])

