from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Path to the cartographer Lua configuration file
    cartographer_config_path = "/home/vighneshreddy/MRM_VighneshReddy/rover_ws/src/robot/config/cartographer_2d.lua"

    # Launch Cartographer Node
    cartographer_node = Node(
        package="cartographer_ros",
        executable="cartographer_node",
        name="cartographer_node",
        output="screen",
        parameters=[{
            "use_sim_time": True,  # Enable this if using Gazebo or simulated time
        }],
        arguments=["-configuration_directory", "/home/vighneshreddy/MRM_VighneshReddy/rover_ws/src/robot/config",
                   "-configuration_basename", "cartographer_2d.lua"]
    )

    # Launch RViz2 for visualization
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", "/home/vighneshreddy/MRM_VighneshReddy/rover_ws/src/robot/rviz/config.rviz"]  # Update RViz config path if needed
    )

    return LaunchDescription([
        cartographer_node,
        rviz_node,
    ])

