#Global Planner
- Implement the Autonomous Traversal of the 4 wheel bot designed in the Group Task from one GPS Coordinate to another using C++ and OOP Concepts (consider no obstacles in the path).

- Terminal #1

cd ~/MRM_VighneshReddy/rover_ws

source /opt/ros/humble/setup.bash 

source install/local_setup.bash

ros2 launch gazebo gazebo_launch.py

-Terminal #2

source /opt/ros/humble/setup.bash

source install/local_setup.bash

ros2 run rover_navigation navigation_node

-Terminal #3 (for robot position)

source /opt/ros/humble/setup.bash


source install/local_setup.bash


ros2 run robot_position get_position



