#Global Planner
- Implement the Autonomous Traversal of the 4 wheel bot designed in the Group Task from one GPS Coordinate to another using C++ and OOP Concepts (consider no obstacles in the path).

#Nodes topics and messages used : 
- Nodes:

    autonomous_navigation (Node):
        This is the main node handling GPS and IMU data for autonomous navigation
        Subscribes to GPS and IMU topics to track the robot's position and orientation
        Publishes velocity commands to control the robot

- Topics:

    /gps (Subscribed):
        Message Type: sensor_msgs/msg/NavSatFix
        Provides the robot's current global position in latitude and longitude
        Used to calculate the distance and heading to the target destination

    /imu (Subscribed):
        Message Type: sensor_msgs/msg/Imu
        Provides orientation data of the robot using a quaterion
        Used to calculate the robot's current heading in radians.

    /cmd_vel (Published):
        Message Type: geometry_msgs/msg/Twist
        Sends linear and angular velocity commands to the robot's movement system
        Controls the robot's speed and rotation to navigate towards the target

- Message Descriptions:

    sensor_msgs/msg/NavSatFix:
        Fields Used:
            latitude (float64): The current latitude of the robot
            longitude (float64): The current longitude of the robot
        Provides the robot's position in global coordinates (latitude, longitude)

    sensor_msgs/msg/Imu:
        Fields Used:
            orientation (geometry_msgs/msg/Quaternion): Quaternion representing the robot's orientation
        Used to calculate the robot's heading (yaw) in radians

    geometry_msgs/msg/Twist:
        Fields Used:
            linear.x (float64): Linear velocity along the x-axis 
            angular.z (float64): Angular velocity around the z-axis- that is "rotation"
        Used to control the robot's movement by specifying speed and turning rate

#Link to youtube video

- https://youtu.be/qJU9y9verV4




