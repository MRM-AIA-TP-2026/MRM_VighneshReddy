#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid

def main():
    rclpy.init()
    node = Node('map_relay')

    pub = node.create_publisher(OccupancyGrid, '/map', 10)

    def callback(msg):
        pub.publish(msg)

    node.create_subscription(OccupancyGrid, '/rtabmap/map', callback, 10)

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
