#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import socket
import csv
import io

rclpy.init()

node = rclpy.create_node('imu_receiver')

publisher = node.create_publisher(Imu, '/imu/phone', 10)

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('10.168.133.129', 2055))  

def receive_and_publish():
    try:
        node.get_logger().info("Waiting for data...")
        
        data, _ = sock.recvfrom(1024)
        node.get_logger().info(f"Raw data received: {data}")

        data_stream = io.StringIO(data.decode('utf-8').strip())
        reader = csv.reader(data_stream)
        values = next(reader)  # for extraction of only the first line

        # csv to float conver
        orientation_x, orientation_y, orientation_z = map(float, values)

        msg = Imu()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        msg.orientation.x = orientation_x
        msg.orientation.y = orientation_y
        msg.orientation.z = orientation_z
        msg.orientation.w = 1.0  

        publisher.publish(msg)
        node.get_logger().info("Published IMU message")
    except Exception as e:
        node.get_logger().error(f"Error while receiving or publishing data: {e}")
    finally:
        node.get_logger().info("Finished execution of receive_and_publish")

timer_period = 0.01  
timer = node.create_timer(timer_period, receive_and_publish)

try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
finally:
    timer.cancel()
    node.destroy_node()
    rclpy.shutdown()
