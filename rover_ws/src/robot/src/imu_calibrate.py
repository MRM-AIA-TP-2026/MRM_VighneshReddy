#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import board
import busio
import adafruit_bno055
import json
import time

class BNO055CalibNode(Node):
    def __init__(self):
        super().__init__('bno055_node')
        i2c = busio.I2C(board.SCL, board.SDA)
        self.sensor = adafruit_bno055.BNO055_I2C(i2c)
        self.restore_calibration()

        self.timer = self.create_timer(1.0, self.check_calib)

    def check_calib(self):
        sys, gyro, accel, mag = self.sensor.calibration_status
        self.get_logger().info(f"Sys:{sys} Gyro:{gyro} Accel:{accel} Mag:{mag}")
        if sys == gyro == accel == mag == 3:
            self.save_calibration()
            self.get_logger().info("Fully calibrated! Saved offsets.")
            self.timer.cancel()  # Stop checking after save

    def save_calibration(self):
        self.sensor.mode = adafruit_bno055.CONFIG_MODE
        time.sleep(0.02)

        offsets = {
            'accel_offset': self.sensor.offsets_accelerometer,
            'gyro_offset': self.sensor.offsets_gyroscope,
            'mag_offset': self.sensor.offsets_magnetometer,
            'accel_radius': self.sensor.radius_accelerometer,
            'mag_radius': self.sensor.radius_magnetometer
        }

        with open('bno055_calibration.json', 'w') as f:
            json.dump(offsets, f)

        self.sensor.mode = adafruit_bno055.NDOF_MODE

    def restore_calibration(self):
        try:
            with open('bno055_calibration.json', 'r') as f:
                offsets = json.load(f)

            self.sensor.mode = adafruit_bno055.CONFIG_MODE
            time.sleep(0.02)

            self.sensor.offsets_accelerometer = tuple(offsets['accel_offset'])
            self.sensor.offsets_gyroscope = tuple(offsets['gyro_offset'])
            self.sensor.offsets_magnetometer = tuple(offsets['mag_offset'])
            self.sensor.radius_accelerometer = offsets['accel_radius']
            self.sensor.radius_magnetometer = offsets['mag_radius']

            self.sensor.mode = adafruit_bno055.NDOF_MODE
            self.get_logger().info("Calibration restored.")
        except Exception as e:
            self.get_logger().warn(f"Failed to restore calibration: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = BNO055CalibNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

