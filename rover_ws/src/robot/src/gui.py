#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from tkinter import Tk, Label, Button, filedialog, Frame
from PIL import Image, ImageTk
from cv_bridge import CvBridge
from sensor_msgs.msg import Image as ROSImage
from sensor_msgs.msg import Imu, NavSatFix, LaserScan
import cv2
import pandas as pd
from datetime import datetime
import math

rclpy.init()
node = Node('tkinter_ros_gui')
bridge = CvBridge()

imu_representation = "euler"
data_storage = []
record_video = False
video_writer = None
video_frame = None

def quaternion_to_euler(q):
    sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (q.w * q.y - q.z * q.x)
    pitch = math.asin(max(-1.0, min(1.0, sinp)))  # Clamp sinp to [-1, 1]

    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

def detect_obstacle(laser_scan):
    threshold_distance = 3.5
    return any(r < threshold_distance for r in laser_scan.ranges if r > 0)

def imu_callback(msg):
    global imu_representation, data_storage
    if imu_representation == "euler":
        roll, pitch, yaw = quaternion_to_euler(msg.orientation)
        imu_data = f"Roll: {roll:.2f}, Pitch: {pitch:.2f}, Yaw: {yaw:.2f}"
    else:
        imu_data = f"Quaternion: x={msg.orientation.x:.2f}, y={msg.orientation.y:.2f}, z={msg.orientation.z:.2f}, w={msg.orientation.w:.2f}"
    imu_label.config(text=f"IMU Data:\n{imu_data}")
    data_storage.append(["IMU", imu_data, datetime.now().isoformat()])

def gps_callback(msg):
    global data_storage
    gps_data = f"Lat: {msg.latitude:.4f}, Lon: {msg.longitude:.4f}, Alt: {msg.altitude:.2f}"
    gps_label.config(text=f"GPS Data:\n{gps_data}")
    data_storage.append(["GPS", gps_data, datetime.now().isoformat()])

def video_callback(msg):
    global video_frame, record_video, video_writer
    frame = bridge.imgmsg_to_cv2(msg, "bgr8")
    video_frame = cv2.resize(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB), (640, 480))
    img = ImageTk.PhotoImage(Image.fromarray(video_frame))
    video_label.config(image=img)
    video_label.image = img

    if record_video and video_writer:
        video_writer.write(cv2.cvtColor(video_frame, cv2.COLOR_RGB2BGR))

def lidar_callback(msg):
    if detect_obstacle(msg):
        obstacle_label.config(text="Obstacle: Detected")
    else:
        obstacle_label.config(text="Obstacle: Path Clear")

def toggle_imu_representation():
    global imu_representation
    imu_representation = "quaternion" if imu_representation == "euler" else "euler"

def take_screenshot():
    global video_frame
    if video_frame is not None:
        path = filedialog.asksaveasfilename(defaultextension=".png")
        if path:
            cv2.imwrite(path, cv2.cvtColor(video_frame, cv2.COLOR_RGB2BGR))

def toggle_recording():
    global record_video, video_writer
    if not record_video:
        path = filedialog.asksaveasfilename(defaultextension=".avi")
        if path:
            fourcc = cv2.VideoWriter_fourcc(*'XVID')
            video_writer = cv2.VideoWriter(path, fourcc, 20.0, (640, 480))
            record_video = True
            record_button.config(text="Stop Recording")
    else:
        if video_writer:
            video_writer.release()
        video_writer = None
        record_video = False
        record_button.config(text="Start Recording")

def export_data():
    global data_storage
    path = filedialog.asksaveasfilename(defaultextension=".csv")
    if path:
        pd.DataFrame(data_storage, columns=["Type", "Data", "Timestamp"]).to_csv(path, index=False)

def ros_spin():
    rclpy.spin_once(node, timeout_sec=0.01)
    root.after(10, ros_spin)

# Initialize GUI
root = Tk()
root.title("ROS 2 Rover GUI")
root.geometry("1280x720")

# Frames
left_frame = Frame(root)
left_frame.pack(side="left", padx=20, pady=20)

right_frame = Frame(root)
right_frame.pack(side="right", padx=20, pady=20)

# Video feed
video_label = Label(left_frame)
video_label.pack()

# Labels
imu_label = Label(right_frame, text="IMU Data: Not Available", font=("Helvetica", 12), justify="left")
imu_label.pack(pady=5)

gps_label = Label(right_frame, text="GPS Data: Not Available", font=("Helvetica", 12), justify="left")
gps_label.pack(pady=5)

obstacle_label = Label(right_frame, text="Obstacle: Unknown", font=("Helvetica", 12))
obstacle_label.pack(pady=5)

# Buttons
Button(right_frame, text="Switch IMU Format", command=toggle_imu_representation).pack(pady=5)
Button(right_frame, text="Take Screenshot", command=take_screenshot).pack(pady=5)
record_button = Button(right_frame, text="Start Recording", command=toggle_recording)
record_button.pack(pady=5)
Button(right_frame, text="Export Data", command=export_data).pack(pady=5)

# ROS subscriptions
node.create_subscription(Imu, '/imu/data', imu_callback, 10)
node.create_subscription(NavSatFix, '/gps', gps_callback, 10)
node.create_subscription(ROSImage, '/d430/color/image_raw', video_callback, 10)
node.create_subscription(LaserScan, '/scan', lidar_callback, 10)

# Start loop
root.after(100, ros_spin)
root.mainloop()
rclpy.shutdown()

