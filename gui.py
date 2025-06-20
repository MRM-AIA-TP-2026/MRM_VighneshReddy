#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from tkinter import Tk, Label, Button, filedialog
from PIL import Image, ImageTk
from cv_bridge import CvBridge
from sensor_msgs.msg import Image as ROSImage
from sensor_msgs.msg import Imu, NavSatFix, LaserScan
import cv2
import pandas as pd
from datetime import datetime

rclpy.init()
node = Node('tkinter_ros_gui')
bridge = CvBridge()

imu_representation = "euler"
data_storage = []
record_video = False
video_writer = None
video_frame = None

# all the necessary callback functions
def imu_callback(msg):
    global imu_representation, data_storage
    if imu_representation == "euler":
        roll, pitch, yaw = quaternion_to_euler(msg.orientation)
        imu_data = f"Roll: {roll:.2f}, Pitch: {pitch:.2f}, Yaw: {yaw:.2f}"
    else:
        imu_data = f"Quaternion: x={msg.orientation.x}, y={msg.orientation.y}, z={msg.orientation.z}, w={msg.orientation.w}"
    imu_label.config(text=f"IMU Data: {imu_data}")
    data_storage.append(["IMU", imu_data, datetime.now().isoformat()])

def gps_callback(msg):
    global data_storage
    gps_data = f"Lat: {msg.latitude:.4f}, Lon: {msg.longitude:.4f}, Alt: {msg.altitude:.2f}"
    gps_label.config(text=f"GPS Data: {gps_data}")
    data_storage.append(["GPS", gps_data, datetime.now().isoformat()])

def video_callback(msg):
    global video_frame, record_video, video_writer
    frame = bridge.imgmsg_to_cv2(msg, "bgr8")
    if record_video:
        video_writer.write(frame)
    video_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    frame_image = ImageTk.PhotoImage(Image.fromarray(video_frame))
    video_label.config(image=frame_image)
    video_label.image = frame_image

def lidar_callback(msg):
    if detect_obstacle(msg):
        obstacle_label.config(text="Obstacle: Detected!")
    else:
        obstacle_label.config(text="Obstacle: Path Clear")

def quaternion_to_euler(q):
    import math
    sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (q.w * q.y - q.z * q.x)
    pitch = math.asin(sinp)

    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw

def detect_obstacle(laser_scan):
    threshold_distance = 3.5
    return any(range_val < threshold_distance for range_val in laser_scan.ranges)

def toggle_imu_representation():
    global imu_representation
    imu_representation = "quaternion" if imu_representation == "euler" else "euler"

def take_screenshot():
    global video_frame
    if video_frame is not None:
        file_path = filedialog.asksaveasfilename(
            defaultextension=".png",
            filetypes=[("PNG files", "*.png"), ("All files", "*.*")]
        )
        if file_path:
            cv2.imwrite(file_path, cv2.cvtColor(video_frame, cv2.COLOR_RGB2BGR))

def toggle_recording():
    global record_video, video_writer
    if not record_video:
        file_path = filedialog.asksaveasfilename(
            defaultextension=".avi",
            filetypes=[("AVI files", "*.avi"), ("All files", "*.*")]
        )
        if file_path:
            fourcc = cv2.VideoWriter_fourcc(*'XVID')
            video_writer = cv2.VideoWriter(file_path, fourcc, 20.0, (640, 480))
            record_video = True
            record_button.config(text="Stop Recording")
    else:
        video_writer.release()
        record_video = False
        record_button.config(text="Start Recording")

def export_data():
    global data_storage
    file_path = filedialog.asksaveasfilename(
        defaultextension=".csv",
        filetypes=[("CSV files", "*.csv"), ("All files", "*.*")]
    )
    if file_path:
        df = pd.DataFrame(data_storage, columns=["Type", "Data", "Timestamp"])
        df.to_csv(file_path, index=False)

def ros_spin():
    rclpy.spin_once(node, timeout_sec=0.1)
    root.after(100, ros_spin)

# initializing tkinter
root = Tk()
root.title("ROS 2 Data Viewer")
root.geometry("1200x720")

# gui part
imu_label = Label(root, text="IMU Data: Not Available", font=("Helvetica", 12))
imu_label.pack(pady=10)

gps_label = Label(root, text="GPS Data: Not Available", font=("Helvetica", 12))
gps_label.pack(pady=10)

obstacle_label = Label(root, text="Obstacle: Unknown", font=("Helvetica", 12))
obstacle_label.pack(pady=10)

video_label = Label(root)
video_label.pack(pady=10)

toggle_button = Button(root, text="Switch IMU Data Representation", command=toggle_imu_representation)
toggle_button.pack(pady=5)

screenshot_button = Button(root, text="Take Screenshot", command=take_screenshot)
screenshot_button.pack(pady=5)

record_button = Button(root, text="Start Recording", command=toggle_recording)
record_button.pack(pady=5)

export_button = Button(root, text="Export Data", command=export_data)
export_button.pack(pady=5)

node.create_subscription(Imu, '/imu', imu_callback, 10)
node.create_subscription(NavSatFix, '/gps', gps_callback, 10)
node.create_subscription(ROSImage, '/d430/color/image_raw', video_callback, 10)
node.create_subscription(LaserScan, '/scan', lidar_callback, 10)

root.after(100, ros_spin)
root.mainloop()

rclpy.shutdown()
