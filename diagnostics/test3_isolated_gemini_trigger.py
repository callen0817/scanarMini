#!/usr/bin/env python3
import os
import sys
import time
import signal
import subprocess
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
import tf2_ros

print('=== TEST 3: ISOLATED GEMINI TRIGGER TEST (AIRY + GEMINI ONLY) ===')

# Launch bringup (Airy + Gemini)
p_bringup = subprocess.Popen(
    ['bash', '-c', 'source /opt/ros/humble/setup.bash && source /home/scanar/scanarMini/install/setup.bash && ros2 launch scan_routine sensor_bringup.launch.py'],
    stdout=open('/tmp/test3_bringup.log', 'w'),
    stderr=subprocess.STDOUT,
    preexec_fn=os.setsid
)

print('Waiting 5 seconds for sensor initialization...')
time.sleep(5.0)

class GeminiMonitor(Node):
    def __init__(self):
        super().__init__('gemini_trigger_monitor')
        self.depth_count = 0
        self.color_count = 0
        self.depth_info = None
        self.color_info = None
        self.start_time = time.time()
        
        self.create_subscription(Image, '/camera/depth/image_raw', self.cb_depth, 30)
        self.create_subscription(Image, '/camera/color/image_raw', self.cb_color, 30)
        self.create_subscription(CameraInfo, '/camera/depth/camera_info', self.cb_d_info, 10)
        self.create_subscription(CameraInfo, '/camera/color/camera_info', self.cb_c_info, 10)
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.timer = self.create_timer(1.0, self.report)

    def cb_depth(self, msg): self.depth_count += 1
    def cb_color(self, msg): self.color_count += 1
    def cb_d_info(self, msg): 
        if self.depth_info is None:
            self.depth_info = msg
    def cb_c_info(self, msg):
        if self.color_info is None:
            self.color_info = msg

    def report(self):
        el = time.time() - self.start_time
        if el > 0:
            print(f'[{el:4.1f}s] ROS Depth Rate = {self.depth_count/el:5.2f} Hz ({self.depth_count} cnt) | ROS Color Rate = {self.color_count/el:5.2f} Hz ({self.color_count} cnt)')

rclpy.init()
mon = GeminiMonitor()

start = time.time()
try:
    while time.time() - start < 25.0:
        rclpy.spin_once(mon, timeout_sec=0.1)
except KeyboardInterrupt:
    pass

print('\n=== CHECKING EXACT DEPTH CAMERA INFO ===')
if mon.depth_info:
    print(f'Depth Width: {mon.depth_info.width}, Height: {mon.depth_info.height}')
    print(f'Depth Distortion Model: {mon.depth_info.distortion_model}')
    print(f'Depth D (Distortion Coeffs): {mon.depth_info.d}')
    print(f'Depth K (Intrinsics Matrix):')
    K = mon.depth_info.k
    print(f'  fx={K[0]:.4f}, fy={K[4]:.4f}, cx={K[2]:.4f}, cy={K[5]:.4f}')
else:
    print('Depth CameraInfo not received.')

print('\n=== CHECKING EXACT COLOR CAMERA INFO ===')
if mon.color_info:
    print(f'Color Width: {mon.color_info.width}, Height: {mon.color_info.height}')
    print(f'Color Distortion Model: {mon.color_info.distortion_model}')
    print(f'Color D (Distortion Coeffs): {mon.color_info.d}')
    print(f'Color K (Intrinsics Matrix):')
    K = mon.color_info.k
    print(f'  fx={K[0]:.4f}, fy={K[4]:.4f}, cx={K[2]:.4f}, cy={K[5]:.4f}')
else:
    print('Color CameraInfo not received.')

print('\n=== CHECKING TF BETWEEN camera_left_ir_optical_frame AND camera_depth_optical_frame ===')
try:
    tf_val = mon.tf_buffer.lookup_transform('camera_left_ir_optical_frame', 'camera_depth_optical_frame', rclpy.time.Time())
    trans = tf_val.transform.translation
    rot = tf_val.transform.rotation
    print(f'TF camera_left_ir_optical_frame -> camera_depth_optical_frame:')
    print(f'  Translation: [{trans.x:.6f}, {trans.y:.6f}, {trans.z:.6f}]')
    print(f'  Rotation (quat): [{rot.x:.6f}, {rot.y:.6f}, {rot.z:.6f}, {rot.w:.6f}]')
except Exception as e:
    print(f'TF lookup exception: {e}')

mon.destroy_node()
rclpy.shutdown()

print('Stopping bringup gracefully...')
os.killpg(os.getpgid(p_bringup.pid), signal.SIGINT)
time.sleep(2.0)
try:
    os.killpg(os.getpgid(p_bringup.pid), signal.SIGKILL)
except:
    pass

print('Test 3 completed.')
