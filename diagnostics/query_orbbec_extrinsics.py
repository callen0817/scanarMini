#!/usr/bin/env python3
import os
import sys
import time
import signal
import subprocess
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from orbbec_camera_msgs.msg import Extrinsics
import tf2_ros

print('=== QUERYING ORBBEC CAMERA FACTORY CALIBRATION & EXTRINSICS ===')

# Launch orbbec camera with left_ir and depth enabled
orbbec_launch = '/home/scanar/scanarMini/src/orbbec_camera/orbbec_camera/launch/gemini_330_series.launch.py'
p_cam = subprocess.Popen(
    ['bash', '-c', f'source /opt/ros/humble/setup.bash && source /home/scanar/scanarMini/install/setup.bash && ros2 launch orbbec_camera gemini_330_series.launch.py camera_name:=camera enable_left_ir:=true enable_depth:=true enable_color:=true enable_publish_extrinsic:=true publish_tf:=true depth_width:=848 depth_height:=480 depth_fps:=30 left_ir_width:=848 left_ir_height:=480 left_ir_fps:=30 color_width:=1280 color_height:=800 color_fps:=30'],
    stdout=open('/tmp/query_cam.log', 'w'),
    stderr=subprocess.STDOUT,
    preexec_fn=os.setsid
)

time.sleep(5.0)

class ExtrinsicQueryNode(Node):
    def __init__(self):
        super().__init__('extrinsic_query_node')
        self.depth_info = None
        self.ir_info = None
        self.depth_to_ir_ext = None
        self.depth_to_color_ext = None
        
        self.create_subscription(CameraInfo, '/camera/depth/camera_info', self.cb_d, 10)
        self.create_subscription(CameraInfo, '/camera/ir/camera_info', self.cb_ir, 10)
        self.create_subscription(Extrinsics, '/camera/depth_to_ir_extrinsics', self.cb_d_ir, 10)
        self.create_subscription(Extrinsics, '/camera/depth_to_color_extrinsics', self.cb_d_col, 10)
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def cb_d(self, msg):
        if self.depth_info is None: self.depth_info = msg
    def cb_ir(self, msg):
        if self.ir_info is None: self.ir_info = msg
    def cb_d_ir(self, msg):
        if self.depth_to_ir_ext is None: self.depth_to_ir_ext = msg
    def cb_d_col(self, msg):
        if self.depth_to_color_ext is None: self.depth_to_color_ext = msg

rclpy.init()
node = ExtrinsicQueryNode()

start = time.time()
while time.time() - start < 10.0:
    rclpy.spin_once(node, timeout_sec=0.2)
    if node.depth_info and node.ir_info and node.depth_to_ir_ext:
        break

print('\n=== RESULTS FROM ORBBEC SDK DRIVER ===')
if node.depth_info:
    print(f'Depth CameraInfo: frame_id={node.depth_info.header.frame_id}, fx={node.depth_info.k[0]:.4f}, fy={node.depth_info.k[4]:.4f}, cx={node.depth_info.k[2]:.4f}, cy={node.depth_info.k[5]:.4f}')
if node.ir_info:
    print(f'Left IR CameraInfo: frame_id={node.ir_info.header.frame_id}, fx={node.ir_info.k[0]:.4f}, fy={node.ir_info.k[4]:.4f}, cx={node.ir_info.k[2]:.4f}, cy={node.ir_info.k[5]:.4f}')

if node.depth_to_ir_ext:
    print('\nDepth to IR Extrinsic (/camera/depth_to_ir_extrinsics):')
    print(f'  Rotation: {node.depth_to_ir_ext.rotation}')
    print(f'  Translation: {node.depth_to_ir_ext.translation}')
else:
    print('depth_to_ir_extrinsics topic not received.')

if node.depth_to_color_ext:
    print('\nDepth to Color Extrinsic (/camera/depth_to_color_extrinsics):')
    print(f'  Rotation: {node.depth_to_color_ext.rotation}')
    print(f'  Translation: {node.depth_to_color_ext.translation}')

# Check TF tree directly
print('\n=== TF FRAMES IN ORBBEC DRIVER ===')
frames_yaml = node.tf_buffer.all_frames_as_yaml()
print(frames_yaml)

node.destroy_node()
rclpy.shutdown()

os.killpg(os.getpgid(p_cam.pid), signal.SIGINT)
time.sleep(2.0)
try:
    os.killpg(os.getpgid(p_cam.pid), signal.SIGKILL)
except:
    pass
