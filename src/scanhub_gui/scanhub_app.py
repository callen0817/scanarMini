#!/usr/bin/env python3
"""
scanHUB Commercial Reality Capture OS v1.0 (scanR Platform)
Primary Desktop & Wearable HUD GUI Interface for scanR Golden Hardware Rig.
Optimized for VITURE Luma Ultra AR Glasses (Peripheral Side HUD Layout).

Feature Addition:
- InertialSense IG-2 Dual RTK Auto-Detection & Status Display:
  Autodetects GPS/RTK status from IG-2 dual-antenna module (/NavSatFix, /gps1/pos_vel, /gps1/info).
  Indoors (No Sky Visibility): Displays "● Dual RTK: INDOORS (NO FIX)" [Red].
  Outdoors (Sky Visibility): Displays "● Dual RTK: 3D FIX" [Yellow] or "● Dual RTK: RTK FIXED" [Green].
"""

import faulthandler
faulthandler.enable()

import sys
import os
import time
import math
import json
import re
import subprocess
import struct
import numpy as np
from datetime import datetime

# Default ROS 2 domain if not set
if "ROS_DOMAIN_ID" not in os.environ:
    os.environ["ROS_DOMAIN_ID"] = "0"

# PyQt6 / PyQt5 imports
try:
    from PyQt6.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout, 
                                 QPushButton, QLabel, QLineEdit, QComboBox, QTextEdit, 
                                 QStatusBar, QProgressBar, QMessageBox, QInputDialog,
                                 QFrame, QSplitter, QDialog, QSlider, QCheckBox, QFormLayout)
    from PyQt6.QtCore import QProcess, QTimer, Qt, QThread, pyqtSignal, QPointF, QPoint, QRect
    from PyQt6.QtGui import QFont, QPalette, QColor, QPainter, QPen, QBrush, QPolygonF, QPolygon, QImage, QPixmap
except ImportError:
    from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout, 
                                 QPushButton, QLabel, QLineEdit, QComboBox, QTextEdit, 
                                 QStatusBar, QProgressBar, QMessageBox, QInputDialog,
                                 QFrame, QSplitter, QDialog, QSlider, QCheckBox, QFormLayout)
    from PyQt5.QtCore import QProcess, QTimer, Qt, QThread, pyqtSignal, QPointF, QPoint, QRect
    from PyQt5.QtGui import QFont, QPalette, QColor, QPainter, QPen, QBrush, QPolygonF, QPolygon, QImage, QPixmap

# ROS 2 Python imports
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, PointCloud2, Imu, NavSatFix
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from cv_bridge import CvBridge

# Import local scanHUB engine modules
try:
    from storage_monitor import StorageMonitor
    from sensor_monitor import SensorMonitor
    from diagnostic_recorder import DiagnosticRecorder
except ImportError:
    from .storage_monitor import StorageMonitor
    from .sensor_monitor import SensorMonitor
    from .diagnostic_recorder import DiagnosticRecorder


class Ros2BridgeWorker(QThread):
    """
    Background Thread running rclpy spin loop.
    Subscribes to:
    - /image_raw (RGB Camera Stream)
    - /rslidar_points (LiDAR Point Cloud)
    - /imu (IG-2 RTK Gyro & Accelerometer)
    - /NavSatFix & /gps1/pos_vel (Dual RTK GPS Fix Status)
    - /odom_ins_enu & /aft_mapped_to_init (Odometry)
    Emits Qt signals for live HUD rendering.
    """
    image_received = pyqtSignal(QImage)
    pointcloud_received = pyqtSignal(np.ndarray, np.ndarray) # (world_pts, raw_body_pts)
    registered_pointcloud_received = pyqtSignal(np.ndarray) # (registered_world_pts)
    pose_received = pyqtSignal(float, float, float) # x, y, yaw
    rtk_status_received = pyqtSignal(int, str) # status_code, status_label
    sensor_rates_received = pyqtSignal(dict) # live sensor status summary
    motion_stability_received = pyqtSignal(bool, float) # (is_stable, stable_duration_sec)
    livo_hud_received = pyqtSignal(str, float, int, float) # (status, proc_ms, matched_pts, vel)
    rotation_progress_received = pyqtSignal(float) # accumulated_deg
    rotation_completed_received = pyqtSignal() # rotation ritual complete

    def __init__(self, diag_recorder=None):
        super().__init__()
        self.running = True
        self.bridge = CvBridge()
        self.diag_recorder = diag_recorder
        
        self.in_rotation_ritual = False
        self.accumulated_rotation_deg = 0.0
        self.last_imu_gyro_time = None
        
        self.last_imu_time = None
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.current_yaw = 0.0
        self.current_quat = [0.0, 0.0, 0.0, 1.0] # [qx, qy, qz, qw]
        self.gyro_bias_z = 0.0
        
        self.using_slam_odom = False
        self.last_slam_time = 0.0
        self.last_color_frame = None
        self.last_depth_frame = None

        # M4.3 Real-Time Topic Timestamps & Health Monitor
        self.last_lidar_time = 0.0
        self.last_camera_time = 0.0
        self.last_left_ir_time = 0.0
        self.last_rgb_time = 0.0
        self.last_depth_time = 0.0
        self.last_imu_msg_time = 0.0
        self.last_slam_msg_time = 0.0
        self.last_camera_stamp_sec = None
        self.last_lidar_stamp_sec = None
        self.last_imu_stamp_sec = None
        self.last_ax = 0.0
        self.last_ay = 0.0
        self.last_az = 9.81

        self.cnt_camera = 0
        self.cnt_lidar = 0
        self.cnt_imu = 0
        self.cnt_slam = 0
        self.cnt_livo_diag = 0
        self.stable_motion_start = None
        self._last_image_emit = 0.0

        # Live Interactive Extrinsic Calibration Parameters (Roll=-135°, Pitch=0°, Yaw=0°, XYZ=0)
        self.calib_raw_mode = False
        self.calib_roll_deg = -135.0
        self.calib_pitch_deg = 0.0
        self.calib_yaw_deg = 0.0
        self.calib_x_m = 0.0
        self.calib_y_m = 0.0
        self.calib_z_m = 0.0
        self.load_extrinsic_config()

    def load_extrinsic_config(self):
        cfg_path = "/home/scanar/scanarMini/config/calibrated_extrinsics.json"
        if os.path.exists(cfg_path):
            try:
                with open(cfg_path, "r") as f:
                    data = json.load(f)
                    self.calib_roll_deg = float(data.get("roll_deg", 0.0))
                    self.calib_pitch_deg = float(data.get("pitch_deg", -45.0))
                    self.calib_yaw_deg = float(data.get("yaw_deg", 90.0))
                    self.calib_x_m = float(data.get("x_offset_m", 0.0))
                    self.calib_y_m = float(data.get("y_offset_m", 0.035814))
                    self.calib_z_m = float(data.get("z_offset_m", -0.074422))
                    self.calib_raw_mode = bool(data.get("raw_mode", False))
            except Exception as e:
                print(f"[EXTRINSIC CFG ERROR] Failed to load config: {e}")

    def run(self):
        print(f"\n[STAGE 2] Ros2BridgeWorker.run() STARTED in thread ID {int(QThread.currentThreadId())}")
        if not rclpy.ok():
            print("[STAGE 2] Initializing rclpy...")
            rclpy.init()

        node = Node("scanhub_ros2_gui_bridge")
        domain_id = os.environ.get("ROS_DOMAIN_ID", "default")
        print(f"[STAGE 1] Created ROS 2 Node '{node.get_name()}' on ROS_DOMAIN_ID={domain_id}")

        def trigger_combined_emit():
            now = time.time()
            if now - self._last_image_emit < 0.045: # Cap HUD video stream to 22 FPS
                return
            self._last_image_emit = now

            import cv2
            color_src = self.last_color_frame
            if color_src is None:
                return

            h_target, w_target = 240, 320
            color_resized = cv2.resize(color_src, (w_target, h_target))

            if self.last_depth_frame is not None:
                depth_resized = cv2.resize(self.last_depth_frame, (w_target, h_target))
            else:
                depth_resized = np.zeros((h_target, w_target, 3), dtype=np.uint8)
                cv2.putText(depth_resized, "DEPTH: STANDBY", (40, h_target // 2), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (139, 148, 158), 2)
            
            combined = np.ascontiguousarray(np.hstack((color_resized, depth_resized)))
            h, w, ch = combined.shape
            bytes_per_line = ch * w
            fmt_rgb = QImage.Format_RGB888 if hasattr(QImage, 'Format_RGB888') else QImage.Format.Format_RGB888
            qimg = QImage(combined.data, w, h, bytes_per_line, fmt_rgb).copy()
            self.image_received.emit(qimg)

        def rgb_color_callback(msg: Image):
            self.cnt_camera += 1
            sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            now = time.time()
            self.last_camera_time = now
            self.last_rgb_time = now
            self.last_camera_stamp_sec = sec
            if self.cnt_camera % 30 == 1:
                print(f"[STAGE 5] RGB Camera Callback #{self.cnt_camera} | {msg.width}x{msg.height} {msg.encoding} | stamp={sec:.3f}")
            try:
                if self.diag_recorder:
                    self.diag_recorder.log_camera(sec)

                raw_bytes = bytes(msg.data)
                enc = msg.encoding.lower()
                if "yuv" in enc or "yuyv" in enc:
                    yuv_data = np.frombuffer(raw_bytes, dtype=np.uint8).reshape((msg.height, msg.width, 2))
                    import cv2
                    bgr = cv2.cvtColor(yuv_data, cv2.COLOR_YUV2BGR_YUY2)
                    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
                elif "bgr8" in enc:
                    bgr = np.frombuffer(raw_bytes, dtype=np.uint8).reshape((msg.height, msg.width, 3))
                    import cv2
                    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
                elif "rgb8" in enc:
                    rgb = np.frombuffer(raw_bytes, dtype=np.uint8).reshape((msg.height, msg.width, 3)).copy()
                else:
                    rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
                
                self.last_color_frame = rgb
                trigger_combined_emit()
            except Exception as e:
                print(f"[STAGE 5 ERROR] RGB Camera Image Conversion Error: {e}")

        def depth_callback(msg: Image):
            now = time.time()
            self.last_camera_time = now
            self.last_depth_time = now
            try:
                raw_bytes = bytes(msg.data)
                depth_img = np.frombuffer(raw_bytes, dtype=np.uint16).reshape((msg.height, msg.width))
                depth_scaled = np.clip((depth_img.astype(np.float32) - 300.0) / (3000.0 - 300.0) * 255.0, 0, 255).astype(np.uint8)
                import cv2
                heatmap = cv2.applyColorMap(depth_scaled, cv2.COLORMAP_JET)
                heatmap_rgb = cv2.cvtColor(heatmap, cv2.COLOR_BGR2RGB)
                self.last_depth_frame = heatmap_rgb
                trigger_combined_emit()
            except Exception as e:
                pass

        def lidar_callback(msg: PointCloud2):
            self.cnt_lidar += 1
            now = time.time()
            self.last_lidar_time = now
            sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.last_lidar_stamp_sec = sec
            num_points = msg.width * msg.height
            if self.cnt_lidar % 30 == 1:
                print(f"[STAGE 5] LiDAR Callback #{self.cnt_lidar} | pts={num_points} | stamp={sec:.3f}")
            if self.diag_recorder:
                self.diag_recorder.log_lidar(sec, num_points)

        def registered_cloud_callback(msg: PointCloud2):
            """FAST-LIVO2 Registered Cloud Callback: Points are ALREADY in the SLAM World Frame."""
            try:
                num_points = msg.width * msg.height
                if num_points == 0 or msg.point_step < 12:
                    return

                raw_bytes = bytes(msg.data)
                dt = np.dtype({
                    'names': ['x', 'y', 'z'],
                    'formats': ['<f4', '<f4', '<f4'],
                    'offsets': [0, 4, 8],
                    'itemsize': msg.point_step
                })
                pts = np.frombuffer(raw_bytes, dtype=dt)
                valid = np.isfinite(pts['x']) & np.isfinite(pts['y']) & np.isfinite(pts['z'])
                pts_valid = pts[valid]
                if len(pts_valid) == 0:
                    return

                rel_z = pts_valid['z'] - self.current_z
                h_mask = (rel_z >= -0.5) & (rel_z < 1.0)
                pts_filtered = pts_valid[h_mask]
                if len(pts_filtered) > 0:
                    stride = max(1, len(pts_filtered) // 500)
                    sampled = pts_filtered[::stride]
                    xy = np.column_stack((sampled['x'], sampled['y'])).astype(np.float32)
                    self.registered_pointcloud_received.emit(xy)
            except Exception as e:
                print(f"[STAGE 5 ERROR] Registered Cloud Error: {e}")

        def imu_callback(msg: Imu):
            self.cnt_imu += 1
            now = time.time()
            sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.last_imu_msg_time = now
            self.last_imu_stamp_sec = sec
            if self.cnt_imu % 100 == 1:
                print(f"[STAGE 5] IMU Callback #{self.cnt_imu} | stamp={sec:.3f}")
            
            gx, gy, gz = msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z
            ax, ay, az = msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z
            
            self.last_ax = ax
            self.last_ay = ay
            self.last_az = az
            
            if self.diag_recorder:
                self.diag_recorder.log_imu(sec, gx, gy, gz, ax, ay, az)

            gyro_mag = math.sqrt(gx*gx + gy*gy + gz*gz)
            accel_mag = math.sqrt(ax*ax + ay*ay + az*az)

            # NavVis Automatic 90-Degree Rotation Ritual Integration
            if self.in_rotation_ritual:
                now_t = time.time()
                if self.last_imu_gyro_time is not None:
                    dt = now_t - self.last_imu_gyro_time
                    if 0.0 < dt < 0.5:
                        gz_deg = abs(gz) * (180.0 / math.pi)
                        if gz_deg > 3.0: # threshold to filter vibration
                            self.accumulated_rotation_deg += gz_deg * dt
                            self.rotation_progress_received.emit(self.accumulated_rotation_deg)
                            if self.accumulated_rotation_deg >= 85.0 and abs(gz) < 0.2:
                                self.in_rotation_ritual = False
                                self.rotation_completed_received.emit()
                self.last_imu_gyro_time = now_t

            is_gyro_quiet = gyro_mag < 0.25
            is_accel_quiet = abs(accel_mag - 9.81) < 1.5

            if is_gyro_quiet and is_accel_quiet:
                if self.stable_motion_start is None:
                    self.stable_motion_start = now
                stable_dur = now - self.stable_motion_start
                self.motion_stability_received.emit(True, stable_dur)
            else:
                self.stable_motion_start = None
                self.motion_stability_received.emit(False, 0.0)



        def slam_odom_callback(msg: Odometry):
            self.cnt_slam += 1
            now = time.time()
            sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.using_slam_odom = True
            self.last_slam_time = now
            self.last_slam_msg_time = now

            px = msg.pose.pose.position.x
            py = msg.pose.pose.position.y
            pz = msg.pose.pose.position.z
            if math.isnan(px) or math.isnan(py) or math.isnan(pz) or math.isinf(px) or math.isinf(py) or math.isinf(pz):
                return

            self.current_x = px
            self.current_y = py
            self.current_z = pz
            
            qx = msg.pose.pose.orientation.x
            qy = msg.pose.pose.orientation.y
            qz = msg.pose.pose.orientation.z
            qw = msg.pose.pose.orientation.w
            norm_q = qx*qx + qy*qy + qz*qz + qw*qw
            if abs(norm_q - 1.0) < 0.1 and not any(math.isnan(v) for v in (qx, qy, qz, qw)):
                self.current_quat = [qx, qy, qz, qw]
                siny_cosp = 2.0 * (qw * qz + qx * qy)
                cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
                self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

            if self.cnt_slam % 20 == 1:
                print(f"[STAGE 5] SLAM Odom Callback #{self.cnt_slam} | X={self.current_x:.2f} Y={self.current_y:.2f} Yaw={math.degrees(self.current_yaw):.1f}° | stamp={sec:.3f}")

            if self.diag_recorder:
                vx = msg.twist.twist.linear.x
                vy = msg.twist.twist.linear.y
                vz = msg.twist.twist.linear.z
                self.diag_recorder.log_slam(sec, self.current_x, self.current_y, self.current_z, qx, qy, qz, qw, 0.0, 0.0, self.current_yaw, vx, vy, vz)

            self.pose_received.emit(self.current_x, self.current_y, self.current_yaw)

        def livo_diag_callback(msg: String):
            self.cnt_livo_diag += 1
            try:
                s = msg.data.strip()
                if not (s.startswith('{') and s.endswith('}')):
                    i0 = s.find('{')
                    i1 = s.rfind('}')
                    if i0 != -1 and i1 != -1 and i1 > i0:
                        s = s[i0:i1+1]
                    else:
                        return
                s = re.sub(r'\bnan\b', '0.0', s, flags=re.IGNORECASE)
                s = re.sub(r'\binf\b', '0.0', s, flags=re.IGNORECASE)
                data = json.loads(s)
                f_status = data.get("tracking_status", "UNKNOWN")
                f_proc = data.get("proc_time_ms", 0.0)
                f_matched = data.get("features_matched", 0)
                f_vel = data.get("estimated_velocity_m_s", 0.0)
                self.livo_hud_received.emit(f_status, f_proc, f_matched, f_vel)
                if self.diag_recorder:
                    self.diag_recorder.log_slam_diagnostics(
                        data.get("frame_idx", 0),
                        data.get("lidar_header_time", 0.0),
                        data.get("lidar_receive_time", 0.0),
                        data.get("slam_publish_time", 0.0),
                        data.get("publish_latency_ms", 0.0),
                        f_proc,
                        data.get("points_raw_count", 0),
                        data.get("points_filtered_count", 0),
                        f_matched,
                        data.get("vio_count", 0),
                        data.get("lio_residual_m", 0.0),
                        f_vel,
                        data.get("estimated_accel_m_s2", 0.0),
                        data.get("delta_translation_m", 0.0),
                        f_status
                    )
            except Exception as e:
                print(f"[STAGE 5 ERROR] LIVO Diag Parsing Error: {e} | data='{msg.data}'")

        # Stage 3: Subscription Creation & Topic Publisher Inspection
        sub_color = node.create_subscription(Image, "/camera/color/image_raw", rgb_color_callback, qos_profile_sensor_data)
        print(f"[STAGE 3] Subscribed: /camera/color/image_raw (Pure RGB) | Publishers on Graph: {node.count_publishers('/camera/color/image_raw')}")

        sub_depth = node.create_subscription(Image, "/camera/depth/image_raw", depth_callback, qos_profile_sensor_data)
        print(f"[STAGE 3] Subscribed: /camera/depth/image_raw | Publishers on Graph: {node.count_publishers('/camera/depth/image_raw')}")

        sub_lidar = node.create_subscription(PointCloud2, "/rslidar_points", lidar_callback, qos_profile_sensor_data)
        print(f"[STAGE 3] Subscribed: /rslidar_points | Publishers on Graph: {node.count_publishers('/rslidar_points')}")

        sub_registered = node.create_subscription(PointCloud2, "/cloud_registered", registered_cloud_callback, qos_profile_sensor_data)
        print(f"[STAGE 3] Subscribed: /cloud_registered | Publishers on Graph: {node.count_publishers('/cloud_registered')}")

        sub_imu = node.create_subscription(Imu, "/rslidar_imu_data", imu_callback, qos_profile_sensor_data)
        print(f"[STAGE 3] Subscribed: /rslidar_imu_data | Publishers on Graph: {node.count_publishers('/rslidar_imu_data')}")

        sub_slam = node.create_subscription(Odometry, "/aft_mapped_to_init", slam_odom_callback, qos_profile_sensor_data)
        print(f"[STAGE 3] Subscribed: /aft_mapped_to_init | Publishers on Graph: {node.count_publishers('/aft_mapped_to_init')}")

        node.create_subscription(String, "/LIVO2/frame_diagnostics", livo_diag_callback, 10)
        print(f"[STAGE 3] Subscribed: /LIVO2/frame_diagnostics | Publishers on Graph: {node.count_publishers('/LIVO2/frame_diagnostics')}")

        last_health_report = 0.0
        while self.running and rclpy.ok():
            try:
                rclpy.spin_once(node, timeout_sec=0.05)
            except Exception as e:
                print(f"[STAGE 4 ERROR] rclpy.spin_once Exception: {e}")

            now = time.time()
            if now - last_health_report >= 2.0:
                last_health_report = now
                rgb_dt = now - self.last_rgb_time if self.last_rgb_time > 0 else 999.0
                depth_dt = now - self.last_depth_time if self.last_depth_time > 0 else 999.0
                lid_dt = now - self.last_lidar_time if self.last_lidar_time > 0 else 999.0
                imu_dt = now - self.last_imu_msg_time if self.last_imu_msg_time > 0 else 999.0
                slam_dt = now - self.last_slam_msg_time if self.last_slam_msg_time > 0 else 999.0

                print(f"[STAGE 2 & 4] Worker Loop Active | Callbacks Total: RGB={self.cnt_camera}, LiDAR={self.cnt_lidar}, IMU={self.cnt_imu}, SLAM={self.cnt_slam}")
                print(f"[STAGE 2 & 4] Time Since Last Msg: RGB={rgb_dt:.1f}s, Depth={depth_dt:.1f}s, LiDAR={lid_dt:.1f}s, IMU={imu_dt:.1f}s, SLAM={slam_dt:.1f}s")

                summary = {
                    "lidar": lid_dt < 2.5,
                    "camera": rgb_dt < 2.5,
                    "imu": imu_dt < 2.5,
                    "slam": slam_dt < 2.5,
                }
                self.sensor_rates_received.emit(summary)

                # M9 Hardware Trigger Slaved State (Airy SYNC_OUT1 Pin 2 -> Gemini VSYNC_IN Pin 6)
                if self.last_camera_stamp_sec is not None and self.last_lidar_stamp_sec is not None:
                    dt_cam_lidar = (self.last_camera_stamp_sec - self.last_lidar_stamp_sec) * 1000.0
                    if abs(dt_cam_lidar) < 100.0 and (rgb_dt < 2.0 and lid_dt < 2.0):
                        sync_code = 1  # Green: Physical Hardware Locked
                        sync_label = f"HARDWARE LOCKED ({dt_cam_lidar:+.1f}ms)"
                    else:
                        sync_code = 0  # Amber: Sync active but high latency
                        sync_label = f"LOCKED (Offset {dt_cam_lidar:+.1f}ms)"
                elif rgb_dt < 2.0:
                    sync_code = 0  # Amber: Stream active
                    sync_label = "STREAMING"
                else:
                    sync_code = -1  # Red: No Sync
                    sync_label = "NO SYNC"
                self.rtk_status_received.emit(sync_code, sync_label)

        node.destroy_node()

    def stop(self):
        self.running = False
        self.wait()


class CameraPreviewCanvas(QFrame):
    """
    Live RGB Camera Stream Preview Frame (/camera/color/image_raw).
    """
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setStyleSheet("background-color: #000000; border: 1px solid #30363d; border-radius: 6px;")
        self.current_image = None

    def update_frame(self, qimg: QImage):
        self.current_image = qimg
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        try:
            w, h = self.width(), self.height()
            painter.fillRect(0, 0, w, h, QColor("#000000"))

            if self.current_image and not self.current_image.isNull():
                pw = self.current_image.width()
                ph = self.current_image.height()
                if pw > 0 and ph > 0:
                    scale = min(w / pw, h / ph)
                    tw = int(pw * scale)
                    th = int(ph * scale)
                    px = (w - tw) // 2
                    py = (h - th) // 2
                    painter.drawImage(QRect(px, py, tw, th), self.current_image)
                
                painter.setPen(QColor("#3fb950"))
                painter.setFont(QFont("Segoe UI", 8, QFont.Weight.Bold))
                painter.drawText(10, 18, "LIVE STREAMS: RGB COLOR (LEFT) | DEPTH HEATMAP (RIGHT)")
        finally:
            painter.end()


class NavVisARPointMapCanvas(QFrame):
    """
    Side-HUD NavVis Point Cloud Slice & Trajectory Map Canvas.
    - Pure #000000 Black Background (OLED Optical Passthrough for VITURE AR Glasses).
    - Location Arrow directly aligned with live IG-2 RTK IMU orientation.
    - Path of Travel Line (#19ffff cyan line).
    - Real-Time 2D Point Cloud Slice accumulation from /rslidar_points.
    """
    def __init__(self, parent=None):
        super().__init__(parent)
        import threading
        self._map_lock = threading.Lock()
        self.setStyleSheet("background-color: #000000; border: 1px solid #30363d; border-radius: 6px;")
        
        self.trajectory_path = []  # List of (x, y, yaw)
        self.accumulated_points = [] # List of (x, y)
        self.voxel_map = set() # 5cm spatial grid cell key set (gx, gy)
        self.flash_anchors = [] # List of {"label": "w00", "x": float, "y": float}
        self.is_active = False
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.view_radius_m = 30.0  # Default 30m radius (60m x 60m viewport)
        self.scale = 10.0          # Dynamic scale (pixels per meter)
        self.total_distance = 0.0
        self.rtk_label = "INDOORS (NO FIX)"
        self.rtk_code = -1
        
        # Floor Plan Point Cloud Map Alignment / Calibration Defaults
        self.initial_yaw = None
        self.auto_align_start_heading = True
        self.map_yaw_offset_deg = 0.0
        self.arrow_yaw_offset_deg = -90.0 # 90° clockwise rotation so forward direction is UP
        self.horizontal_flip = True     # Horizontal flip enabled by default
        self.map_x_offset_m = 0.0
        self.map_y_offset_m = 0.0
        
        # Gravity Leveling Map View Offsets (isolated from physical sensor extrinsics)
        self.gravity_roll_deg = 0.0
        self.gravity_pitch_deg = 0.0

        # Live FAST-LIVO2 HUD Line Status
        self.livo_status = "READY"
        self.livo_proc_ms = 0.0
        self.livo_matched_pts = 0
        self.livo_vel = 0.0
        self.pause_map_accumulation = False
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)

    def wheelEvent(self, event):
        """Smooth Mouse Wheel Zooming (5m to 150m radius)."""
        delta = event.angleDelta().y()
        if delta > 0:
            self.view_radius_m = max(5.0, self.view_radius_m * 0.85)
        elif delta < 0:
            self.view_radius_m = min(150.0, self.view_radius_m * 1.15)
        self.update()

    def zoom_in(self):
        self.view_radius_m = max(5.0, self.view_radius_m * 0.75)
        self.update()

    def zoom_out(self):
        self.view_radius_m = min(150.0, self.view_radius_m * 1.35)
        self.update()

    def set_view_radius(self, radius_m: float):
        self.view_radius_m = radius_m
        self.update()

    def fit_map(self):
        with self._map_lock:
            pts_snap = list(self.accumulated_points)
        if pts_snap:
            max_dist = 10.0
            for wx, wy in pts_snap:
                d = math.hypot(wx - self.current_x, wy - self.current_y)
                if d > max_dist:
                    max_dist = d
            self.view_radius_m = min(150.0, max(10.0, max_dist * 1.15))
        else:
            self.view_radius_m = 30.0
        self.update()

    def clear_map(self):
        """Clear persistent SLAM world map voxels."""
        with self._map_lock:
            self.accumulated_points.clear()
            self.voxel_map.clear()
        self.initial_yaw = None
        self.update()

    def update_livo_hud(self, status: str, proc_ms: float, matched_pts: int, vel: float):
        self.livo_status = status
        self.livo_proc_ms = proc_ms
        self.livo_matched_pts = matched_pts
        self.livo_vel = vel
        self.update()

    def reset_map(self):
        with self._map_lock:
            self.trajectory_path.clear()
            self.accumulated_points.clear()
            self.voxel_map.clear()
            self.flash_anchors.clear()
        self.is_active = False
        self.initial_yaw = None
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.total_distance = 0.0
        self.update()

    def add_flash_anchor(self, label: str, x: float, y: float):
        with self._map_lock:
            self.flash_anchors.append({"label": label, "x": x, "y": y})
        self.update()

    def update_registered_points(self, registered_pts_arr: np.ndarray):
        """Authoritative FAST-LIVO2 Registered World Map (Emerald Green) - Fixed World Coordinates."""
        if self.pause_map_accumulation:
            with self._map_lock:
                self.accumulated_points.clear()
                self.voxel_map.clear()
            self.update()
            return

        if registered_pts_arr.size > 0:
            pts_list = registered_pts_arr.tolist()
            with self._map_lock:
                for wx, wy in pts_list:
                    grid_key = (int(wx * 10.0), int(wy * 10.0)) # 10cm spatial voxel cell
                    if grid_key not in self.voxel_map:
                        self.voxel_map.add(grid_key)
                        self.accumulated_points.append((wx, wy))

                if len(self.accumulated_points) > 8000:
                    self.accumulated_points = self.accumulated_points[-8000:]
                    self.voxel_map = set((int(wx * 10.0), int(wy * 10.0)) for wx, wy in self.accumulated_points)

        self.update()

    def update_pose(self, x: float, y: float, yaw: float):
        with self._map_lock:
            if self.auto_align_start_heading and self.initial_yaw is None:
                self.initial_yaw = yaw
                # Auto-align map orientation so wherever the camera/lidar is facing at scan start maps directly to straight UP (0° on screen)
                self.map_yaw_offset_deg = math.degrees(-yaw) - self.arrow_yaw_offset_deg

            if self.trajectory_path:
                lx, ly, _ = self.trajectory_path[-1]
                dist = math.hypot(x - lx, y - ly)
                self.total_distance += dist
                if dist > 0.04: # Only record trajectory point when moved > 4cm
                    self.trajectory_path.append((x, y, yaw))
                    if len(self.trajectory_path) > 3000:
                        self.trajectory_path = self.trajectory_path[-3000:]
            else:
                self.trajectory_path.append((x, y, yaw))

        self.current_x = x
        self.current_y = y
        self.current_yaw = yaw

        self.update()

    def set_rtk_status(self, code: int, label: str):
        self.rtk_code = code
        self.rtk_label = label
        self.update()

    def paintEvent(self, event):
        import time
        t_start = time.time()
        painter = QPainter(self)
        try:
            painter.setRenderHint(QPainter.RenderHint.Antialiasing)

            w, h = self.width(), self.height()
            cx, cy = w // 2, h // 2

            # Dynamic Scale Calculation based on View Radius (Pixels per Meter)
            min_dim = min(w, h)
            self.scale = (min_dim * 0.45) / max(1.0, float(self.view_radius_m))

            # Pure #000000 Background (OLED Passthrough)
            painter.fillRect(0, 0, w, h, QColor("#000000"))

            # Draw Concentric Range Rings (10m, 20m, 30m, 50m)
            pen_ring = QPen(QColor("#21262d"), 1.0, Qt.PenStyle.DashLine)
            painter.setPen(pen_ring)
            font_ring = QFont("Monospace", 7)
            painter.setFont(font_ring)

            for ring_m in [10.0, 20.0, 30.0, 50.0, 80.0]:
                r_px = ring_m * self.scale
                if r_px < min_dim * 0.9:
                    painter.drawEllipse(QPointF(cx, cy), r_px, r_px)
                    painter.setPen(QColor("#484f58"))
                    painter.drawText(int(cx + r_px + 4), int(cy - 2), f"{int(ring_m)}m")
                    painter.setPen(pen_ring)

            def world_to_screen(wx, wy):
                if wx is None or wy is None or math.isnan(wx) or math.isnan(wy) or math.isinf(wx) or math.isinf(wy):
                    return -9999, -9999
                if self.current_x is None or self.current_y is None or math.isnan(self.current_x) or math.isnan(self.current_y):
                    return -9999, -9999
                    
                # 1. Apply translation offsets
                tx = wx + self.map_x_offset_m
                ty = wy + self.map_y_offset_m
                
                # 2. Apply yaw rotation offset around current operator position
                yaw_rad = math.radians(self.map_yaw_offset_deg)
                cos_y = math.cos(yaw_rad)
                sin_y = math.sin(yaw_rad)
                
                dx = tx - self.current_x
                dy = ty - self.current_y
                
                # Rotated coordinates (CCW)
                rx = dx * cos_y - dy * sin_y
                ry = dx * sin_y + dy * cos_y
                
                # Convert to screen frame (with horizontal flip)
                scale = max(0.1, float(self.scale))
                try:
                    if self.horizontal_flip:
                        sx = int(cx + (ry * scale))
                    else:
                        sx = int(cx - (ry * scale))
                    sy = int(cy - (rx * scale))
                    return sx, sy
                except (ValueError, OverflowError):
                    return -9999, -9999

            with self._map_lock:
                points_snapshot = list(self.accumulated_points)
                path_snapshot = list(self.trajectory_path)
                anchors_snapshot = list(self.flash_anchors)
                curr_dist = self.total_distance

            # 1. Persistent Accumulated World Map (#38d9a9 Emerald Green) - Vectorized Fast Batch
            if points_snapshot:
                pts = np.array(points_snapshot, dtype=np.float32)
                dx = pts[:, 0] + self.map_x_offset_m - self.current_x
                dy = pts[:, 1] + self.map_y_offset_m - self.current_y
                yaw_rad = math.radians(self.map_yaw_offset_deg)
                cos_y = math.cos(yaw_rad)
                sin_y = math.sin(yaw_rad)
                rx = dx * cos_y - dy * sin_y
                ry = dx * sin_y + dy * cos_y
                scale = max(0.1, float(self.scale))
                if self.horizontal_flip:
                    sx = (cx + ry * scale).astype(np.int32)
                else:
                    sx = (cx - ry * scale).astype(np.int32)
                sy = (cy - rx * scale).astype(np.int32)
                
                mask = (sx >= 0) & (sx <= w) & (sy >= 0) & (sy <= h)
                sx_v = sx[mask]
                sy_v = sy[mask]
                
                if len(sx_v) > 0:
                    pen_pts = QPen(QColor("#38d9a9"), 2.0, Qt.PenStyle.SolidLine)
                    painter.setPen(pen_pts)
                    for px, py in zip(sx_v, sy_v):
                        painter.drawPoint(int(px), int(py))

            # 2. Path of Travel Line (#19ffff Cyan Line)
            if len(path_snapshot) > 1:
                pen_path = QPen(QColor("#19ffff"), 2, Qt.PenStyle.SolidLine)
                painter.setPen(pen_path)
                for i in range(len(path_snapshot) - 1):
                    p1 = world_to_screen(path_snapshot[i][0], path_snapshot[i][1])
                    p2 = world_to_screen(path_snapshot[i+1][0], path_snapshot[i+1][1])
                    if p1[0] != -9999 and p2[0] != -9999:
                        painter.drawLine(p1[0], p1[1], p2[0], p2[1])

            # 3. FARO Flash Scan Anchors (Amber Gold dots + 'w00', 'w01' numerical labels)
            if anchors_snapshot:
                font_anchor = QFont("Segoe UI", 8, QFont.Weight.Bold)
                painter.setFont(font_anchor)
                
                for anchor in anchors_snapshot:
                    ax, ay = anchor["x"], anchor["y"]
                    label = anchor["label"]
                    sx, sy = world_to_screen(ax, ay)
                    
                    if 0 <= sx <= w and 0 <= sy <= h:
                        # Draw Gold Anchor Dot
                        painter.setPen(QPen(QColor("#ffffff"), 1.5))
                        painter.setBrush(QBrush(QColor("#ffb700")))
                        painter.drawEllipse(QPointF(sx, sy), 5.0, 5.0)

                        # Draw Dark Label Tag
                        painter.setPen(QPen(QColor("#ffb700"), 1))
                        painter.setBrush(QBrush(QColor("#161b22")))
                        painter.drawRoundedRect(sx + 8, sy - 14, 28, 14, 3, 3)

                        # Draw Label Text (w00, w01...)
                        painter.setPen(QColor("#ffffff"))
                        painter.drawText(sx + 11, sy - 3, label)

            # 4. Location & Heading Arrow Aligned with Live IMU / SLAM (#3fb950)
            sx, sy = world_to_screen(self.current_x, self.current_y)
            arrow_yaw_rad = self.current_yaw + math.radians(self.arrow_yaw_offset_deg)
            fwd_x = self.current_x + math.cos(arrow_yaw_rad)
            fwd_y = self.current_y + math.sin(arrow_yaw_rad)
            sx_fwd, sy_fwd = world_to_screen(fwd_x, fwd_y)

            if sx != -9999 and sy != -9999 and sx_fwd != -9999 and sy_fwd != -9999:
                painter.save()
                painter.translate(sx, sy)
                
                vx = sx_fwd - sx
                vy = sy_fwd - sy
                screen_heading_deg = math.degrees(math.atan2(vx, -vy))
                painter.rotate(screen_heading_deg)

                arrow_poly = QPolygonF([
                    QPointF(0, -14),
                    QPointF(-7, 10),
                    QPointF(0, 5),
                    QPointF(7, 10)
                ])

                painter.setPen(QPen(QColor("#ffffff"), 1))
                painter.setBrush(QBrush(QColor("#3fb950")))
                painter.drawPolygon(arrow_poly)
                painter.restore()

            # Canvas Title & Diagnostics Overlay
            painter.setPen(QColor("#38d9a9"))
            painter.setFont(QFont("Segoe UI", 9, QFont.Weight.Bold))
            painter.drawText(10, 18, "SLAM AUTHORITATIVE REALITY CAPTURE")

            # Live SLAM HUD Line
            livo_color = "#3fb950" if ("GOOD" in self.livo_status or "READY" in self.livo_status) else "#f85149"
            painter.setPen(QColor(livo_color))
            hud_str = f"SLAM: {self.livo_status}  |  LAT: {self.livo_proc_ms:.0f}ms  |  FEATURES: {self.livo_matched_pts}  |  VEL: {self.livo_vel:.2f} m/s"
            painter.drawText(10, 50, hud_str)

            # RTK Overlay Badge on Canvas (Adapted for Hardware Sync)
            rtk_color = "#f85149" if self.rtk_code == -1 else ("#d29922" if self.rtk_code == 0 else "#3fb950")
            painter.setPen(QColor(rtk_color))
            painter.setFont(QFont("Segoe UI", 8, QFont.Weight.Bold))
            painter.drawText(w - 140, 18, f"SYNC: {self.rtk_label}")

            # Telemetry Text & Diagnostics Overlay
            painter.setPen(QColor("#8b949e"))
            painter.setFont(QFont("Monospace", 8))
            deg_str = f"{(math.degrees(self.current_yaw) % 360):.1f}°"
            painter.drawText(10, h - 32, f"POSE SOURCE: SLAM (/aft_mapped_to_init)")
            painter.drawText(10, h - 20, f"POS: X={self.current_x:+.2f}m Y={self.current_y:+.2f}m | HEADING: {deg_str}")
            painter.drawText(10, h - 8, f"MAP: {len(points_snapshot)} voxels (5cm grid) | DIST: {curr_dist:.1f}m | ANCHORS: {len(anchors_snapshot)}")
        finally:
            painter.end()
            
        t_paint = time.time() - t_start
        if t_paint > 0.015:
            print(f"[PROFILE] Paint time: {t_paint*1000:.2f} ms")



class ClearPassthroughViewport(QFrame):
    """
    Main Center Window: COMPLETELY CLEAR / PURE BLACK (#000000).
    Ensures 100% unobstructed optical passthrough for VITURE AR Glasses while walking.
    """
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setStyleSheet("background-color: #000000; border: none;")

    def paintEvent(self, event):
        painter = QPainter(self)
        try:
            w, h = self.width(), self.height()
            painter.fillRect(0, 0, w, h, QColor("#000000"))
        finally:
            painter.end()


class ExtrinsicCalibrationDialog(QDialog):
    """
    Live Interactive 6-DOF Extrinsic Calibration (AIRY -> IG-2 IMU Frame).
    Allows real-time tuning of Roll, Pitch, Yaw and XYZ translation offsets with 0ms visual feedback.
    """
    def __init__(self, ros_worker, parent=None):
        super().__init__(parent)
        self.ros_worker = ros_worker
        self.setWindowTitle("🛠 Live Interactive Extrinsic Calibration (AIRY LiDAR → IG-2 IMU)")
        self.setFixedSize(540, 480)
        self.setStyleSheet("background-color: #0d1117; color: #e6edf3;")
        
        layout = QVBoxLayout(self)
        layout.setSpacing(12)
        
        title = QLabel("🛠 Live Extrinsic Alignment & CAD Tuning")
        title.setFont(QFont("Segoe UI", 11, QFont.Weight.Bold))
        title.setStyleSheet("color: #38d9a9;")
        layout.addWidget(title)
        
        info = QLabel("Adjust sliders to align live point cloud slice with room walls in real time.")
        info.setStyleSheet("color: #8b949e; font-size: 11px;")
        layout.addWidget(info)
        
        form = QFormLayout()
        form.setSpacing(8)
        
        # Raw Mode Checkbox
        self.raw_cb = QCheckBox("Raw Sensor Mode (Direct un-transformed /rslidar_points)")
        self.raw_cb.setChecked(self.ros_worker.calib_raw_mode)
        self.raw_cb.setStyleSheet("color: #f85149; font-weight: bold;")
        self.raw_cb.toggled.connect(self.on_raw_toggled)
        form.addRow("Mode:", self.raw_cb)
        
        # Roll Slider (-180 to +180 deg)
        self.roll_lbl = QLabel(f"{self.ros_worker.calib_roll_deg:.1f}°")
        self.roll_lbl.setStyleSheet("color: #19ffff; font-weight: bold;")
        self.roll_slider = QSlider(Qt.Orientation.Horizontal)
        self.roll_slider.setRange(-180, 180)
        self.roll_slider.setValue(int(self.ros_worker.calib_roll_deg))
        self.roll_slider.valueChanged.connect(self.on_roll_changed)
        form.addRow("Roll (X):", self.make_slider_box(self.roll_slider, self.roll_lbl))
        
        # Pitch Slider (-180 to +180 deg)
        self.pitch_lbl = QLabel(f"{self.ros_worker.calib_pitch_deg:.1f}°")
        self.pitch_lbl.setStyleSheet("color: #19ffff; font-weight: bold;")
        self.pitch_slider = QSlider(Qt.Orientation.Horizontal)
        self.pitch_slider.setRange(-180, 180)
        self.pitch_slider.setValue(int(self.ros_worker.calib_pitch_deg))
        self.pitch_slider.valueChanged.connect(self.on_pitch_changed)
        form.addRow("Pitch (Y):", self.make_slider_box(self.pitch_slider, self.pitch_lbl))
        
        # Yaw Slider (-180 to +180 deg)
        self.yaw_lbl = QLabel(f"{self.ros_worker.calib_yaw_deg:.1f}°")
        self.yaw_lbl.setStyleSheet("color: #19ffff; font-weight: bold;")
        self.yaw_slider = QSlider(Qt.Orientation.Horizontal)
        self.yaw_slider.setRange(-180, 180)
        self.yaw_slider.setValue(int(self.ros_worker.calib_yaw_deg))
        self.yaw_slider.valueChanged.connect(self.on_yaw_changed)
        form.addRow("Yaw (Z):", self.make_slider_box(self.yaw_slider, self.yaw_lbl))

        # X Offset Slider (-100cm to +100cm)
        self.x_lbl = QLabel(f"{self.ros_worker.calib_x_m * 100.0:.1f} cm")
        self.x_lbl.setStyleSheet("color: #ffb700; font-weight: bold;")
        self.x_slider = QSlider(Qt.Orientation.Horizontal)
        self.x_slider.setRange(-100, 100)
        self.x_slider.setValue(int(self.ros_worker.calib_x_m * 100.0))
        self.x_slider.valueChanged.connect(self.on_x_changed)
        form.addRow("X Offset (Left/Right):", self.make_slider_box(self.x_slider, self.x_lbl))

        # Y Offset Slider (-100cm to +100cm)
        self.y_lbl = QLabel(f"{self.ros_worker.calib_y_m * 100.0:.1f} cm")
        self.y_lbl.setStyleSheet("color: #ffb700; font-weight: bold;")
        self.y_slider = QSlider(Qt.Orientation.Horizontal)
        self.y_slider.setRange(-100, 100)
        self.y_slider.setValue(int(self.ros_worker.calib_y_m * 100.0))
        self.y_slider.valueChanged.connect(self.on_y_changed)
        form.addRow("Y Offset (Fwd/Back):", self.make_slider_box(self.y_slider, self.y_lbl))

        # Z Offset Slider (-100cm to +100cm)
        self.z_lbl = QLabel(f"{self.ros_worker.calib_z_m * 100.0:.1f} cm")
        self.z_lbl.setStyleSheet("color: #ffb700; font-weight: bold;")
        self.z_slider = QSlider(Qt.Orientation.Horizontal)
        self.z_slider.setRange(-100, 100)
        self.z_slider.setValue(int(self.ros_worker.calib_z_m * 100.0))
        self.z_slider.valueChanged.connect(self.on_z_changed)
        form.addRow("Z Offset (Up/Down):", self.make_slider_box(self.z_slider, self.z_lbl))

        layout.addLayout(form)
        
        btn_box = QHBoxLayout()
        reset_btn = QPushButton("↺ Reset CAD Baseline")
        reset_btn.setStyleSheet("background-color: #21262d; color: #e6edf3; font-weight: bold; padding: 6px 12px; border-radius: 4px;")
        reset_btn.clicked.connect(self.reset_cad_baseline)
        
        save_btn = QPushButton("💾 Save Calibrated Extrinsics")
        save_btn.setStyleSheet("background-color: #3fb950; color: #ffffff; font-weight: bold; padding: 6px 16px; border-radius: 4px;")
        save_btn.clicked.connect(self.save_extrinsics)
        
        btn_box.addWidget(reset_btn)
        btn_box.addWidget(save_btn)
        layout.addLayout(btn_box)

    def make_slider_box(self, slider: QSlider, label: QLabel):
        box = QWidget()
        l = QHBoxLayout(box)
        l.setContentsMargins(0, 0, 0, 0)
        l.addWidget(slider)
        l.addWidget(label)
        label.setFixedWidth(65)
        return box

    def on_raw_toggled(self, checked: bool):
        self.ros_worker.calib_raw_mode = checked
        if self.parent() and hasattr(self.parent(), 'map_canvas'):
            self.parent().map_canvas.update()

    def on_roll_changed(self, val: int):
        self.ros_worker.calib_roll_deg = float(val)
        self.roll_lbl.setText(f"{val:.1f}°")
        if self.parent() and hasattr(self.parent(), 'map_canvas'):
            self.parent().map_canvas.update()

    def on_pitch_changed(self, val: int):
        self.ros_worker.calib_pitch_deg = float(val)
        self.pitch_lbl.setText(f"{val:.1f}°")
        if self.parent() and hasattr(self.parent(), 'map_canvas'):
            self.parent().map_canvas.update()

    def on_yaw_changed(self, val: int):
        self.ros_worker.calib_yaw_deg = float(val)
        self.yaw_lbl.setText(f"{val:.1f}°")
        if self.parent() and hasattr(self.parent(), 'map_canvas'):
            self.parent().map_canvas.update()

    def on_x_changed(self, val: int):
        self.ros_worker.calib_x_m = val / 100.0
        self.x_lbl.setText(f"{val:.1f} cm")
        if self.parent() and hasattr(self.parent(), 'map_canvas'):
            self.parent().map_canvas.update()

    def on_y_changed(self, val: int):
        self.ros_worker.calib_y_m = val / 100.0
        self.y_lbl.setText(f"{val:.1f} cm")
        if self.parent() and hasattr(self.parent(), 'map_canvas'):
            self.parent().map_canvas.update()

    def on_z_changed(self, val: int):
        self.ros_worker.calib_z_m = val / 100.0
        self.z_lbl.setText(f"{val:.1f} cm")
        if self.parent() and hasattr(self.parent(), 'map_canvas'):
            self.parent().map_canvas.update()

    def reset_cad_baseline(self):
        self.raw_cb.setChecked(False)
        self.roll_slider.setValue(-135)
        self.pitch_slider.setValue(0)
        self.yaw_slider.setValue(0)
        self.x_slider.setValue(0)
        self.y_slider.setValue(0)
        self.z_slider.setValue(0)

    def save_extrinsics(self):
        cfg_path = "/home/scanar/scanarMini/config/calibrated_extrinsics.json"
        data = {
            "roll_deg": self.ros_worker.calib_roll_deg,
            "pitch_deg": self.ros_worker.calib_pitch_deg,
            "yaw_deg": self.ros_worker.calib_yaw_deg,
            "x_offset_m": self.ros_worker.calib_x_m,
            "y_offset_m": self.ros_worker.calib_y_m,
            "z_offset_m": self.ros_worker.calib_z_m,
            "raw_mode": self.ros_worker.calib_raw_mode
        }
        try:
            os.makedirs("/home/scanar/scanarMini/config", exist_ok=True)
            with open(cfg_path, "w") as f:
                json.dump(data, f, indent=2)
            QMessageBox.information(self, "Extrinsics Saved", f"Saved calibrated extrinsics to:\n{cfg_path}")
            self.accept()
        except Exception as e:
            QMessageBox.critical(self, "Error Saving", f"Failed to write config: {e}")


class ScanHUBOS(QWidget):
    def __init__(self):
        super().__init__()
        
        self.sensor_bringup_process = None
        self.mapping_process = None
        
        # State Tracking Flags
        self.is_calibrated = False
        self.is_initialized = False
        self.is_recording = False
        self.diagnostic_mode_enabled = False
        self.calib_ticks = 0
        self.accumulated_yaw = 0.0
        self.anchor_count = 0
        
        # System Monitors & Diagnostic Recorder
        self.storage_monitor = StorageMonitor(target_path="/home/scanar")
        self.sensor_monitor = SensorMonitor()
        self.sensor_monitor.start_ros_listener()
        
        self.diag_recorder = DiagnosticRecorder(base_dir="/home/scanar/scanarMini/diagnostics")
        
        # ROS 2 Threaded Worker Bridge
        self.ros_worker = Ros2BridgeWorker(diag_recorder=self.diag_recorder)
        self.ros_worker.rotation_progress_received.connect(self.update_rotation_ritual_ui)
        self.ros_worker.rotation_completed_received.connect(self.on_rotation_ritual_completed)

        # Live Scan Elapsed Clock Timer
        self.scan_elapsed_seconds = 0
        self.scan_clock_timer = QTimer()
        self.scan_clock_timer.timeout.connect(self.update_scan_clock_tick)
        
        self.init_ui()
        self.setup_timers()
        self.start_live_sensor_stack()

    def init_ui(self):
        self.setWindowTitle("scanHUB — Commercial Reality Capture OS (scanR Platform)")
        self.resize(1180, 780)
        
        palette = QPalette()
        palette.setColor(QPalette.ColorRole.Window, QColor(0, 0, 0))
        palette.setColor(QPalette.ColorRole.WindowText, QColor(230, 237, 243))
        palette.setColor(QPalette.ColorRole.Base, QColor(0, 0, 0))
        palette.setColor(QPalette.ColorRole.Text, QColor(230, 237, 243))
        palette.setColor(QPalette.ColorRole.Button, QColor(22, 27, 34))
        palette.setColor(QPalette.ColorRole.ButtonText, QColor(230, 237, 243))
        self.setPalette(palette)
        
        main_layout = QVBoxLayout()
        main_layout.setSpacing(0)
        main_layout.setContentsMargins(0, 0, 0, 0)
        
        # ─────────────────────────────────────────────────────────────────────────────
        # 1. HEADER BAR (#000000)
        # ─────────────────────────────────────────────────────────────────────────────
        header = QFrame()
        header.setFixedHeight(44)
        header.setStyleSheet("background-color: #000000; border-bottom: 1px solid #30363d;")
        header_layout = QHBoxLayout(header)
        header_layout.setContentsMargins(14, 0, 14, 0)
        header_layout.setSpacing(16)
        
        title = QLabel("scanHUB™ (scanarMini)")
        title.setFont(QFont("Segoe UI", 13, QFont.Weight.Bold))
        title.setStyleSheet("color: #e6edf3;")
        
        sub_title = QLabel("scanarMini Commercial Rig | Airy LiDAR & IMU + Gemini 336L Stereo IR + RGB")
        sub_title.setFont(QFont("Segoe UI", 10))
        sub_title.setStyleSheet("color: #8b949e;")
        
        header_layout.addWidget(title)
        header_layout.addWidget(sub_title)
        header_layout.addStretch()
        
        self.dot_lidar = QLabel("● LiDAR")
        self.dot_lidar.setStyleSheet("color: #d29922; font-weight: bold; font-size: 11px;")
        
        self.dot_camera = QLabel("● Camera")
        self.dot_camera.setStyleSheet("color: #d29922; font-weight: bold; font-size: 11px;")
        
        self.dot_imu = QLabel("● IMU")
        self.dot_imu.setStyleSheet("color: #d29922; font-weight: bold; font-size: 11px;")
        
        self.dot_rtk = QLabel("● Hardware Sync: UNKNOWN")
        self.dot_rtk.setStyleSheet("color: #f85149; font-weight: bold; font-size: 11px;")
        
        self.storage_header_lbl = QLabel("● Storage: Checking...")
        self.storage_header_lbl.setStyleSheet("color: #3fb950; font-weight: bold; font-size: 11px;")
        
        header_layout.addWidget(self.dot_lidar)
        header_layout.addWidget(self.dot_camera)
        header_layout.addWidget(self.dot_imu)
        header_layout.addWidget(self.dot_rtk)
        header_layout.addWidget(self.storage_header_lbl)
        
        main_layout.addWidget(header)
        
        # ─────────────────────────────────────────────────────────────────────────────
        # 2. CONTROL BAR (#000000)
        # ─────────────────────────────────────────────────────────────────────────────
        ctrl_bar = QFrame()
        ctrl_bar.setFixedHeight(46)
        ctrl_bar.setStyleSheet("background-color: #000000; border-bottom: 1px solid #30363d;")
        ctrl_layout = QHBoxLayout(ctrl_bar)
        ctrl_layout.setContentsMargins(14, 0, 14, 0)
        ctrl_layout.setSpacing(10)
        
        self.slam_status_lbl = QLabel("● SLAM: STANDBY")
        self.slam_status_lbl.setFont(QFont("Segoe UI", 11, QFont.Weight.Bold))
        self.slam_status_lbl.setStyleSheet("color: #d29922;")
        ctrl_layout.addWidget(self.slam_status_lbl)
        
        sep = QFrame()
        sep.setFrameShape(QFrame.Shape.VLine)
        sep.setStyleSheet("color: #30363d;")
        ctrl_layout.addWidget(sep)
        
        self.start_btn = QPushButton("▶ START")
        self.start_btn.setStyleSheet("""
            QPushButton { background: #3fb950; border: 1px solid #3fb950; color: #ffffff; padding: 5px 16px; border-radius: 4px; font-weight: bold; }
            QPushButton:hover { background: #2ea043; border-color: #2ea043; }
            QPushButton:disabled { color: #484f58; border-color: #21262d; background: #21262d; }
        """)
        self.start_btn.clicked.connect(self.initiate_scan_workflow)
        
        self.anchor_btn = QPushButton("Flash Scan")
        self.anchor_btn.setEnabled(False)
        self.anchor_btn.setStyleSheet("""
            QPushButton { background: #000000; border: 1px solid #30363d; color: #e6edf3; padding: 5px 14px; border-radius: 4px; font-weight: bold; }
            QPushButton:hover { color: #d29922; border-color: #d29922; }
            QPushButton:disabled { color: #484f58; border-color: #21262d; }
        """)
        self.anchor_btn.clicked.connect(self.trigger_faro_flash_anchor)
        
        self.stop_btn = QPushButton("🛑 STOP")
        self.stop_btn.setEnabled(False)
        self.stop_btn.setStyleSheet("""
            QPushButton { background: #000000; border: 1px solid #30363d; color: #e6edf3; padding: 5px 14px; border-radius: 4px; font-weight: bold; }
            QPushButton:hover { color: #f85149; border-color: #f85149; }
            QPushButton:disabled { color: #484f58; border-color: #21262d; }
        """)
        self.stop_btn.clicked.connect(self.stop_mapping_session)

        self.save_btn = QPushButton("💾 SAVE")
        self.save_btn.setEnabled(False)
        self.save_btn.setStyleSheet("""
            QPushButton { background: #000000; border: 1px solid #30363d; color: #e6edf3; padding: 5px 14px; border-radius: 4px; font-weight: bold; }
            QPushButton:hover { color: #58a6ff; border-color: #58a6ff; }
            QPushButton:disabled { color: #484f58; border-color: #21262d; }
        """)
        self.save_btn.clicked.connect(self.save_capture_data)
        
        self.diag_btn = QPushButton("🔴 Diagnostic Mode: OFF")
        self.diag_btn.setStyleSheet("""
            QPushButton { background: #000000; border: 1px solid #30363d; color: #8b949e; padding: 5px 14px; border-radius: 4px; font-weight: bold; }
            QPushButton:hover { color: #f85149; border-color: #f85149; }
        """)
        self.diag_btn.clicked.connect(self.toggle_diagnostic_mode)

        self.rename_btn = QPushButton("Rename Dataset")
        self.rename_btn.setStyleSheet("""
            QPushButton { background: #000000; border: 1px solid #30363d; color: #8b949e; padding: 5px 14px; border-radius: 4px; }
            QPushButton:hover { color: #58a6ff; border-color: #58a6ff; }
        """)
        self.rename_btn.clicked.connect(self.rename_dataset)
        
        self.calib_btn = QPushButton("🛠 Extrinsic Calib")
        self.calib_btn.setStyleSheet("""
            QPushButton { background: #161b22; border: 1px solid #38d9a9; color: #38d9a9; padding: 5px 14px; border-radius: 4px; font-weight: bold; }
            QPushButton:hover { background: #38d9a9; color: #000000; border-color: #38d9a9; }
        """)
        self.calib_btn.clicked.connect(self.open_extrinsic_calib_dialog)

        self.clock_lbl = QLabel("⏱ SCAN TIME: 00:00")
        self.clock_lbl.setFont(QFont("Segoe UI", 11, QFont.Weight.Bold))
        self.clock_lbl.setStyleSheet("color: #3fb950; background-color: #000000; border: 1px solid #30363d; padding: 4px 10px; border-radius: 4px;")

        ctrl_layout.addWidget(self.start_btn)
        ctrl_layout.addWidget(self.calib_btn)
        ctrl_layout.addWidget(self.stop_btn)
        ctrl_layout.addWidget(self.save_btn)
        ctrl_layout.addWidget(self.clock_lbl)
        ctrl_layout.addWidget(self.anchor_btn)
        ctrl_layout.addWidget(self.diag_btn)
        ctrl_layout.addWidget(self.rename_btn)
        ctrl_layout.addStretch()
        
        proj_lbl = QLabel("Project:")
        proj_lbl.setStyleSheet("color: #8b949e; font-size: 11px;")
        self.proj_input = QComboBox()
        self.proj_input.setEditable(True)
        self.proj_input.addItems(["Project_Alpha", "Facility_A", "Infrastructure_B"])
        self.proj_input.setStyleSheet("background-color: #000000; border: 1px solid #30363d; color: #e6edf3; padding: 3px 8px; border-radius: 4px;")
        
        data_lbl = QLabel("Dataset:")
        data_lbl.setStyleSheet("color: #8b949e; font-size: 11px;")
        self.data_input = QLineEdit()
        self.data_input.setText(f"Scan_{datetime.now().strftime('%m%d_%H%M')}")
        self.data_input.setStyleSheet("background-color: #000000; border: 1px solid #30363d; color: #e6edf3; padding: 3px 8px; border-radius: 4px; width: 110px;")
        
        ctrl_layout.addWidget(proj_lbl)
        ctrl_layout.addWidget(self.proj_input)
        ctrl_layout.addWidget(data_lbl)
        ctrl_layout.addWidget(self.data_input)
        
        main_layout.addWidget(ctrl_bar)
        
        # ─────────────────────────────────────────────────────────────────────────────
        # 3. MAIN SPLIT CONTENT AREA (60% Main Clear Center / 40% Peripheral Right HUD)
        # ─────────────────────────────────────────────────────────────────────────────
        splitter = QSplitter(Qt.Orientation.Horizontal)
        splitter.setStyleSheet("QSplitter::handle { background-color: #30363d; }")
        
        # LEFT / CENTER PANEL (60%): COMPLETELY CLEAR FOR AR PASSTHROUGH VISION
        left_container = QWidget()
        left_container.setStyleSheet("background-color: #000000;")
        left_layout = QVBoxLayout(left_container)
        left_layout.setContentsMargins(10, 10, 5, 10)
        left_layout.setSpacing(8)
        
        prog_row = QHBoxLayout()
        self.progress_label = QLabel("STAGE 1: Place Device Static On Flat Ground, Then Press 'START'...")
        self.progress_label.setFont(QFont("Segoe UI", 10, QFont.Weight.Bold))
        self.progress_label.setStyleSheet("color: #d29922;")
        prog_row.addWidget(self.progress_label)
        prog_row.addStretch()
        
        self.main_calib_btn = QPushButton("🛠 EXTRINSIC CALIBRATION")
        self.main_calib_btn.setStyleSheet("""
            QPushButton { background: #38d9a9; border: 1px solid #38d9a9; color: #000000; padding: 4px 12px; border-radius: 4px; font-weight: bold; font-size: 11px; }
            QPushButton:hover { background: #19ffff; border-color: #19ffff; }
        """)
        self.main_calib_btn.clicked.connect(self.open_extrinsic_calib_dialog)
        prog_row.addWidget(self.main_calib_btn)
        
        left_layout.addLayout(prog_row)
        
        self.bar = QProgressBar()
        self.bar.setRange(0, 100)
        self.bar.setValue(0)
        self.bar.setFixedHeight(14)
        self.bar.setStyleSheet("""
            QProgressBar { border: 1px solid #30363d; border-radius: 3px; text-align: center; background-color: #000000; color: white; font-size: 9px; }
            QProgressBar::chunk { background-color: #d29922; border-radius: 2px; }
        """)
        left_layout.addWidget(self.bar)
        
        self.center_viewport = ClearPassthroughViewport()
        left_layout.addWidget(self.center_viewport, 1)
        
        splitter.addWidget(left_container)
        
        # RIGHT PANEL (40% PERIPHERAL SIDE HUD)
        right_container = QWidget()
        right_container.setStyleSheet("background-color: #000000;")
        right_layout = QVBoxLayout(right_container)
        right_layout.setContentsMargins(5, 10, 10, 10)
        right_layout.setSpacing(8)
        
        # 1. Camera Preview Box
        self.camera_canvas = CameraPreviewCanvas()
        self.camera_canvas.setFixedHeight(180)
        right_layout.addWidget(self.camera_canvas)
        
        # 2. NavVis Point Cloud Slice & Path Canvas with IMU Heading Arrow
        map_hdr_row = QHBoxLayout()
        map_title = QLabel("SLAM 2D MAP OVERVIEW")
        map_title.setFont(QFont("Segoe UI", 9, QFont.Weight.Bold))
        map_title.setStyleSheet("color: #38d9a9;")
        map_hdr_row.addWidget(map_title)
        map_hdr_row.addStretch()

        self.map_canvas = NavVisARPointMapCanvas()
        self.map_canvas.setMinimumHeight(280)

        btn_zoom_out = QPushButton("-")
        btn_zoom_out.setToolTip("Zoom Out (Increase View Radius)")
        btn_zoom_out.setFixedSize(22, 22)
        btn_zoom_out.setStyleSheet("background: #161b22; border: 1px solid #30363d; color: #e6edf3; font-weight: bold; border-radius: 3px;")
        btn_zoom_out.clicked.connect(lambda: self.map_canvas.zoom_out())

        btn_zoom_30m = QPushButton("30m")
        btn_zoom_30m.setToolTip("Reset to Standard 30m View Radius")
        btn_zoom_30m.setFixedHeight(22)
        btn_zoom_30m.setStyleSheet("background: #161b22; border: 1px solid #30363d; color: #38d9a9; font-weight: bold; font-size: 10px; border-radius: 3px; padding: 0 5px;")
        btn_zoom_30m.clicked.connect(lambda: self.map_canvas.set_view_radius(30.0))

        btn_zoom_in = QPushButton("+")
        btn_zoom_in.setToolTip("Zoom In (Decrease View Radius)")
        btn_zoom_in.setFixedSize(22, 22)
        btn_zoom_in.setStyleSheet("background: #161b22; border: 1px solid #30363d; color: #e6edf3; font-weight: bold; border-radius: 3px;")
        btn_zoom_in.clicked.connect(lambda: self.map_canvas.zoom_in())

        btn_fit_map = QPushButton("🎯 Fit")
        btn_fit_map.setToolTip("Auto-Fit Viewport to Recorded Map Bounds")
        btn_fit_map.setFixedHeight(22)
        btn_fit_map.setStyleSheet("background: #161b22; border: 1px solid #30363d; color: #19ffff; font-weight: bold; font-size: 10px; border-radius: 3px; padding: 0 5px;")
        btn_fit_map.clicked.connect(lambda: self.map_canvas.fit_map())

        btn_clear_map = QPushButton("🧹 Clear")
        btn_clear_map.setToolTip("Clear Persistent SLAM Map Voxels")
        btn_clear_map.setFixedHeight(22)
        btn_clear_map.setStyleSheet("background: #161b22; border: 1px solid #30363d; color: #f85149; font-weight: bold; font-size: 10px; border-radius: 3px; padding: 0 4px;")
        btn_clear_map.clicked.connect(lambda: self.map_canvas.clear_map())

        map_hdr_row.addWidget(btn_zoom_out)
        map_hdr_row.addWidget(btn_zoom_30m)
        map_hdr_row.addWidget(btn_zoom_in)
        map_hdr_row.addWidget(btn_fit_map)
        map_hdr_row.addWidget(btn_clear_map)

        right_layout.addLayout(map_hdr_row)
        right_layout.addWidget(self.map_canvas, 1)
        
        # 3. Storage & Telemetry Gauge Box
        gauge_box = QFrame()
        gauge_box.setStyleSheet("background-color: #000000; border: 1px solid #30363d; border-radius: 6px; padding: 6px;")
        gauge_layout = QVBoxLayout(gauge_box)
        gauge_layout.setContentsMargins(6, 6, 6, 6)
        gauge_layout.setSpacing(4)
        
        self.storage_title = QLabel("NVMe Storage: Checking...")
        self.storage_title.setFont(QFont("Segoe UI", 9, QFont.Weight.Bold))
        self.storage_title.setStyleSheet("color: #e6edf3;")
        
        self.storage_bar = QProgressBar()
        self.storage_bar.setRange(0, 100)
        self.storage_bar.setValue(0)
        self.storage_bar.setFixedHeight(10)
        self.storage_bar.setStyleSheet("""
            QProgressBar { border: 1px solid #30363d; border-radius: 3px; text-align: center; background-color: #000000; color: white; font-size: 8px; }
            QProgressBar::chunk { background-color: #3fb950; border-radius: 2px; }
        """)
        
        gauge_layout.addWidget(self.storage_title)
        gauge_layout.addWidget(self.storage_bar)
        right_layout.addWidget(gauge_box)
        
        # 4. Telemetry Console
        console_hdr = QLabel("LOG")
        console_hdr.setFont(QFont("Segoe UI", 9, QFont.Weight.Bold))
        console_hdr.setStyleSheet("color: #58a6ff;")
        right_layout.addWidget(console_hdr)
        
        self.console = QTextEdit()
        self.console.setReadOnly(True)
        self.console.setFont(QFont("Monospace", 8))
        self.console.setStyleSheet("background-color: #000000; color: #3fb950; border: 1px solid #30363d; border-radius: 6px; padding: 6px;")
        self.console.append("[scanHUB] Commercial Reality Capture OS Initialized.")
        self.console.append("[SYNC] Hardware-synced temporal offset monitor active.")
        self.console.append("[LIVE DRIVERS] Launching ROS 2 sensor_bringup stack...\n")
        right_layout.addWidget(self.console, 1)
        
        splitter.addWidget(right_container)
        splitter.setSizes([650, 450])
        
        main_layout.addWidget(splitter, 1)
        
        # ─────────────────────────────────────────────────────────────────────────────
        # 4. BOTTOM STATUS BAR
        # ─────────────────────────────────────────────────────────────────────────────
        self.status_bar = QStatusBar()
        self.status_bar.setFont(QFont("Segoe UI", 9))
        self.status_bar.setStyleSheet("background-color: #000000; border-top: 1px solid #30363d; padding: 4px; color: #8b949e;")
        self.status_bar.showMessage("SYSTEM READY  |  Host Epoch Sync (CycloneDDS)  |  ROS Domain: 2")
        main_layout.addWidget(self.status_bar)
        
        self.setLayout(main_layout)

        # Connect ROS 2 worker signals
        self.ros_worker.image_received.connect(self.camera_canvas.update_frame)
        self.ros_worker.registered_pointcloud_received.connect(self.map_canvas.update_registered_points)
        self.ros_worker.pose_received.connect(self.map_canvas.update_pose)
        self.ros_worker.rtk_status_received.connect(self.update_rtk_badge)
        self.ros_worker.sensor_rates_received.connect(self.update_live_sensor_rates_ui)
        self.ros_worker.motion_stability_received.connect(self.update_attitude_motion_gate_ui)
        self.ros_worker.livo_hud_received.connect(self.map_canvas.update_livo_hud)
        self.ros_worker.start()

    def toggle_diagnostic_mode(self):
        self.diagnostic_mode_enabled = not self.diagnostic_mode_enabled
        if self.diagnostic_mode_enabled:
            self.diag_btn.setText("🟢 DIAGNOSTIC MODE: ON")
            self.diag_btn.setStyleSheet("""
                QPushButton { background: #238636; border: 1px solid #3fb950; color: #ffffff; padding: 5px 14px; border-radius: 4px; font-weight: bold; }
                QPushButton:hover { background: #2ea043; }
            """)
            self.console.append("[DIAGNOSTICS] Diagnostic Capture Mode ENABLED.")
            self.console.append("[DIAGNOSTICS] All capture walks will record telemetry CSVs and background rosbag.")
        else:
            self.diag_btn.setText("🔴 Diagnostic Mode: OFF")
            self.diag_btn.setStyleSheet("""
                QPushButton { background: #000000; border: 1px solid #30363d; color: #8b949e; padding: 5px 14px; border-radius: 4px; font-weight: bold; }
                QPushButton:hover { color: #f85149; border-color: #f85149; }
            """)
            self.console.append("[DIAGNOSTICS] Diagnostic Capture Mode DISABLED.")

    def update_button_states(self):
        # 1. STOP button: Enabled immediately whenever recording is active
        self.stop_btn.setEnabled(self.is_recording)

        # 2. SAVE button: Enabled when NOT recording, allowing data to be saved afterwards
        self.save_btn.setEnabled(not self.is_recording)

        # 3. Flash Scan button:
        # Requires ONLY: SLAM pose available AND is_recording == True
        now = time.time()
        slam_available = self.ros_worker.using_slam_odom or (now - self.ros_worker.last_slam_time < 2.5)
        self.anchor_btn.setEnabled(self.is_recording and slam_available)

    def update_live_sensor_rates_ui(self, rates: dict):
        lidar_ok = rates.get("lidar", False)
        camera_ok = rates.get("camera", False)
        imu_ok = rates.get("imu", False)
        slam_ok = rates.get("slam", False)

        self.dot_lidar.setText(f"● LiDAR: {'OK' if lidar_ok else 'NO DATA'}")
        self.dot_lidar.setStyleSheet(f"color: {'#3fb950' if lidar_ok else '#f85149'}; font-weight: bold; font-size: 11px;")

        self.dot_camera.setText(f"● Camera: {'OK' if camera_ok else 'NO DATA'}")
        self.dot_camera.setStyleSheet(f"color: {'#3fb950' if camera_ok else '#f85149'}; font-weight: bold; font-size: 11px;")

        self.dot_imu.setText(f"● IMU: {'OK' if imu_ok else 'NO DATA'}")
        self.dot_imu.setStyleSheet(f"color: {'#3fb950' if imu_ok else '#f85149'}; font-weight: bold; font-size: 11px;")

        self.slam_status_lbl.setText(f"● SLAM: {'RUNNING' if slam_ok else 'STANDBY'}")
        self.slam_status_lbl.setStyleSheet(f"color: {'#3fb950' if slam_ok else '#d29922'}; font-weight: bold; font-size: 11px;")

        # Update button state cleanly without overriding state machine
        self.update_button_states()

    def update_attitude_motion_gate_ui(self, is_stable: bool, stable_duration: float):
        if not self.is_calibrated or self.is_initialized:
            return

        # M4.2 Attitude Lock Gate
        if is_stable:
            if stable_duration >= 3.0:
                self.progress_label.setText("🟢 ATTITUDE LOCKED: Gravity & Gyro Biases Stable. Click START MAPPING.")
                self.progress_label.setStyleSheet("color: #3fb950; font-weight: bold;")
                self.start_btn.setEnabled(True)
                self.start_btn.setText("▶ START MAPPING (ATTITUDE LOCKED)")
                self.start_btn.setStyleSheet("background-color: #3fb950; color: #ffffff; font-weight: bold;")
            else:
                self.progress_label.setText(f"🟡 STABILIZING ATTITUDE: Hold scanner steady... ({stable_duration:.1f}s / 3.0s)")
                self.progress_label.setStyleSheet("color: #d29922; font-weight: bold;")
                self.start_btn.setEnabled(False)
                self.start_btn.setText(f"⏳ HOLD SCANNER STABLE ({3.0 - stable_duration:.1f}s)...")
                self.start_btn.setStyleSheet("background-color: #d29922; color: #000000; font-weight: bold;")
        else:
            self.progress_label.setText("⚠️ MOTION DETECTED! Hold scanner STABLE to lock gravity vector...")
            self.progress_label.setStyleSheet("color: #f85149; font-weight: bold;")
            self.start_btn.setEnabled(False)
            self.start_btn.setText("⚠️ MOTION DETECTED - HOLD STABLE")
            self.start_btn.setStyleSheet("background-color: #f85149; color: #ffffff; font-weight: bold;")

    def update_rtk_badge(self, code: int, label: str):
        self.dot_rtk.setText(f"● Sync: {label}")
        if code == -1: # Unlocked / No Data
            self.dot_rtk.setStyleSheet("color: #f85149; font-weight: bold; font-size: 11px;")
        elif code == 0: # Stable / Low jitter
            self.dot_rtk.setStyleSheet("color: #d29922; font-weight: bold; font-size: 11px;")
        else: # Hardware Locked
            self.dot_rtk.setStyleSheet("color: #3fb950; font-weight: bold; font-size: 11px;")
            
        self.map_canvas.set_rtk_status(code, label)

    def start_live_sensor_stack(self):
        try:
            res = subprocess.run(["pgrep", "-f", "rslidar_sdk_node"], stdout=subprocess.PIPE)
            if res.returncode != 0:
                self.console.append("[INIT] Starting live sensor driver processes (sensor_bringup.launch.py)...")
                self.sensor_bringup_process = QProcess(self)
                cmd = "source /opt/ros/humble/setup.bash && source /home/scanar/scanarMini/install/setup.bash && ros2 launch scan_routine sensor_bringup.launch.py"
                self.sensor_bringup_process.start("bash", ["-c", cmd])
            else:
                self.console.append("[INIT] Sensor driver stack already active. Subscribing to live streams.")
        except Exception as e:
            self.console.append(f"[ERROR] Failed to start sensor drivers: {str(e)}")

    def setup_timers(self):
        self.storage_timer = QTimer()
        self.storage_timer.timeout.connect(self.update_storage_status)
        self.storage_timer.start(2000)
        self.update_storage_status()

    def update_storage_status(self):
        stats = self.storage_monitor.get_storage_stats()
        free_gb = stats["free_gb"]
        total_gb = stats["total_gb"]
        percent_free = stats["percent_free"]
        percent_used = stats["percent_used"]
        is_critical = stats["is_critical"]
        
        self.storage_title.setText(f"NVMe Storage: {free_gb} GB Free / {total_gb} GB Total ({percent_free}% Free)")
        self.storage_bar.setValue(int(percent_used))
        self.storage_header_lbl.setText(f"● Storage: {free_gb} GB ({percent_free}% Free)")
        
        if is_critical:
            self.storage_header_lbl.setStyleSheet("color: #f85149; font-weight: bold; font-size: 11px;")
            if self.is_recording:
                self.console.append(f"[CRITICAL STOP] Storage dropped to {percent_free}%. Halting scan!")
                self.stop_mapping_session()
        else:
            self.storage_header_lbl.setStyleSheet("color: #3fb950; font-weight: bold; font-size: 11px;")

    def update_scan_clock_tick(self):
        if self.is_recording:
            self.scan_elapsed_seconds += 1
            mins = self.scan_elapsed_seconds // 60
            secs = self.scan_elapsed_seconds % 60
            self.clock_lbl.setText(f"⏱ SCAN TIME: {mins:02d}:{secs:02d}")

    def verify_hardware_serials(self):
        """
        Query system active USB devices via sysfs to extract and match Gemini 336L serial.
        Returns (bool, str): (is_matched, detected_serial_or_error_msg)
        """
        import glob
        expected_serial = "CPC6463000XW"
        detected_serials = []
        try:
            # Query all USB serial attributes
            for serial_path in glob.glob("/sys/bus/usb/devices/*/serial"):
                try:
                    with open(serial_path, "r") as f:
                        serial = f.read().strip()
                        if serial:
                            detected_serials.append(serial)
                except Exception:
                    continue
        except Exception as e:
            print(f"[SERIAL CHECK ERROR] Error accessing sysfs: {e}")
            return False, f"Sysfs error: {e}"

        # Match against our active expected serial
        if expected_serial in detected_serials:
            return True, expected_serial
        else:
            # Format list of detected serials for warning dialog
            detected_str = ", ".join(detected_serials) if detected_serials else "None"
            return False, detected_str

    def initiate_scan_workflow(self):
        # USB-Sysfs Hardware Serial startup check & calibration mismatch validation gate
        serial_ok, detected = self.verify_hardware_serials()
        if not serial_ok:
            msg_box = QMessageBox(self)
            try:
                msg_box.setIcon(QMessageBox.Icon.Warning)
                yes_btn = QMessageBox.StandardButton.Yes
                no_btn = QMessageBox.StandardButton.No
            except AttributeError:
                msg_box.setIcon(QMessageBox.Warning)
                yes_btn = QMessageBox.Yes
                no_btn = QMessageBox.No

            msg_box.setWindowTitle("⚠️ CALIBRATION MISMATCH WARNING")
            msg_box.setText(
                "🚨 CALIBRATION MISMATCH, DO NOT START MAPPING! 🚨\n\n"
                f"Expected Camera Serial: CPC6463000XW\n"
                f"Detected USB Serials: {detected}\n\n"
                "WARNING: Active calibration profile parameters do NOT match the connected physical device!\n"
                "Proceeding may result in degraded SLAM estimation, poor data registration, or system failure.\n\n"
                "Do you wish to OVERRIDE and proceed with mapping anyway?"
            )
            msg_box.setStandardButtons(yes_btn | no_btn)
            msg_box.setDefaultButton(no_btn)
            
            res = msg_box.exec_() if hasattr(msg_box, 'exec_') else msg_box.exec()
            if res == no_btn:
                self.console.append("[ABORT] Scan workflow aborted by operator due to calibration mismatch.")
                return
            else:
                self.console.append("[OVERRIDE] Calibration check bypassed by operator override. Proceeding...")

        self.start_btn.hide() # Button disappears immediately on click per user specification
        if not self.is_calibrated:
            self.run_stage_1_bias_calibration()
        else:
            self.start_rotation_ritual()

    def run_stage_1_bias_calibration(self):
        self.console.append("[INIT] NavVis Ritual Stage 1: Static ZUPT Bias Calibration (Done once per power-on)...")
        self.calib_ticks = 0
        self.bar.setValue(0)
        
        self.calib_timer = QTimer()
        self.calib_timer.timeout.connect(self.sample_bias_ticks)
        self.calib_timer.start(100)

    def sample_bias_ticks(self):
        self.calib_ticks += 1
        progress = int((self.calib_ticks / 100) * 100)
        self.bar.setValue(progress)
        
        if self.calib_ticks == 1:
            self.calib_ax_sum = 0.0
            self.calib_ay_sum = 0.0
            self.calib_az_sum = 0.0

        self.calib_ax_sum += self.ros_worker.last_ax
        self.calib_ay_sum += self.ros_worker.last_ay
        self.calib_az_sum += self.ros_worker.last_az
        
        if self.calib_ticks >= 100:
            self.calib_timer.stop()
            self.is_calibrated = True
            
            # Compute average gravity vector
            ax_avg = self.calib_ax_sum / 100.0
            ay_avg = self.calib_ay_sum / 100.0
            az_avg = self.calib_az_sum / 100.0
            
            # Calculate Roll and Pitch gravity alignment angles relative to gravity
            roll_est = math.atan2(ay_avg, az_avg)
            pitch_est = math.atan2(-ax_avg, math.hypot(ay_avg, az_avg))
            
            roll_est_deg = math.degrees(roll_est)
            pitch_est_deg = math.degrees(pitch_est)
            
            self.console.append(f"[SUCCESS] Stage 1 Complete: Gravity aligned! Average accel: X={ax_avg:+.2f} Y={ay_avg:+.2f} Z={az_avg:+.2f} m/s²")
            self.console.append(f"[CALIBRATION] Calculated Attitude from Ground Baseline: Roll={roll_est_deg:+.1f}° Pitch={pitch_est_deg:+.1f}°")
            
            # Save the calculated gravity offsets to the map canvas for view alignment/leveling.
            # This keeps the physical sensor extrinsics fixed and immutable as calibrated!
            self.map_canvas.gravity_roll_deg = roll_est_deg
            self.map_canvas.gravity_pitch_deg = pitch_est_deg
            
            self.start_rotation_ritual()

    def start_rotation_ritual(self):
        self.ros_worker.accumulated_rotation_deg = 0.0
        self.ros_worker.last_imu_gyro_time = time.time()
        self.ros_worker.in_rotation_ritual = True
        
        self.bar.setValue(0)
        self.console.append("[INIT] NavVis Ritual Stage 2: Pivot scanner 90° to synchronize SLAM attitude...")
        self.progress_label.setText("🔄 NAVVIS RITUAL: Slowly pivot scanner 90° to synchronize SLAM attitude...")
        self.progress_label.setStyleSheet("color: #ffb700; font-weight: bold;")

    def update_rotation_ritual_ui(self, deg: float):
        progress = min(100, int((deg / 90.0) * 100))
        self.bar.setValue(progress)
        self.progress_label.setText(f"🔄 NAVVIS RITUAL: Pivot scanner 90°... ({deg:.0f}° / 90°)")
        self.progress_label.setStyleSheet("color: #ffb700; font-weight: bold;")

    def on_rotation_ritual_completed(self):
        self.bar.setValue(100)
        self.console.append("[SUCCESS] NavVis 90° Pivot Ritual Automatically Detected! SLAM attitude synchronized.")
        self.progress_label.setText("🟢 90° PIVOT LOCKED! Reality capture recording active...")
        self.progress_label.setStyleSheet("color: #3fb950; font-weight: bold;")
        self.execute_active_capture_pipelines()

    def execute_active_capture_pipelines(self):
        project = self.proj_input.currentText().strip().replace(" ", "_")
        dataset = self.data_input.text().strip().replace(" ", "_")
        self.console.append(f"[CAPTURE] Session active: {dataset}")
        
        # Reset and start live scan clock
        self.scan_elapsed_seconds = 0
        self.clock_lbl.setText("⏱ SCAN TIME: 00:00")
        self.scan_clock_timer.start(1000)

        # Launch FAST-LIVO2 SLAM Engine
        try:
            res = subprocess.run(["pgrep", "-f", "fastlivo_mapping"], stdout=subprocess.PIPE)
            if res.returncode != 0:
                self.console.append("[FAST-LIVO2] Launching FAST-LIVO2 Tightly-Coupled LIVO SLAM Engine...")
                self.mapping_process = QProcess(self)
                cmd = "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && source /opt/ros/humble/setup.bash && source /home/scanar/scanarMini/install/setup.bash && ros2 launch fast_livo mapping_robosense.launch.py"
                self.mapping_process.start("bash", ["-c", cmd])
            else:
                self.console.append("[FAST-LIVO2] SLAM engine already active. Subscribing to live odometry.")
        except Exception as e:
            self.console.append(f"[ERROR] Failed to start FAST-LIVO2 SLAM engine: {str(e)}")

        self.is_recording = True
        self.update_button_states()

        if self.diagnostic_mode_enabled:
            ok, diag_path = self.diag_recorder.start_session(label=dataset, record_rosbag=True)
            if ok:
                self.console.append(f"[DIAGNOSTICS] Diagnostic Mode ACTIVE. Saving telemetry & rosbag to:\n  {diag_path}")

        self.map_canvas.reset_map()
        self.map_canvas.is_active = True
        
        self.slam_status_lbl.setText("● SLAM: RUNNING")
        self.slam_status_lbl.setStyleSheet("color: #3fb950;")

    def trigger_faro_flash_anchor(self):
        label = f"w{self.anchor_count:02d}"
        self.anchor_count += 1
        timestamp = datetime.now().strftime("%H:%M:%S")
        
        cx = self.map_canvas.current_x
        cy = self.map_canvas.current_y
        
        self.map_canvas.add_flash_anchor(label, cx, cy)
        self.console.append(f"[FARO FLASH SCAN] Anchor {label} Captured at X={cx:+.2f}m Y={cy:+.2f}m ({timestamp})")

    def stop_mapping_session(self):
        self.console.append("[INFO] Finalizing capture session...")
        if self.mapping_process and self.mapping_process.state() == QProcess.ProcessState.Running:
            self.console.append("[FAST-LIVO2] Stopping FAST-LIVO2 SLAM engine...")
            self.mapping_process.terminate()
            self.mapping_process.waitForFinished(1000)
            self.mapping_process = None

        self.is_recording = False
        self.scan_clock_timer.stop()
        self.clock_lbl.setText("⏱ SCAN TIME: 00:00")
        self.update_button_states()
        
        # Start button re-appears on scan stop for subsequent scans
        self.start_btn.show()
        self.start_btn.setEnabled(True)
        self.start_btn.setText("▶ START")
        self.start_btn.setStyleSheet("background-color: #3fb950; border: 1px solid #3fb950; color: #ffffff; padding: 5px 16px; border-radius: 4px; font-weight: bold;")
        try:
            self.start_btn.clicked.disconnect()
        except Exception:
            pass
        self.start_btn.clicked.connect(self.initiate_scan_workflow)
        
        self.map_canvas.is_active = False
        
        if self.diag_recorder.is_recording:
            report_path = self.diag_recorder.stop_session()
            self.console.append(f"[DIAGNOSTICS] Diagnostic session finalized.\n  Report: {report_path}")

        self.slam_status_lbl.setText("● SLAM: STANDBY")
        self.slam_status_lbl.setStyleSheet("color: #d29922;")
        self.progress_label.setText("SESSION STOPPED: Click 'START' to begin another capture.")
        self.progress_label.setStyleSheet("color: #8b949e; font-weight: bold;")

    def save_capture_data(self):
        dataset = self.data_input.text().strip().replace(" ", "_")
        if not dataset:
            dataset = datetime.now().strftime("Scan_%m%d_%H%M")
        
        save_dir = f"/home/scanar/scanarMini/data/{dataset}"
        os.makedirs(save_dir, exist_ok=True)
        
        self.console.append(f"\n[SAVE] Exporting and saving captured point cloud data for session: {dataset}...")
        
        # 1. Export Trajectory CSV and TUM format
        traj_csv = os.path.join(save_dir, "trajectory.csv")
        traj_tum = os.path.join(save_dir, "trajectory_tum.txt")
        try:
            with open(traj_csv, "w") as f_csv, open(traj_tum, "w") as f_tum:
                f_csv.write("index,x_m,y_m,yaw_deg\n")
                for idx, (tx, ty, tyaw) in enumerate(self.map_canvas.trajectory_path):
                    f_csv.write(f"{idx},{tx:.4f},{ty:.4f},{tyaw:.2f}\n")
                    f_tum.write(f"{idx * 0.1:.4f} {tx:.4f} {ty:.4f} 0.0000 0.0 0.0 {math.sin(math.radians(tyaw)/2.0):.4f} {math.cos(math.radians(tyaw)/2.0):.4f}\n")
            self.console.append(f"[SAVE] Exported SLAM trajectory: {traj_csv}")
        except Exception as e:
            self.console.append(f"[SAVE ERROR] Trajectory export failed: {e}")

        # 2. Export FARO Flash Anchors JSON
        anchors_json = os.path.join(save_dir, "flash_anchors.json")
        try:
            with open(anchors_json, "w") as f_anchors:
                json.dump(self.map_canvas.flash_anchors, f_anchors, indent=2)
            self.console.append(f"[SAVE] Exported {len(self.map_canvas.flash_anchors)} Flash Anchors: {anchors_json}")
        except Exception as e:
            self.console.append(f"[SAVE ERROR] Anchors export failed: {e}")

        # 3. Export Persistent Point Cloud Map (.ply)
        ply_file = os.path.join(save_dir, "pointcloud_map.ply")
        try:
            pts = list(self.map_canvas.voxel_map) if self.map_canvas.voxel_map else self.map_canvas.accumulated_points
            with open(ply_file, "w") as f_ply:
                f_ply.write("ply\nformat ascii 1.0\n")
                f_ply.write(f"element vertex {len(pts)}\n")
                f_ply.write("property float x\nproperty float y\nproperty float z\n")
                f_ply.write("end_header\n")
                for wx, wy in pts:
                    f_ply.write(f"{wx:.3f} {wy:.3f} 0.000\n")
            self.console.append(f"[SAVE] Exported point cloud map: {ply_file} ({len(pts)} points)")
        except Exception as e:
            self.console.append(f"[SAVE ERROR] PLY export failed: {e}")

        self.console.append(f"[SAVE] Dataset saved successfully! Path: {save_dir}/")
        self.progress_label.setText(f"🎉 DATASET SAVED: /home/scanar/scanarMini/data/{dataset}/")
        self.progress_label.setStyleSheet("color: #38d9a9; font-weight: bold;")

    def rename_dataset(self):
        old_dataset = self.data_input.text().strip().replace(" ", "_")
        new_dataset, ok = QInputDialog.getText(self, "Rename Capture Session", f"Current Label: {old_dataset}\nEnter new label:")
        if ok and new_dataset.strip():
            self.data_input.setText(new_dataset.strip().replace(" ", "_"))

    def open_extrinsic_calib_dialog(self):
        # We do NOT reset the map so the operator can align the live scan sweep with the accumulated room map!
        self.map_canvas.pause_map_accumulation = True
        dialog = ExtrinsicCalibrationDialog(self.ros_worker, self)
        if hasattr(dialog, 'exec_'):
            dialog.exec_()
        else:
            dialog.exec()
        self.map_canvas.pause_map_accumulation = False

    def closeEvent(self, event):
        self.ros_worker.stop()
        if hasattr(self, 'sensor_bringup_process') and self.sensor_bringup_process and self.sensor_bringup_process.state() == QProcess.ProcessState.Running:
            self.sensor_bringup_process.terminate()
            self.sensor_bringup_process.waitForFinished(1000)
        if hasattr(self, 'mapping_process') and self.mapping_process and self.mapping_process.state() == QProcess.ProcessState.Running:
            self.mapping_process.terminate()
            self.mapping_process.waitForFinished(1000)
        event.accept()


def main():
    app = QApplication(sys.argv)
    gui = ScanHUBOS()
    gui.show()
    sys.exit(app.exec_() if hasattr(app, 'exec_') else app.exec())


if __name__ == "__main__":
    main()
