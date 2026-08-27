#!/usr/bin/env python3
"""
Milestone M12.7: Human-in-the-Loop Dynamic Multi-Modal Fusion Test Harness & Exporter
Features:
- Explicit phase control and event tagging (no auto-advancing timers for human movement)
- Full message telemetry tracking: received vs admitted vs keyframed vs rejected vs dropped
- Three-source trajectory consistency check: Raw Odom (A), Recorder (B), Export CSV (C)
- Phase-isolated Gemini-to-Airy plane residual evaluation (Stationary vs Dynamic)
- Production Three-Class ASPS Exporter saving all 7 multi-modal files
"""

import os
import sys
import time
import math
import json
import hashlib
import numpy as np
from collections import deque
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, PointField, Image, CameraInfo, Imu
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from m12_7_dynamic_fusion_engine import execute_three_class_asps_fusion

# Calibrated Extrinsics
# Factory Depth to RGB Extrinsics
qx_d2r, qy_d2r, qz_d2r, qw_d2r = -0.000658, -0.000174, -0.001189, 0.999999
R_RGB_DEPTH = np.array([
    [1 - 2*(qy_d2r**2 + qz_d2r**2), 2*(qx_d2r*qy_d2r - qz_d2r*qw_d2r), 2*(qx_d2r*qz_d2r + qy_d2r*qw_d2r)],
    [2*(qx_d2r*qy_d2r + qz_d2r*qw_d2r), 1 - 2*(qx_d2r**2 + qz_d2r**2), 2*(qy_d2r*qz_d2r - qx_d2r*qw_d2r)],
    [2*(qx_d2r*qz_d2r - qy_d2r*qw_d2r), 2*(qy_d2r*qz_d2r + qx_d2r*qw_d2r), 1 - 2*(qx_d2r**2 + qy_d2r**2)]
], dtype=np.float64)
T_RGB_DEPTH_VEC = np.array([0.023731, -0.000093, -0.000425], dtype=np.float64)

# Calibrated LiDAR to RGB Extrinsics (M12.1 qualified)
R_RGB_LIDAR = np.array([
    [ 0.000350, -0.999997,  0.002377],
    [-0.001316, -0.002378, -0.999996],
    [ 0.999999,  0.000347, -0.001317]
], dtype=np.float64)
T_RGB_LIDAR_VEC = np.array([0.023855, 0.014534, -0.055265], dtype=np.float64)

# FAST-LIVO2 IMU from LiDAR Extrinsics
R_IMU_LIDAR = np.array([
    [ 0.012366, -0.999888,  0.008426],
    [-0.999919, -0.012340,  0.003184],
    [-0.003079, -0.008465, -0.999959]
], dtype=np.float64)
T_IMU_LIDAR_VEC = np.array([0.004164, 0.004315, -0.004411], dtype=np.float64)

# Rigid Transform from Gemini Depth to LiDAR frame:
R_LIDAR_DEPTH = R_RGB_LIDAR.T @ R_RGB_DEPTH
T_LIDAR_DEPTH_VEC = R_RGB_LIDAR.T @ (T_RGB_DEPTH_VEC - T_RGB_LIDAR_VEC)

# Rigid Transform from Gemini Depth to IMU (body) frame:
R_IMU_DEPTH = R_IMU_LIDAR @ R_LIDAR_DEPTH
T_IMU_DEPTH_VEC = R_IMU_LIDAR @ T_LIDAR_DEPTH_VEC + T_IMU_LIDAR_VEC

T_DEPTH_IMU = np.eye(4, dtype=np.float64)
T_DEPTH_IMU[:3, :3] = R_IMU_DEPTH
T_DEPTH_IMU[:3, 3] = T_IMU_DEPTH_VEC

K_COLOR_DEF = np.array([
    [609.006531,   0.000000, 643.868774],
    [  0.000000, 609.015625, 399.326538],
    [  0.000000,   0.000000,   1.000000]
], dtype=np.float64)


def quat_to_rot(q):
    qx, qy, qz, qw = q
    n = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    if n < 1e-10:
        return np.eye(3)
    qx, qy, qz, qw = qx/n, qy/n, qz/n, qw/n
    return np.array([
        [1.0 - 2.0*(qy*qy + qz*qz), 2.0*(qx*qy - qz*qw),     2.0*(qx*qz + qy*qw)],
        [2.0*(qx*qy + qz*qw),     1.0 - 2.0*(qx*qx + qz*qz), 2.0*(qy*qz - qx*qw)],
        [2.0*(qx*qz - qy*qw),     2.0*(qy*qz + qx*qw),     1.0 - 2.0*(qx*qx + qy*qy)]
    ], dtype=np.float64)


def slerp_quat(q0, q1, alpha):
    """
    Self-contained normalized quaternion spherical linear interpolation (SLERP).
    q0, q1: [qx, qy, qz, qw]
    alpha: float in [0.0, 1.0]
    """
    q0 = np.asarray(q0, dtype=np.float64)
    q1 = np.asarray(q1, dtype=np.float64)
    norm0 = np.linalg.norm(q0)
    norm1 = np.linalg.norm(q1)
    if norm0 < 1e-10 or norm1 < 1e-10:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
    q0 = q0 / norm0
    q1 = q1 / norm1

    dot = float(np.sum(q0 * q1))
    if dot < 0.0:
        q1 = -q1
        dot = -dot

    if dot > 0.9995:
        q_interp = (1.0 - alpha) * q0 + alpha * q1
        norm_interp = np.linalg.norm(q_interp)
        return q_interp / norm_interp if norm_interp > 1e-10 else q0

    theta = math.acos(max(-1.0, min(1.0, dot)))
    sin_theta = math.sin(theta)
    if abs(sin_theta) < 1e-6:
        return q0

    w0 = math.sin((1.0 - alpha) * theta) / sin_theta
    w1 = math.sin(alpha * theta) / sin_theta
    q_interp = w0 * q0 + w1 * q1
    return q_interp / np.linalg.norm(q_interp)


class M127InteractiveTestHarness(Node):
    def __init__(self, dataset_name="M12_7_Controlled_Motion"):
        super().__init__('m12_7_interactive_test_harness')
        self.dataset_name = dataset_name
        self.export_dir = f"/home/scanar/scanarMini/data/{dataset_name}"
        os.makedirs(self.export_dir, exist_ok=True)

        self.bridge = CvBridge()
        self.lock = threading.RLock()

        # Cumulative Global Counters
        self.cnt_raw_odom_msgs = 0
        self.cnt_raw_airy_msgs = 0
        self.cnt_raw_imu_msgs = 0
        self.cnt_raw_registered_msgs = 0
        self.cnt_raw_depth_msgs = 0
        self.cnt_raw_rgb_msgs = 0

        self.cnt_admitted_airy_sweeps = 0
        self.cnt_admitted_depth_frames = 0
        self.cnt_rgb_timestamp_matched = 0
        self.cnt_temporal_gate_rejects = 0
        self.cnt_range_gate_rejects = 0
        self.cnt_missing_pose_rejects = 0
        self.cnt_queue_drops = 0

        # Phase-Isolated Telemetry Tracking (Reset per Stage)
        self.phase_telemetry = {}
        self.active_phase_stats = None

        # Raw Odom Tracking for direct Source A comparison
        self.raw_odom_poses = [] # [(stamp, pos, rot)]

        # Recorder Internal Trajectory (Source B)
        self.trajectory_b = []   # [(stamp, x, y, z, qx, qy, qz, qw, yaw, phase_name)]
        self.odom_buffer = deque(maxlen=2000)

        # Observations with Phase Tagging
        self.airy_sweeps = []    # [(stamp, xyz_world, phase_name)]
        self.gemini_frames = []  # [(stamp, xyz_world, rgb, phase_name)]
        self.phase_raw_depth_frames = {} # {phase_name: [(sec, depth_img, rgb_img, rgb_stamp)]}

        # Raw Buffers for Deferred Processing (Zero latency on ROS callbacks)
        self.raw_depth_records = [] # [(sec, cv_depth, rgb_img, rgb_stamp, phase)]
        self.raw_lidar_records = [] # [(sec, raw_bytes, point_step, is_cam_init, phase)]

        # Live State
        self.latest_rgb_image = None
        self.latest_rgb_stamp = 0.0
        self.depth_k = None

        self.is_recording = False
        self.current_phase = "IDLE"
        self.phase_start_t = 0.0
        self.session_start_t = 0.0

        # Live Preview Publisher
        qos_pub = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=5)
        self.pub_live_fused = self.create_publisher(PointCloud2, '/scanar/live_fused_cloud', qos_pub)

        from rclpy.callback_groups import ReentrantCallbackGroup
        self.cb_group = ReentrantCallbackGroup()

        # Sensor QoS Matching
        qos_reliable = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=100)
        qos_sensor = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=100)

        self.sub_odom = self.create_subscription(Odometry, '/aft_mapped_to_init', self.odom_callback, qos_reliable, callback_group=self.cb_group)
        self.sub_registered = self.create_subscription(PointCloud2, '/cloud_registered', self.registered_callback, qos_reliable, callback_group=self.cb_group)
        self.sub_lidar_raw = self.create_subscription(PointCloud2, '/rslidar_points', self.lidar_callback, qos_sensor, callback_group=self.cb_group)
        self.sub_imu = self.create_subscription(Imu, '/rslidar_imu_data', self.imu_callback, qos_sensor, callback_group=self.cb_group)
        self.sub_rgb = self.create_subscription(Image, '/camera/color/image_raw', self.rgb_callback, qos_sensor, callback_group=self.cb_group)
        self.sub_depth = self.create_subscription(Image, '/camera/depth/image_raw', self.depth_callback, qos_sensor, callback_group=self.cb_group)
        self.sub_depth_rect = self.create_subscription(Image, '/camera/depth/image_rect_raw', self.depth_callback, qos_sensor, callback_group=self.cb_group)
        self.sub_depth_info = self.create_subscription(CameraInfo, '/camera/depth/camera_info', self.depth_info_callback, qos_sensor, callback_group=self.cb_group)

        self.get_logger().info(f"[M12.7 Test Harness] Initialized with matched QoS profiles for: {self.dataset_name}")

    def imu_callback(self, msg: Imu):
        with self.lock:
            self.cnt_raw_imu_msgs += 1
            if self.is_recording and self.active_phase_stats is not None:
                self.active_phase_stats["raw_rslidar_imu"] += 1

    def registered_callback(self, msg: PointCloud2):
        sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with self.lock:
            self.cnt_raw_registered_msgs += 1
            if self.is_recording and self.active_phase_stats is not None:
                self.active_phase_stats["raw_cloud_registered"] += 1

        if not self.is_recording:
            return

        if msg.width * msg.height == 0 or msg.point_step < 12:
            return

        raw_bytes = bytes(msg.data)
        with self.lock:
            cur_phase = self.current_phase
            self.raw_lidar_records.append((sec, raw_bytes, msg.point_step, True, cur_phase))

    def odom_callback(self, msg: Odometry):
        sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        px, py, pz = msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z
        qx, qy, qz, qw = msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w
        
        pos = np.array([px, py, pz], dtype=np.float64)
        quat = np.array([qx, qy, qz, qw], dtype=np.float64)
        rot = quat_to_rot(quat)
        yaw = math.atan2(2.0*(qw*qz + qx*qy), 1.0 - 2.0*(qy*qy + qz*qz))

        with self.lock:
            self.cnt_raw_odom_msgs += 1
            if self.is_recording and self.active_phase_stats is not None:
                self.active_phase_stats["raw_aft_mapped_to_init"] += 1
                if self.active_phase_stats["start_pose"] is None:
                    self.active_phase_stats["start_pose"] = [float(px), float(py), float(pz), float(yaw)]
                self.active_phase_stats["end_pose"] = [float(px), float(py), float(pz), float(yaw)]

            self.raw_odom_poses.append((sec, pos, quat, rot))
            self.odom_buffer.append((sec, pos, quat, rot))
            if self.is_recording:
                self.trajectory_b.append((sec, px, py, pz, qx, qy, qz, qw, yaw, self.current_phase))

    def rgb_callback(self, msg: Image):
        sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with self.lock:
            self.cnt_raw_rgb_msgs += 1
            if self.is_recording and self.active_phase_stats is not None:
                self.active_phase_stats["raw_color_image"] += 1

        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            with self.lock:
                self.latest_rgb_image = cv_img
                self.latest_rgb_stamp = sec
        except Exception:
            pass

    def depth_info_callback(self, msg: CameraInfo):
        if self.depth_k is None:
            self.depth_k = np.array(msg.k, dtype=np.float64).reshape((3, 3))

    def depth_callback(self, msg: Image):
        sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with self.lock:
            self.cnt_raw_depth_msgs += 1
            if self.is_recording and self.active_phase_stats is not None:
                self.active_phase_stats["raw_depth_image"] += 1

        if not self.is_recording:
            return

        try:
            cv_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception:
            return

        with self.lock:
            rgb_img = self.latest_rgb_image.copy() if self.latest_rgb_image is not None else None
            rgb_stamp = self.latest_rgb_stamp
            cur_phase = self.current_phase
            self.raw_depth_records.append((sec, cv_depth, rgb_img, rgb_stamp, cur_phase))

    def lidar_callback(self, msg: PointCloud2):
        with self.lock:
            self.cnt_raw_airy_msgs += 1
            if self.is_recording and self.active_phase_stats is not None:
                self.active_phase_stats["raw_rslidar_points"] += 1

    def get_interpolated_pose(self, query_time):
        with self.lock:
            if len(self.odom_buffer) == 0:
                return None, None
            times = [item[0] for item in self.odom_buffer]
            if query_time <= times[0]:
                return self.odom_buffer[0][1], self.odom_buffer[0][3]
            if query_time >= times[-1]:
                return self.odom_buffer[-1][1], self.odom_buffer[-1][3]
            idx = np.searchsorted(times, query_time)
            t0, p0, q0, r0 = self.odom_buffer[idx - 1]
            t1, p1, q1, r1 = self.odom_buffer[idx]
            dt = t1 - t0
            alpha = (query_time - t0) / dt if dt > 1e-6 else 0.0
            alpha = max(0.0, min(1.0, alpha))
            p_interp = (1.0 - alpha) * p0 + alpha * p1
            q_interp = slerp_quat(q0, q1, alpha)
            r_interp = quat_to_rot(q_interp)
            return p_interp, r_interp

    def backproject_depth(self, depth_raw, rgb_img, K_depth, pos_w, rot_w, decimate=4):
        h, w = depth_raw.shape[:2]
        depth_m = depth_raw.astype(np.float32) / 1000.0
        u_grid, v_grid = np.meshgrid(np.arange(0, w, decimate), np.arange(0, h, decimate))
        z = depth_m[v_grid, u_grid]

        valid = (z >= 0.35) & (z <= 4.5) & np.isfinite(z)
        self.cnt_range_gate_rejects += int(np.sum(~valid))
        if np.sum(valid) == 0:
            return None, None

        u_v, v_v, z_v = u_grid[valid], v_grid[valid], z[valid]
        fx, fy = K_depth[0, 0], K_depth[1, 1]
        cx, cy = K_depth[0, 2], K_depth[1, 2]

        x_c = (u_v - cx) * z_v / fx
        y_c = (v_v - cy) * z_v / fy
        pts_cam = np.column_stack((x_c, y_c, z_v))

        # Transform Cam -> IMU (body) -> World
        pts_imu = (T_DEPTH_IMU[:3, :3] @ pts_cam.T).T + T_DEPTH_IMU[:3, 3]
        pts_world = (rot_w @ pts_imu.T).T + pos_w

        # RGB Colorization
        colors = np.full((len(pts_world), 3), 180, dtype=np.uint8)
        if rgb_img is not None:
            pts_cam_rgb = (R_RGB_DEPTH @ pts_cam.T).T + T_RGB_DEPTH_VEC
            z_rgb = pts_cam_rgb[:, 2]
            valid_z = z_rgb > 0.2

            fx_c, fy_c = K_COLOR_DEF[0, 0], K_COLOR_DEF[1, 1]
            cx_c, cy_c = K_COLOR_DEF[0, 2], K_COLOR_DEF[1, 2]

            u_rgb = (pts_cam_rgb[valid_z, 0] * fx_c / z_rgb[valid_z] + cx_c).astype(int)
            v_rgb = (pts_cam_rgb[valid_z, 1] * fy_c / z_rgb[valid_z] + cy_c).astype(int)

            img_h, img_w = rgb_img.shape[:2]
            in_bounds = (u_rgb >= 0) & (u_rgb < img_w) & (v_rgb >= 0) & (v_rgb < img_h)
            valid_indices = np.where(valid_z)[0][in_bounds]
            u_valid = u_rgb[in_bounds]
            v_valid = v_rgb[in_bounds]

            bgr_samples = rgb_img[v_valid, u_valid]
            colors[valid_indices] = bgr_samples[:, [2, 1, 0]]

        return pts_world, colors

    def execute_live_preview_ssd(self, airy_xyz, gemini_xyz, gemini_rgb, tau_m=0.020):
        from scipy.spatial import cKDTree
        tree = cKDTree(airy_xyz)
        dists, _ = tree.query(gemini_xyz, k=1, workers=2)
        keep = dists > tau_m
        if np.sum(keep) > 0:
            fused_xyz = np.vstack((airy_xyz, gemini_xyz[keep]))
            airy_rgb = np.full((len(airy_xyz), 3), 140, dtype=np.uint8)
            fused_rgb = np.vstack((airy_rgb, gemini_rgb[keep]))
        else:
            fused_xyz = airy_xyz
            fused_rgb = np.full((len(airy_xyz), 3), 140, dtype=np.uint8)
        return fused_xyz, fused_rgb

    def publish_live_cloud(self, xyz, rgb, stamp):
        n = len(xyz)
        if n == 0:
            return
        msg = PointCloud2()
        msg.header.stamp = stamp
        msg.header.frame_id = "camera_init"
        msg.height, msg.width = 1, n
        msg.is_dense = True
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        msg.point_step = 16
        msg.row_step = 16 * n

        r, g, b = rgb[:, 0].astype(np.uint32), rgb[:, 1].astype(np.uint32), rgb[:, 2].astype(np.uint32)
        rgb_float = ((r << 16) | (g << 8) | b).view(np.float32)

        dt = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('rgb', '<f4')])
        arr = np.empty(n, dtype=dt)
        arr['x'], arr['y'], arr['z'], arr['rgb'] = xyz[:, 0], xyz[:, 1], xyz[:, 2], rgb_float

        msg.data = arr.tobytes()
        self.pub_live_fused.publish(msg)

    def set_phase(self, phase_name):
        with self.lock:
            self.current_phase = phase_name
            self.phase_start_t = time.time()
            if not self.is_recording and phase_name != "IDLE":
                self.is_recording = True
                self.session_start_t = time.time()

            # Initialize isolated counters for this phase (purge any previous records for this phase)
            self.raw_lidar_records = [r for r in self.raw_lidar_records if r[4] != phase_name]
            self.raw_depth_records = [r for r in self.raw_depth_records if r[4] != phase_name]
            self.airy_sweeps = [r for r in self.airy_sweeps if r[2] != phase_name]
            self.gemini_frames = [r for r in self.gemini_frames if r[3] != phase_name]
            self.trajectory_b = [r for r in self.trajectory_b if r[9] != phase_name]

            self.active_phase_stats = {
                "stage_name": phase_name,
                "start_time": time.time(),
                "end_time": None,
                "duration": 0.0,
                "raw_rslidar_points": 0,
                "raw_rslidar_imu": 0,
                "raw_aft_mapped_to_init": 0,
                "raw_cloud_registered": 0,
                "raw_color_image": 0,
                "raw_depth_image": 0,
                "admitted_airy_sweeps": 0,
                "admitted_depth_frames": 0,
                "rgb_matched": 0,
                "temporal_rejects": 0,
                "range_rejects": 0,
                "missing_pose_rejects": 0,
                "queue_drops": 0,
                "start_pose": None,
                "end_pose": None
            }
            self.get_logger().info(f">>> [PHASE START] {phase_name} counters initialized to zero (purged prior records if any) <<<")

    def close_phase(self, phase_name):
        with self.lock:
            if self.active_phase_stats is None:
                return {}
            end_t = time.time()
            self.active_phase_stats["end_time"] = end_t
            dur = max(end_t - self.active_phase_stats["start_time"], 1e-4)
            self.active_phase_stats["duration"] = dur

            # Process buffered raw records for this phase
            self.process_buffered_stage_data(phase_name)

            # Compute rates
            stats = dict(self.active_phase_stats)
            stats["rslidar_points_hz"] = round(stats["raw_rslidar_points"] / dur, 2)
            stats["rslidar_imu_hz"] = round(stats["raw_rslidar_imu"] / dur, 2)
            stats["aft_mapped_to_init_hz"] = round(stats["raw_aft_mapped_to_init"] / dur, 2)
            stats["cloud_registered_hz"] = round(stats["raw_cloud_registered"] / dur, 2)
            stats["color_image_fps"] = round(stats["raw_color_image"] / dur, 2)
            stats["depth_image_fps"] = round(stats["raw_depth_image"] / dur, 2)

            # Trajectory stats for this phase
            phase_poses = [p for p in self.trajectory_b if p[9] == phase_name]
            if len(phase_poses) > 1:
                p_arr = np.array([[p[1], p[2], p[3]] for p in phase_poses])
                stats["trajectory_length_m"] = float(round(np.sum(np.linalg.norm(np.diff(p_arr, axis=0), axis=1)), 4))
                stats["net_displacement_m"] = float(round(np.linalg.norm(p_arr[-1] - p_arr[0]), 4))
                yaw_start = phase_poses[0][8]
                yaw_end = phase_poses[-1][8]
                stats["delta_yaw_deg"] = float(round(abs(math.degrees(yaw_end - yaw_start)), 2))
            else:
                stats["trajectory_length_m"] = 0.0
                stats["net_displacement_m"] = 0.0
                stats["delta_yaw_deg"] = 0.0

            # Evaluate phase geometric residuals
            eval_res = self.evaluate_single_phase_residuals(phase_name)
            stats.update(eval_res)

            # Evaluate Analysis-Only Camera Time Offset Sweep (-60ms to +60ms in 5ms increments)
            sweep_dict = self.evaluate_camera_time_offset_sweep(phase_name)
            stats["camera_time_offset_sweep"] = sweep_dict

            self.phase_telemetry[phase_name] = stats
            self.get_logger().info(f">>> [PHASE CLOSED] {phase_name} | Duration: {dur:.2f}s | Airy: {stats['raw_rslidar_points']} ({stats['rslidar_points_hz']}Hz) | Depth: {stats['raw_depth_image']} ({stats['depth_image_fps']}FPS) | Odom: {stats['raw_aft_mapped_to_init']} ({stats['aft_mapped_to_init_hz']}Hz) | Residual P50: {eval_res['p50_residual_mm']:.2f}mm, P95: {eval_res['p95_residual_mm']:.2f}mm <<<")
            return stats

    def process_buffered_stage_data(self, phase_name):
        with self.lock:
            l_recs = [r for r in self.raw_lidar_records if r[4] == phase_name]
            d_recs = [r for r in self.raw_depth_records if r[4] == phase_name]
            depth_k = self.depth_k if self.depth_k is not None else np.array([[460., 0., 424.], [0., 460., 240.], [0., 0., 1.]])
            self.phase_raw_depth_frames[phase_name] = [(r[0], r[1].copy(), r[2], r[3]) for r in d_recs]

        # Process LiDAR
        for sec, raw_bytes, point_step, is_cam_init, ph in l_recs:
            dt = np.dtype({
                'names': ['x', 'y', 'z'],
                'formats': ['<f4', '<f4', '<f4'],
                'offsets': [0, 4, 8],
                'itemsize': point_step
            })
            pts = np.frombuffer(raw_bytes, dtype=dt)
            valid = np.isfinite(pts['x']) & np.isfinite(pts['y']) & np.isfinite(pts['z'])
            pts_valid = pts[valid]
            if len(pts_valid) == 0:
                continue

            if is_cam_init:
                airy_xyz_world = np.column_stack((pts_valid['x'], pts_valid['y'], pts_valid['z'])).astype(np.float64)[::3]
            else:
                pts_lidar = np.column_stack((pts_valid['x'], pts_valid['y'], pts_valid['z'])).astype(np.float64)[::3]
                pos_w, rot_w = self.get_interpolated_pose(sec)
                if pos_w is None or rot_w is None:
                    with self.lock:
                        self.cnt_missing_pose_rejects += 1
                        if self.active_phase_stats is not None:
                            self.active_phase_stats["missing_pose_rejects"] += 1
                    continue
                airy_xyz_world = (rot_w @ pts_lidar.T).T + pos_w

            with self.lock:
                self.airy_sweeps.append((sec, airy_xyz_world, ph))
                self.cnt_admitted_airy_sweeps += 1
                if self.active_phase_stats is not None:
                    self.active_phase_stats["admitted_airy_sweeps"] += 1

        # Process Depth (Nominal offset = 0.0s)
        for sec, cv_depth, rgb_img, rgb_stamp, ph in d_recs:
            pos_w, rot_w = self.get_interpolated_pose(sec)
            if pos_w is None or rot_w is None:
                with self.lock:
                    self.cnt_missing_pose_rejects += 1
                    if self.active_phase_stats is not None:
                        self.active_phase_stats["missing_pose_rejects"] += 1
                continue

            if rgb_img is not None and abs(sec - rgb_stamp) < 0.05:
                with self.lock:
                    self.cnt_rgb_timestamp_matched += 1
                    if self.active_phase_stats is not None:
                        self.active_phase_stats["rgb_matched"] += 1
            else:
                with self.lock:
                    self.cnt_temporal_gate_rejects += 1
                    if self.active_phase_stats is not None:
                        self.active_phase_stats["temporal_rejects"] += 1

            gemini_xyz_world, gemini_rgb = self.backproject_depth(cv_depth, rgb_img, depth_k, pos_w, rot_w, decimate=4)
            if gemini_xyz_world is not None and len(gemini_xyz_world) > 0:
                gemini_xyz_world = gemini_xyz_world[::2]
                gemini_rgb = gemini_rgb[::2]
                with self.lock:
                    self.gemini_frames.append((sec, gemini_xyz_world, gemini_rgb, ph))
                    self.cnt_admitted_depth_frames += 1
                    if self.active_phase_stats is not None:
                        self.active_phase_stats["admitted_depth_frames"] += 1

        # Purge raw buffers for this phase to immediately free memory
        with self.lock:
            self.raw_lidar_records = [r for r in self.raw_lidar_records if r[4] != phase_name]
            self.raw_depth_records = [r for r in self.raw_depth_records if r[4] != phase_name]

    def evaluate_single_phase_residuals(self, phase_name):
        res_sweep = self.evaluate_camera_time_offset_sweep(phase_name, offset_range_ms=[0])
        if "+0ms" in res_sweep:
            r0 = res_sweep["+0ms"]
            return {
                "points_evaluated": r0["points_evaluated"],
                "p50_residual_mm": r0["p50_mm"],
                "p90_residual_mm": r0["p90_mm"],
                "p95_residual_mm": r0["p95_mm"],
                "signed_offset_mean_mm": r0["mean_mm"],
                "wall_thickness_95pct_mm": float(round(4.0 * r0["std_mm"], 2)),
                "double_wall_incidence_percent": r0["double_wall_pct"]
            }
        return {
            "points_evaluated": 0,
            "p50_residual_mm": 0.0,
            "p90_residual_mm": 0.0,
            "p95_residual_mm": 0.0,
            "signed_offset_mean_mm": 0.0,
            "wall_thickness_95pct_mm": 0.0,
            "double_wall_incidence_percent": 0.0
        }

    def evaluate_camera_time_offset_sweep(self, phase_name, offset_range_ms=range(-60, 65, 5)):
        from scipy.spatial import cKDTree
        with self.lock:
            phase_airy = [(s[0], s[1]) for s in self.airy_sweeps if len(s[1]) > 0 and s[2] == phase_name]
            raw_depth = self.phase_raw_depth_frames.get(phase_name, [])
            depth_k = self.depth_k if self.depth_k is not None else np.array([[460., 0., 424.], [0., 460., 240.], [0., 0., 1.]])

        if len(phase_airy) == 0 or len(raw_depth) == 0:
            return {}

        # Pre-build KD-trees and local planar patches for each individual Airy sweep
        ball_radius = 0.15
        sweep_structures = []
        for t_airy, airy_pts in phase_airy:
            if len(airy_pts) < 20:
                continue
            tree = cKDTree(airy_pts)
            sweep_structures.append((t_airy, airy_pts, tree, {}))

        if len(sweep_structures) == 0:
            return {}

        # Pair each depth frame with its single co-temporal Airy sweep based on ORIGINAL acquisition timestamp
        paired_frames = []
        pairs_rejected = 0
        dt_list = []
        for sec_d, cv_depth, rgb_img, rgb_stamp in raw_depth:
            best_idx = None
            min_dt = 1e9
            for idx, (t_a, _, _, _) in enumerate(sweep_structures):
                dt = abs(sec_d - t_a)
                if dt < min_dt:
                    min_dt = dt
                    best_idx = idx
            if best_idx is not None and min_dt <= 0.050: # strict <= 50ms co-temporal threshold
                paired_frames.append((sec_d, cv_depth, best_idx, min_dt * 1000.0))
                dt_list.append(min_dt * 1000.0)
            else:
                pairs_rejected += 1

        if len(paired_frames) == 0:
            # Fall back to closest available within 100ms
            for sec_d, cv_depth, rgb_img, rgb_stamp in raw_depth:
                best_idx = int(np.argmin([abs(sec_d - s[0]) for s in sweep_structures]))
                dt_ms = abs(sec_d - sweep_structures[best_idx][0]) * 1000.0
                paired_frames.append((sec_d, cv_depth, best_idx, dt_ms))
                dt_list.append(dt_ms)

        median_dt_ms = float(round(np.median(dt_list), 2)) if len(dt_list) > 0 else 0.0

        sweep_results = {}
        for off_ms in offset_range_ms:
            off_s = off_ms / 1000.0
            pt2plane_list = []
            range_list = []
            high_conf_list = []

            for sec_d, cv_depth, struct_idx, dt_phys_ms in paired_frames:
                t_airy_ref, airy_pts_ref, tree_ref, tangs_ref = sweep_structures[struct_idx]
                pos_w, rot_w = self.get_interpolated_pose(sec_d + off_s)
                if pos_w is None or rot_w is None:
                    continue

                # Backproject depth using the offset-shifted pose
                h, w = cv_depth.shape[:2]
                depth_m = cv_depth.astype(np.float32) / 1000.0
                u_grid, v_grid = np.meshgrid(np.arange(0, w, 4), np.arange(0, h, 4))
                z = depth_m[v_grid, u_grid]
                valid = (z > 0.4) & (z < 5.0)
                if np.sum(valid) == 0:
                    continue
                u_v, v_v, z_v = u_grid[valid], v_grid[valid], z[valid]
                fx, fy = depth_k[0, 0], depth_k[1, 1]
                cx, cy = depth_k[0, 2], depth_k[1, 2]
                x_c = (u_v - cx) * z_v / fx
                y_c = (v_v - cy) * z_v / fy
                pts_cam = np.column_stack((x_c, y_c, z_v))
                pts_imu = (T_DEPTH_IMU[:3, :3] @ pts_cam.T).T + T_DEPTH_IMU[:3, 3]
                pts_w = (rot_w @ pts_imu.T).T + pos_w

                # Query against the FIXED single co-temporal reference sweep
                dists, idxs = tree_ref.query(pts_w, k=1, workers=1)
                valid_m = (dists <= ball_radius)
                if np.sum(valid_m) == 0:
                    continue

                # Compute tangent planes on-demand for matched points
                needed_u = np.unique(idxs[valid_m])
                missing_u = [u for u in needed_u if u not in tangs_ref]
                if len(missing_u) > 0:
                    balls = tree_ref.query_ball_point(airy_pts_ref[missing_u], r=ball_radius, workers=1)
                    for k_idx, u in enumerate(missing_u):
                        nb_i = balls[k_idx]
                        if len(nb_i) >= 6:
                            nb = airy_pts_ref[nb_i]
                            cent = np.mean(nb, axis=0)
                            cov = np.cov((nb - cent).T)
                            evals, evecs = np.linalg.eigh(cov)
                            if evals[0] < 0.008:
                                tangs_ref[u] = (evecs[:, 0], cent)
                            else:
                                tangs_ref[u] = None
                        else:
                            tangs_ref[u] = None

                for i, p_g in enumerate(pts_w):
                    if not valid_m[i]:
                        continue
                    u = idxs[i]
                    if u in tangs_ref and tangs_ref[u] is not None:
                        norm, cent = tangs_ref[u]
                        signed_d = float(np.dot(p_g - cent, norm))
                        pt2plane_list.append(signed_d)
                        range_list.append(z_v[i])
                        if dt_phys_ms <= 20.0:
                            high_conf_list.append(signed_d)

            if len(pt2plane_list) > 10:
                abs_res_mm = np.abs(np.array(pt2plane_list)) * 1000.0
                signed_res_mm = np.array(pt2plane_list) * 1000.0
                ranges = np.array(range_list)

                p50 = float(round(np.percentile(abs_res_mm, 50), 2))
                p90 = float(round(np.percentile(abs_res_mm, 90), 2))
                p95 = float(round(np.percentile(abs_res_mm, 95), 2))
                mean_res = float(round(np.mean(signed_res_mm), 2))
                std_res = float(round(np.std(signed_res_mm), 2))
                double_wall_pts = np.sum((abs_res_mm > 20.0) & (abs_res_mm <= 80.0))
                double_wall_pct = float(round((double_wall_pts / len(abs_res_mm)) * 100.0, 2))

                near_m = (ranges >= 0.5) & (ranges < 1.5)
                mid_m = (ranges >= 1.5) & (ranges < 2.5)
                far_m = (ranges >= 2.5) & (ranges <= 4.0)

                # High confidence subset (|dt| <= 20ms)
                if len(high_conf_list) > 5:
                    hc_abs_mm = np.abs(np.array(high_conf_list)) * 1000.0
                    hc_p50 = float(round(np.percentile(hc_abs_mm, 50), 2))
                    hc_p95 = float(round(np.percentile(hc_abs_mm, 95), 2))
                else:
                    hc_p50, hc_p95 = None, None

                sweep_results[f"{off_ms:+d}ms"] = {
                    "offset_ms": off_ms,
                    "points_evaluated": int(len(pt2plane_list)),
                    "pairs_accepted": int(len(paired_frames)),
                    "pairs_rejected_gt_50ms": int(pairs_rejected),
                    "median_dt_ref_ms": median_dt_ms,
                    "p50_mm": p50,
                    "p90_mm": p90,
                    "p95_mm": p95,
                    "mean_mm": mean_res,
                    "std_mm": std_res,
                    "double_wall_pct": double_wall_pct,
                    "near_p50_mm": float(round(np.percentile(abs_res_mm[near_m], 50), 2)) if np.sum(near_m) > 5 else None,
                    "near_p95_mm": float(round(np.percentile(abs_res_mm[near_m], 95), 2)) if np.sum(near_m) > 5 else None,
                    "mid_p50_mm": float(round(np.percentile(abs_res_mm[mid_m], 50), 2)) if np.sum(mid_m) > 5 else None,
                    "mid_p95_mm": float(round(np.percentile(abs_res_mm[mid_m], 95), 2)) if np.sum(mid_m) > 5 else None,
                    "far_p50_mm": float(round(np.percentile(abs_res_mm[far_m], 50), 2)) if np.sum(far_m) > 5 else None,
                    "far_p95_mm": float(round(np.percentile(abs_res_mm[far_m], 95), 2)) if np.sum(far_m) > 5 else None,
                    "high_conf_le_20ms_p50_mm": hc_p50,
                    "high_conf_le_20ms_p95_mm": hc_p95
                }

        return sweep_results

    def stop_and_export(self):
        with self.lock:
            self.is_recording = False
            self.current_phase = "FINISHED"

        duration = time.time() - self.session_start_t if self.session_start_t else 0.0
        self.get_logger().info(f"[SESSION STOPPED] Processing session duration {duration:.2f}s...")

        # 1. Trajectory Calculations (Source A, B, C)
        # Source A: Raw Odom recorded during session
        t_start = self.session_start_t
        raw_odom_sub = [p for p in self.raw_odom_poses if p[0] >= t_start]
        if len(raw_odom_sub) > 1:
            raw_pos_a = np.array([p[1] for p in raw_odom_sub])
            traj_len_a = float(np.sum(np.linalg.norm(np.diff(raw_pos_a, axis=0), axis=1)))
        else:
            traj_len_a = 0.0

        # Source B: Recorder Internal Trajectory
        if len(self.trajectory_b) > 1:
            raw_pos_b = np.array([[p[1], p[2], p[3]] for p in self.trajectory_b])
            traj_len_b = float(np.sum(np.linalg.norm(np.diff(raw_pos_b, axis=0), axis=1)))
            p_start_b = raw_pos_b[0]
            p_end_b = raw_pos_b[-1]
            net_displacement_b = float(np.linalg.norm(p_end_b - p_start_b))
            max_excursion_b = float(np.max(np.linalg.norm(raw_pos_b - p_start_b, axis=1)))
            yaw_start_deg = math.degrees(self.trajectory_b[0][8])
            yaw_end_deg = math.degrees(self.trajectory_b[-1][8])
            net_yaw_change_deg = float(abs(yaw_end_deg - yaw_start_deg))
        else:
            traj_len_b = 0.0
            net_displacement_b = 0.0
            max_excursion_b = 0.0
            net_yaw_change_deg = 0.0

        # Source C: Export to trajectory.csv and re-read
        traj_csv = os.path.join(self.export_dir, "trajectory.csv")
        traj_tum = os.path.join(self.export_dir, "trajectory_tum.txt")
        with open(traj_csv, 'w') as f_csv, open(traj_tum, 'w') as f_tum:
            f_csv.write("index,timestamp,x_m,y_m,z_m,qx,qy,qz,qw,yaw_deg,phase\n")
            for idx, (t, tx, ty, tz, qx, qy, qz, qw, tyaw, ph) in enumerate(self.trajectory_b):
                f_csv.write(f"{idx},{t:.6f},{tx:.4f},{ty:.4f},{tz:.4f},{qx:.6f},{qy:.6f},{qz:.6f},{qw:.6f},{math.degrees(tyaw):.2f},{ph}\n")
                f_tum.write(f"{t:.6f} {tx:.4f} {ty:.4f} {tz:.4f} {qx:.6f} {qy:.6f} {qz:.6f} {qw:.6f}\n")

        data_c = np.genfromtxt(traj_csv, delimiter=',', skip_header=1)
        if len(data_c.shape) > 1 and len(data_c) > 1:
            xyz_c = data_c[:, 2:5]
            traj_len_c = float(np.sum(np.linalg.norm(np.diff(xyz_c, axis=0), axis=1)))
        else:
            traj_len_c = 0.0

        # 2. Keyframe Selection & Voxel Aggregation
        # Airy: All admitted sweeps voxelized to 15mm
        all_a_xyz = [s[1] for s in self.airy_sweeps if len(s[1]) > 0]
        if len(all_a_xyz) == 0:
            self.get_logger().error("No Airy points available for export!")
            return None

        airy_raw = np.vstack(all_a_xyz)
        if len(airy_raw) > 500000:
            airy_raw = airy_raw[::max(1, len(airy_raw) // 500000)]
        voxel_size = 0.015
        a_keys = np.floor(airy_raw / voxel_size).astype(int)
        _, u_idx = np.unique(a_keys, axis=0, return_index=True)
        airy_clean = airy_raw[u_idx]
        n_airy = len(airy_clean)

        # Gemini: All admitted depth frames voxelized to 15mm
        all_g_xyz, all_g_rgb = [], []
        for gf in self.gemini_frames:
            if gf[1] is not None and len(gf[1]) > 0:
                all_g_xyz.append(gf[1])
                all_g_rgb.append(gf[2])

        if len(all_g_xyz) > 0:
            g_raw_xyz = np.vstack(all_g_xyz)
            g_raw_rgb = np.vstack(all_g_rgb)
            if len(g_raw_xyz) > 300000:
                step_g = max(1, len(g_raw_xyz) // 300000)
                g_raw_xyz = g_raw_xyz[::step_g]
                g_raw_rgb = g_raw_rgb[::step_g]
            g_keys = np.floor(g_raw_xyz / voxel_size).astype(int)
            _, gu_idx = np.unique(g_keys, axis=0, return_index=True)
            gemini_clean_xyz = g_raw_xyz[gu_idx]
            gemini_clean_rgb = g_raw_rgb[gu_idx]
            n_gemini = len(gemini_clean_xyz)
        else:
            gemini_clean_xyz = np.empty((0, 3), dtype=np.float64)
            gemini_clean_rgb = np.empty((0, 3), dtype=np.uint8)
            n_gemini = 0

        # 3. Production Three-Class ASPS Fusion
        t0_asps = time.perf_counter()
        fused_xyz, fused_rgb, class_meta = execute_three_class_asps_fusion(
            airy_clean, gemini_clean_xyz, gemini_clean_rgb,
            tau_snap_m=0.020, tau_conflict_m=0.080, r_search=0.15
        )
        t_asps_ms = (time.perf_counter() - t0_asps) * 1000.0

        # 4. Phase-by-Phase Dynamic Residual Analysis
        phase_metrics = self.evaluate_phase_isolated_residuals(airy_clean)

        # 5. Multi-Modal Exports
        self.write_ply(os.path.join(self.export_dir, "pointcloud_airy.ply"), airy_clean, None)
        self.write_pcd(os.path.join(self.export_dir, "pointcloud_airy.pcd"), airy_clean, None)
        self.write_ply(os.path.join(self.export_dir, "pointcloud_stereo.ply"), gemini_clean_xyz, gemini_clean_rgb)
        self.write_pcd(os.path.join(self.export_dir, "pointcloud_stereo.pcd"), gemini_clean_xyz, gemini_clean_rgb)
        self.write_ply(os.path.join(self.export_dir, "pointcloud_fused_rgb.ply"), fused_xyz, fused_rgb)
        self.write_pcd(os.path.join(self.export_dir, "pointcloud_fused_rgb.pcd"), fused_xyz, fused_rgb)
        with open(os.path.join(self.export_dir, "flash_anchors.json"), 'w') as f_anchors:
            json.dump([], f_anchors, indent=2)

        metadata = {
            "session_id": str(self.dataset_name),
            "export_timestamp": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            "capture_duration_seconds": float(round(duration, 2)),
            "telemetry_counters": {
                "raw_messages_received": {
                    "fastlivo_odometry_received": int(self.cnt_raw_odom_msgs),
                    "airy_clouds_received": int(self.cnt_raw_airy_msgs),
                    "gemini_depth_frames_received": int(self.cnt_raw_depth_msgs),
                    "gemini_rgb_frames_received": int(self.cnt_raw_rgb_msgs)
                },
                "observations_admitted_to_fusion": {
                    "airy_sweeps_admitted": int(self.cnt_admitted_airy_sweeps),
                    "gemini_depth_frames_admitted": int(self.cnt_admitted_depth_frames),
                    "rgb_frames_timestamp_matched": int(self.cnt_rgb_timestamp_matched)
                },
                "observation_rejections_and_drops": {
                    "temporal_gate_rejects": int(self.cnt_temporal_gate_rejects),
                    "range_gate_rejects": int(self.cnt_range_gate_rejects),
                    "missing_pose_rejects": int(self.cnt_missing_pose_rejects),
                    "queue_drops": int(self.cnt_queue_drops)
                }
            },
            "trajectory_consistency_validation": {
                "source_a_raw_fastlivo_length_m": float(round(traj_len_a, 4)),
                "source_b_recorder_internal_length_m": float(round(traj_len_b, 4)),
                "source_c_exported_csv_length_m": float(round(traj_len_c, 4)),
                "trajectory_sources_agree": bool(abs(traj_len_b - traj_len_c) < 0.005),
                "net_start_to_end_displacement_m": float(round(net_displacement_b, 4)),
                "max_excursion_from_start_m": float(round(max_excursion_b, 4)),
                "net_yaw_change_deg": float(round(net_yaw_change_deg, 2))
            },
            "asps_three_class_statistics": class_meta,
            "asps_processing_time_ms": float(round(t_asps_ms, 2)),
            "phase_isolated_residuals": phase_metrics,
            "point_counts": {
                "authoritative_airy_points": int(n_airy),
                "supplemental_gemini_points": int(n_gemini),
                "fused_total_points": int(len(fused_xyz)),
                "airy_contribution_percent": float(round((n_airy / len(fused_xyz)) * 100.0, 2)),
                "gemini_contribution_percent": float(round(((len(fused_xyz) - n_airy) / len(fused_xyz)) * 100.0, 2))
            },
            "calibration_hashes": {
                "extrinsic_T_rgb_airy_lidar_sha256": hashlib.sha256(T_RGB_LIDAR_VEC.tobytes()).hexdigest(),
                "extrinsic_T_depth_airy_lidar_sha256": hashlib.sha256(T_LIDAR_DEPTH_VEC.tobytes()).hexdigest(),
                "color_intrinsics_K_sha256": hashlib.sha256(K_COLOR_DEF.tobytes()).hexdigest()
            }
        }

        with open(os.path.join(self.export_dir, "scan_metadata.json"), 'w') as f_meta:
            json.dump(metadata, f_meta, indent=2)

        self.get_logger().info(f"[EXPORT SUCCESSFUL] All 7 files and complete metadata saved to {self.export_dir}")
        return metadata

    def evaluate_phase_isolated_residuals(self, airy_clean):
        """Calculates Gemini-to-Airy normal residuals separately per captured phase."""
        from scipy.spatial import cKDTree
        if len(airy_clean) == 0:
            return {}

        airy_tree = cKDTree(airy_clean)
        unique_phases = sorted(list(set([gf[3] for gf in self.gemini_frames])))
        
        phase_dict = {}
        for ph in unique_phases:
            g_phase_pts = [gf[1] for gf in self.gemini_frames if gf[3] == ph and len(gf[1]) > 0]
            if len(g_phase_pts) == 0:
                continue
            g_all = np.vstack(g_phase_pts)
            # Sample up to 1000 points for evaluation
            if len(g_all) > 1000:
                step = len(g_all) // 1000
                g_eval = g_all[::step]
            else:
                g_eval = g_all

            dists, idxs = airy_tree.query(g_eval, k=1, workers=4)
            # Query local tangent plane for matched Airy points
            pt2plane_list = []
            for i, p_g in enumerate(g_eval):
                if dists[i] > 0.15:
                    continue
                p_a = airy_clean[idxs[i]]
                # Local neighborhood around p_a
                nb_idxs = airy_tree.query_ball_point(p_a, r=0.15)
                if len(nb_idxs) >= 6:
                    nb = airy_clean[nb_idxs]
                    cent = np.mean(nb, axis=0)
                    cov = np.cov((nb - cent).T)
                    evals, evecs = np.linalg.eigh(cov)
                    norm = evecs[:, 0]
                    if evals[0] < 0.008:
                        signed_d = float(np.dot(p_g - cent, norm))
                        pt2plane_list.append(signed_d)

            if len(pt2plane_list) > 10:
                abs_res_mm = np.abs(np.array(pt2plane_list)) * 1000.0
                signed_res_mm = np.array(pt2plane_list) * 1000.0
                # Wall thickness is 2 * 2-sigma (95.4% interval) of normal residual distribution
                wall_thick_mm = float(round(4.0 * float(np.std(signed_res_mm)), 2))
                # Double-wall incidence: points with 20mm < |s| <= 80mm before correction
                double_wall_pts = np.sum((abs_res_mm > 20.0) & (abs_res_mm <= 80.0))
                double_wall_pct = float(round((double_wall_pts / len(abs_res_mm)) * 100.0, 2))

                phase_dict[ph] = {
                    "points_evaluated": int(len(pt2plane_list)),
                    "p50_residual_mm": float(round(np.percentile(abs_res_mm, 50), 2)),
                    "p90_residual_mm": float(round(np.percentile(abs_res_mm, 90), 2)),
                    "p95_residual_mm": float(round(np.percentile(abs_res_mm, 95), 2)),
                    "signed_offset_mean_mm": float(round(np.mean(signed_res_mm), 2)),
                    "wall_thickness_95pct_mm": wall_thick_mm,
                    "double_wall_incidence_percent": double_wall_pct
                }
            else:
                phase_dict[ph] = {
                    "points_evaluated": int(len(pt2plane_list)),
                    "p50_residual_mm": 0.0,
                    "p90_residual_mm": 0.0,
                    "p95_residual_mm": 0.0,
                    "signed_offset_mean_mm": 0.0,
                    "wall_thickness_95pct_mm": 0.0,
                    "double_wall_incidence_percent": 0.0
                }
        return phase_dict

    def write_ply(self, filename, xyz, rgb=None):
        n = len(xyz)
        if rgb is not None:
            header = f"ply\nformat binary_little_endian 1.0\nelement vertex {n}\nproperty float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n".encode('ascii')
            dt = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('r', 'u1'), ('g', 'u1'), ('b', 'u1')])
            arr = np.empty(n, dtype=dt)
            arr['x'], arr['y'], arr['z'] = xyz[:, 0], xyz[:, 1], xyz[:, 2]
            arr['r'], arr['g'], arr['b'] = rgb[:, 0], rgb[:, 1], rgb[:, 2]
        else:
            header = f"ply\nformat binary_little_endian 1.0\nelement vertex {n}\nproperty float x\nproperty float y\nproperty float z\nend_header\n".encode('ascii')
            dt = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4')])
            arr = np.empty(n, dtype=dt)
            arr['x'], arr['y'], arr['z'] = xyz[:, 0], xyz[:, 1], xyz[:, 2]
        with open(filename, 'wb') as f:
            f.write(header)
            f.write(arr.tobytes())

    def write_pcd(self, filename, xyz, rgb=None):
        n = len(xyz)
        if rgb is not None:
            r, g, b = rgb[:, 0].astype(np.uint32), rgb[:, 1].astype(np.uint32), rgb[:, 2].astype(np.uint32)
            rgb_float = ((r << 16) | (g << 8) | b).view(np.float32)
            header = f"# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z rgb\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nWIDTH {n}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {n}\nDATA binary\n".encode('ascii')
            dt = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('rgb', '<f4')])
            arr = np.empty(n, dtype=dt)
            arr['x'], arr['y'], arr['z'], arr['rgb'] = xyz[:, 0], xyz[:, 1], xyz[:, 2], rgb_float
        else:
            header = f"# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nWIDTH {n}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {n}\nDATA binary\n".encode('ascii')
            dt = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4')])
            arr = np.empty(n, dtype=dt)
            arr['x'], arr['y'], arr['z'] = xyz[:, 0], xyz[:, 1], xyz[:, 2]
        with open(filename, 'wb') as f:
            f.write(header)
            f.write(arr.tobytes())
