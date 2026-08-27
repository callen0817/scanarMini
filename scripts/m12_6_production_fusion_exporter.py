#!/usr/bin/env python3
"""
Milestone M12.6: Production Real-Time Dual-Mode Fusion Pipeline & Multi-Modal Exporter
Features:
1. LIVE FUSION: Low-latency SSD policy (tau=20mm) emitting /scanar/live_fused_cloud at <25ms
   for real-time GUI preview without blocking FAST-LIVO2.
2. FINAL FUSION: High-fidelity ASPS policy (tau=20mm) with local tangent plane snapping:
   p_G' = p_G - [n^T (p_G - p_A)] n, preserving tangential detail and RGB color attributes.
3. MULTI-MODAL EXPORTER: On STOP/SAVE, exports:
   - pointcloud_airy.ply / .pcd
   - pointcloud_stereo.ply / .pcd
   - pointcloud_fused_rgb.ply / .pcd
   - trajectory.csv
   - trajectory_tum.txt
   - flash_anchors.json
   - scan_metadata.json
"""

import os
import sys
import time
import math
import json
import struct
import cv2
import hashlib
import numpy as np
from scipy.spatial import cKDTree
from collections import deque
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, PointField, Image, CameraInfo
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge

# Calibrated Extrinsics (T_cam_lidar)
T_RGB_LIDAR = np.array([
    [ 0.000350, -0.999997,  0.002377,  0.023855],
    [-0.001316, -0.002378, -0.999996,  0.014534],
    [ 0.999999,  0.000347, -0.001317, -0.055265],
    [ 0.000000,  0.000000,  0.000000,  1.000000]
], dtype=np.float64)

# Optical to LiDAR Coordinate Transformation for Gemini Depth
# Depth optical frame: X right, Y down, Z forward
# Airy LiDAR frame: X forward, Y left, Z up
T_DEPTH_LIDAR = np.array([
    [ 0.000000, -1.000000,  0.000000,  0.047640],
    [ 0.000000,  0.000000, -1.000000,  0.014311],
    [ 1.000000,  0.000000,  0.000000, -0.055700],
    [ 0.000000,  0.000000,  0.000000,  1.000000]
], dtype=np.float64)

# Default Color Intrinsics
K_COLOR_DEF = np.array([
    [609.006531,   0.000000, 643.868774],
    [  0.000000, 609.015625, 399.326538],
    [  0.000000,   0.000000,   1.000000]
], dtype=np.float64)
D_COLOR_DEF = np.array([-0.03113378, 0.03646491, 0.00048966, -0.00030747, -0.01320058], dtype=np.float64)


def quat_to_rot(q):
    """Convert [qx, qy, qz, qw] to 3x3 rotation matrix."""
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


class M126ProductionFusionNode(Node):
    def __init__(self, duration_sec=15.0, dataset_name="M12_6_Production_Capture"):
        super().__init__('m12_6_production_fusion_node')
        self.duration_sec = duration_sec
        self.dataset_name = dataset_name
        self.export_dir = f"/home/scanar/scanarMini/data/{dataset_name}"
        os.makedirs(self.export_dir, exist_ok=True)

        self.bridge = CvBridge()
        self.lock = threading.Lock()

        # Buffers for Live State Estimation & Raw Observations
        self.trajectory = [] # [(stamp, x, y, z, qx, qy, qz, qw, yaw)]
        self.odom_buffer = deque(maxlen=500) # [(stamp, pos, rot)]
        
        # Point Buffers (World Frame)
        self.airy_sweeps = []    # List of (stamp, np.ndarray (N, 3))
        self.gemini_frames = []  # List of (stamp, np.ndarray (M, 3), np.ndarray (M, 3)) [xyz, rgb]

        # Live State
        self.latest_rgb_image = None
        self.latest_rgb_stamp = 0.0
        self.latest_depth_image = None
        self.latest_depth_stamp = 0.0
        self.color_k = K_COLOR_DEF.copy()
        self.color_d = D_COLOR_DEF.copy()
        self.depth_k = None

        self.is_recording = False
        self.start_time = None
        self.sweep_count = 0
        self.depth_frame_count = 0

        # Live Preview Publisher
        qos_pub = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        self.pub_live_fused = self.create_publisher(
            PointCloud2,
            '/scanar/live_fused_cloud',
            qos_pub
        )

        # QoS for high-rate sensor streams
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscriptions
        self.sub_odom = self.create_subscription(
            Odometry,
            '/aft_mapped_to_init',
            self.odom_callback,
            qos_sensor
        )

        self.sub_lidar = self.create_subscription(
            PointCloud2,
            '/cloud_registered',
            self.lidar_callback,
            qos_sensor
        )

        self.sub_rgb = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.rgb_callback,
            qos_sensor
        )

        self.sub_depth = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.depth_callback,
            qos_sensor
        )

        self.sub_depth_rect = self.create_subscription(
            Image,
            '/camera/depth/image_rect_raw',
            self.depth_callback,
            qos_sensor
        )

        self.sub_depth_info = self.create_subscription(
            CameraInfo,
            '/camera/depth/camera_info',
            self.depth_info_callback,
            qos_sensor
        )

        self.get_logger().info(f"M12.6 Production Fusion Node Ready. Target Session: {self.duration_sec}s -> {self.export_dir}")

    def odom_callback(self, msg: Odometry):
        sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        pz = msg.pose.pose.position.z
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        
        pos = np.array([px, py, pz], dtype=np.float64)
        rot = quat_to_rot([qx, qy, qz, qw])

        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        with self.lock:
            self.odom_buffer.append((sec, pos, rot))
            if self.is_recording:
                self.trajectory.append((sec, px, py, pz, qx, qy, qz, qw, yaw))

    def rgb_callback(self, msg: Image):
        sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            with self.lock:
                self.latest_rgb_image = cv_img
                self.latest_rgb_stamp = sec
        except Exception as e:
            pass

    def depth_info_callback(self, msg: CameraInfo):
        if self.depth_k is None:
            self.depth_k = np.array(msg.k, dtype=np.float64).reshape((3, 3))

    def depth_callback(self, msg: Image):
        sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        try:
            cv_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            with self.lock:
                self.latest_depth_image = cv_depth
                self.latest_depth_stamp = sec
                self.depth_frame_count += 1
        except Exception as e:
            pass

    def get_interpolated_pose(self, query_time):
        with self.lock:
            if len(self.odom_buffer) == 0:
                return None, None
            if len(self.odom_buffer) == 1:
                return self.odom_buffer[0][1], self.odom_buffer[0][2]

            times = [item[0] for item in self.odom_buffer]
            if query_time <= times[0]:
                return self.odom_buffer[0][1], self.odom_buffer[0][2]
            if query_time >= times[-1]:
                return self.odom_buffer[-1][1], self.odom_buffer[-1][2]

            idx = np.searchsorted(times, query_time)
            t0, p0, r0 = self.odom_buffer[idx - 1]
            t1, p1, r1 = self.odom_buffer[idx]
            dt = t1 - t0
            alpha = (query_time - t0) / dt if dt > 1e-6 else 0.0
            p_interp = (1.0 - alpha) * p0 + alpha * p1
            # Nearest rotation
            r_interp = r1 if alpha >= 0.5 else r0
            return p_interp, r_interp

    def lidar_callback(self, msg: PointCloud2):
        if not self.is_recording:
            return

        sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        num_points = msg.width * msg.height
        if num_points == 0 or msg.point_step < 12:
            return

        self.sweep_count += 1
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

        airy_xyz_world = np.column_stack((pts_valid['x'], pts_valid['y'], pts_valid['z'])).astype(np.float64)

        # Buffer Airy sweep
        self.airy_sweeps.append((sec, airy_xyz_world))

        # Process accompanying Gemini Depth frame if available
        t_start_live = time.perf_counter()
        gemini_xyz_world = None
        gemini_rgb = None

        with self.lock:
            depth_img = self.latest_depth_image.copy() if self.latest_depth_image is not None else None
            depth_stamp = self.latest_depth_stamp
            rgb_img = self.latest_rgb_image.copy() if self.latest_rgb_image is not None else None
            depth_k = self.depth_k if self.depth_k is not None else np.array([[460., 0., 424.], [0., 460., 240.], [0., 0., 1.]])

        if depth_img is not None and abs(sec - depth_stamp) < 0.10:
            pos_w, rot_w = self.get_interpolated_pose(depth_stamp)
            if pos_w is not None and rot_w is not None:
                gemini_xyz_world, gemini_rgb = self.backproject_depth_frame(
                    depth_img, rgb_img, depth_k, pos_w, rot_w, decimate=4
                )
                if gemini_xyz_world is not None and len(gemini_xyz_world) > 0:
                    self.gemini_frames.append((depth_stamp, gemini_xyz_world, gemini_rgb))

        # LIVE FUSION: Fast SSD (tau = 20mm) for GUI Visualization (< 25 ms)
        if gemini_xyz_world is not None and len(gemini_xyz_world) > 0:
            live_fused_xyz, live_fused_rgb = self.execute_live_ssd_fusion(airy_xyz_world, gemini_xyz_world, gemini_rgb, tau_m=0.020)
        else:
            live_fused_xyz = airy_xyz_world
            live_fused_rgb = np.full((len(airy_xyz_world), 3), 160, dtype=np.uint8)

        t_elapsed_live_ms = (time.perf_counter() - t_start_live) * 1000.0

        # Publish Live Fused Cloud
        self.publish_live_cloud(live_fused_xyz, live_fused_rgb, msg.header.stamp)

    def backproject_depth_frame(self, depth_raw, rgb_img, K_depth, pos_w, rot_w, decimate=4):
        """Backprojects depth image to world-frame point cloud with color."""
        h, w = depth_raw.shape[:2]
        depth_m = depth_raw.astype(np.float32) / 1000.0 # mm to meters

        # Subsample grid for compute efficiency
        u_grid, v_grid = np.meshgrid(np.arange(0, w, decimate), np.arange(0, h, decimate))
        z = depth_m[v_grid, u_grid]

        # Valid depth mask: 0.35m to 4.5m
        valid = (z >= 0.35) & (z <= 4.5) & np.isfinite(z)
        if np.sum(valid) == 0:
            return None, None

        u_v = u_grid[valid]
        v_v = v_grid[valid]
        z_v = z[valid]

        fx = K_depth[0, 0]
        fy = K_depth[1, 1]
        cx = K_depth[0, 2]
        cy = K_depth[1, 2]

        x_c = (u_v - cx) * z_v / fx
        y_c = (v_v - cy) * z_v / fy
        pts_cam = np.column_stack((x_c, y_c, z_v))

        # Transform Cam -> LiDAR -> World
        # pts_lidar = T_DEPTH_LIDAR[:3, :3] @ pts_cam.T + T_DEPTH_LIDAR[:3, 3]
        pts_lidar = (T_DEPTH_LIDAR[:3, :3] @ pts_cam.T).T + T_DEPTH_LIDAR[:3, 3]
        pts_world = (rot_w @ pts_lidar.T).T + pos_w

        # RGB Colorization from latest color frame
        colors = np.full((len(pts_world), 3), 180, dtype=np.uint8)
        if rgb_img is not None:
            # Project pts_lidar into RGB camera
            pts_cam_rgb = (T_RGB_LIDAR[:3, :3] @ pts_lidar.T).T + T_RGB_LIDAR[:3, 3]
            z_rgb = pts_cam_rgb[:, 2]
            valid_z = z_rgb > 0.2

            fx_c = self.color_k[0, 0]
            fy_c = self.color_k[1, 1]
            cx_c = self.color_k[0, 2]
            cy_c = self.color_k[1, 2]

            u_rgb = (pts_cam_rgb[valid_z, 0] * fx_c / z_rgb[valid_z] + cx_c).astype(int)
            v_rgb = (pts_cam_rgb[valid_z, 1] * fy_c / z_rgb[valid_z] + cy_c).astype(int)

            img_h, img_w = rgb_img.shape[:2]
            in_bounds = (u_rgb >= 0) & (u_rgb < img_w) & (v_rgb >= 0) & (v_rgb < img_h)
            
            valid_indices = np.where(valid_z)[0][in_bounds]
            u_valid = u_rgb[in_bounds]
            v_valid = v_rgb[in_bounds]

            # OpenCV image is BGR -> convert to RGB
            bgr_samples = rgb_img[v_valid, u_valid]
            colors[valid_indices] = bgr_samples[:, [2, 1, 0]]

        return pts_world, colors

    def execute_live_ssd_fusion(self, airy_xyz, gemini_xyz, gemini_rgb, tau_m=0.020):
        """Ultra-fast Selective Source Decimation for real-time GUI preview."""
        tree = cKDTree(airy_xyz)
        dists, _ = tree.query(gemini_xyz, k=1, workers=2)
        
        # Keep non-overlapping Gemini points
        keep_gemini = dists > tau_m
        if np.sum(keep_gemini) > 0:
            fused_xyz = np.vstack((airy_xyz, gemini_xyz[keep_gemini]))
            airy_rgb = np.full((len(airy_xyz), 3), 140, dtype=np.uint8)
            fused_rgb = np.vstack((airy_rgb, gemini_rgb[keep_gemini]))
        else:
            fused_xyz = airy_xyz
            fused_rgb = np.full((len(airy_xyz), 3), 140, dtype=np.uint8)

        return fused_xyz, fused_rgb

    def publish_live_cloud(self, xyz, rgb, stamp):
        """Publishes sensor_msgs/PointCloud2 with XYZ + RGB."""
        n = len(xyz)
        if n == 0:
            return

        msg = PointCloud2()
        msg.header.stamp = stamp
        msg.header.frame_id = "camera_init"
        msg.height = 1
        msg.width = n
        msg.is_dense = True
        msg.is_bigendian = False

        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        msg.point_step = 16
        msg.row_step = msg.point_step * n

        r = rgb[:, 0].astype(np.uint32)
        g = rgb[:, 1].astype(np.uint32)
        b = rgb[:, 2].astype(np.uint32)
        rgb_packed = (r << 16) | (g << 8) | b
        rgb_float = rgb_packed.view(np.float32)

        dt = np.dtype([
            ('x', '<f4'), ('y', '<f4'), ('z', '<f4'),
            ('rgb', '<f4')
        ])
        arr = np.empty(n, dtype=dt)
        arr['x'] = xyz[:, 0]
        arr['y'] = xyz[:, 1]
        arr['z'] = xyz[:, 2]
        arr['rgb'] = rgb_float

        msg.data = arr.tobytes()
        self.pub_live_fused.publish(msg)

    def start_recording(self):
        self.is_recording = True
        self.start_time = time.time()
        self.get_logger().info(f"[RECORDING STARTED] Capturing multi-modal streams for {self.duration_sec}s...")

    def stop_and_export(self):
        self.is_recording = False
        duration = time.time() - self.start_time if self.start_time else 0.0
        self.get_logger().info(f"[RECORDING STOPPED] Captured {self.sweep_count} Airy sweeps, {len(self.gemini_frames)} Gemini frames in {duration:.2f}s.")
        self.get_logger().info(f"[FINAL ASPS FUSION] Building authoritative world map and executing ASPS (tau=20mm)...")

        # 1. Aggregate All Airy Points & Voxel Downsample
        all_airy_list = [sweep[1] for sweep in self.airy_sweeps if len(sweep[1]) > 0]
        if len(all_airy_list) == 0:
            self.get_logger().error("No Airy points recorded!")
            return None

        airy_raw_world = np.vstack(all_airy_list)
        self.get_logger().info(f"Total Raw Airy Points: {len(airy_raw_world):,}")

        # Voxel Grid Filter on Airy Backbone (15mm voxel)
        voxel_size = 0.015 # 15 mm resolution
        airy_vox_keys = np.floor(airy_raw_world / voxel_size).astype(int)
        _, unique_idx = np.unique(airy_vox_keys, axis=0, return_index=True)
        airy_world_clean = airy_raw_world[unique_idx]
        n_airy = len(airy_world_clean)
        self.get_logger().info(f"Voxelized Authoritative Airy Backbone: {n_airy:,} points (15mm resolution).")

        # 2. Aggregate All Gemini Depth Points & RGB
        all_gemini_xyz = []
        all_gemini_rgb = []
        for g_frame in self.gemini_frames:
            if g_frame[1] is not None and len(g_frame[1]) > 0:
                all_gemini_xyz.append(g_frame[1])
                all_gemini_rgb.append(g_frame[2])

        if len(all_gemini_xyz) > 0:
            gemini_raw_world = np.vstack(all_gemini_xyz)
            gemini_raw_rgb = np.vstack(all_gemini_rgb)
            # Voxel filter Gemini at 15mm
            g_vox_keys = np.floor(gemini_raw_world / voxel_size).astype(int)
            _, g_unique_idx = np.unique(g_vox_keys, axis=0, return_index=True)
            gemini_world_clean = gemini_raw_world[g_unique_idx]
            gemini_rgb_clean = gemini_raw_rgb[g_unique_idx]
            n_gemini = len(gemini_world_clean)
            self.get_logger().info(f"Voxelized Gemini Stereo/RGB Cloud: {n_gemini:,} points.")
        else:
            gemini_world_clean = np.empty((0, 3), dtype=np.float64)
            gemini_rgb_clean = np.empty((0, 3), dtype=np.uint8)
            n_gemini = 0

        # 3. Execute High-Precision ASPS Fusion (tau = 20 mm)
        t_asps_start = time.perf_counter()
        fused_xyz, fused_rgb, asps_meta = self.execute_final_asps_fusion(
            airy_world_clean, gemini_world_clean, gemini_rgb_clean, tau_m=0.020, r_search=0.15
        )
        t_asps_ms = (time.perf_counter() - t_asps_start) * 1000.0
        self.get_logger().info(f"ASPS Final Fusion completed in {t_asps_ms:.2f} ms: {len(fused_xyz):,} total fused points.")

        # 4. Multi-Modal Dataset Export
        # Artifact 1: pointcloud_airy.ply / .pcd
        airy_ply = os.path.join(self.export_dir, "pointcloud_airy.ply")
        airy_pcd = os.path.join(self.export_dir, "pointcloud_airy.pcd")
        self.write_ply(airy_ply, airy_world_clean, None)
        self.write_pcd(airy_pcd, airy_world_clean, None)

        # Artifact 2: pointcloud_stereo.ply / .pcd
        stereo_ply = os.path.join(self.export_dir, "pointcloud_stereo.ply")
        stereo_pcd = os.path.join(self.export_dir, "pointcloud_stereo.pcd")
        self.write_ply(stereo_ply, gemini_world_clean, gemini_rgb_clean)
        self.write_pcd(stereo_pcd, gemini_world_clean, gemini_rgb_clean)

        # Artifact 3: pointcloud_fused_rgb.ply / .pcd
        fused_ply = os.path.join(self.export_dir, "pointcloud_fused_rgb.ply")
        fused_pcd = os.path.join(self.export_dir, "pointcloud_fused_rgb.pcd")
        self.write_ply(fused_ply, fused_xyz, fused_rgb)
        self.write_pcd(fused_pcd, fused_xyz, fused_rgb)

        # Artifact 4 & 5: Trajectory CSV & TUM
        traj_csv = os.path.join(self.export_dir, "trajectory.csv")
        traj_tum = os.path.join(self.export_dir, "trajectory_tum.txt")
        with open(traj_csv, 'w') as f_csv, open(traj_tum, 'w') as f_tum:
            f_csv.write("index,timestamp,x_m,y_m,z_m,qx,qy,qz,qw,yaw_deg\n")
            for idx, (t, tx, ty, tz, qx, qy, qz, qw, tyaw) in enumerate(self.trajectory):
                f_csv.write(f"{idx},{t:.6f},{tx:.4f},{ty:.4f},{tz:.4f},{qx:.6f},{qy:.6f},{qz:.6f},{qw:.6f},{math.degrees(tyaw):.2f}\n")
                f_tum.write(f"{t:.6f} {tx:.4f} {ty:.4f} {tz:.4f} {qx:.6f} {qy:.6f} {qz:.6f} {qw:.6f}\n")

        # Artifact 6: Flash Anchors JSON
        flash_anchors = []
        anchors_json = os.path.join(self.export_dir, "flash_anchors.json")
        with open(anchors_json, 'w') as f_anchors:
            json.dump(flash_anchors, f_anchors, indent=2)

        # Trajectory Length
        traj_arr = np.array([[p[1], p[2], p[3]] for p in self.trajectory])
        if len(traj_arr) > 1:
            step_dists = np.linalg.norm(np.diff(traj_arr, axis=0), axis=1)
            traj_length_m = float(np.sum(step_dists))
        else:
            traj_length_m = 0.0

        # Artifact 7: Scan Metadata JSON
        meta_json = os.path.join(self.export_dir, "scan_metadata.json")
        meta_data = {
            "session_id": str(self.dataset_name),
            "export_timestamp": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            "capture_duration_seconds": float(round(duration, 2)),
            "trajectory_length_meters": float(round(traj_length_m, 3)),
            "total_sweeps_recorded": int(self.sweep_count),
            "total_depth_frames_recorded": int(len(self.gemini_frames)),
            "fusion_method": "ASPS (Adaptive Surface Projection & Snapping)",
            "association_tolerance_mm": float(20.0),
            "voxel_resolution_mm": float(15.0),
            "point_counts": {
                "authoritative_airy_points": int(n_airy),
                "supplemental_gemini_points": int(n_gemini),
                "fused_total_points": int(len(fused_xyz)),
                "airy_contribution_percent": float(round((n_airy / len(fused_xyz)) * 100.0, 2)),
                "gemini_contribution_percent": float(round(((len(fused_xyz) - n_airy) / len(fused_xyz)) * 100.0, 2))
            },
            "asps_classification_statistics": asps_meta,
            "calibration_hashes": {
                "extrinsic_T_rgb_airy_lidar_sha256": hashlib.sha256(T_RGB_LIDAR.tobytes()).hexdigest(),
                "extrinsic_T_depth_airy_lidar_sha256": hashlib.sha256(T_DEPTH_LIDAR.tobytes()).hexdigest(),
                "color_intrinsics_K_sha256": hashlib.sha256(self.color_k.tobytes()).hexdigest()
            },
            "system_hardware": {
                "lidar_model": "RoboSense Airy 192 (10Hz Master)",
                "imu_model": "RoboSense Internal IMU (200Hz)",
                "camera_model": "Orbbec Gemini 336L (Stereo Depth 30Hz + RGB 30Hz)",
                "slam_backend": "FAST-LIVO2 (Authoritative Trajectory & Geometry)",
                "fusion_engine": "Dual-Path Real-Time SSD (Live GUI) + Post-LIO ASPS (Final Export)"
            }
        }

        with open(meta_json, 'w') as f_meta:
            json.dump(meta_data, f_meta, indent=2)

        self.get_logger().info(f"[EXPORT COMPLETE] All 7 production artifacts written to {self.export_dir}")
        return meta_data

    def execute_final_asps_fusion(self, airy_xyz, gemini_xyz, gemini_rgb, tau_m=0.020, r_search=0.15):
        """
        Production High-Precision ASPS Fusion.
        Snapping Rule: p_G' = p_G - [n^T (p_G - p_A)] n
        Preserves Gemini in-plane detail and RGB while eliminating normal offset.
        """
        n_airy = len(airy_xyz)
        n_gemini = len(gemini_xyz)
        if n_gemini == 0:
            return airy_xyz, np.full((n_airy, 3), 160, dtype=np.uint8), {}

        # 1. Build Airy KDTree
        tree = cKDTree(airy_xyz)
        dists_euclid, idxs_a = tree.query(gemini_xyz, k=1, workers=4)

        # 2. Precompute Tangent Planes on unique matched Airy anchors
        unique_uids = np.unique(idxs_a)
        ball_idxs = tree.query_ball_point(airy_xyz[unique_uids], r=r_search, workers=4)
        tangents = {}
        for i, uid in enumerate(unique_uids):
            idxs = ball_idxs[i]
            if len(idxs) >= 6:
                sub = airy_xyz[idxs]
                cent = np.mean(sub, axis=0)
                cov = np.cov((sub - cent).T)
                evals, evecs = np.linalg.eigh(cov)
                norm = evecs[:, 0]
                if evals[0] < 0.008: # Clean planar patch
                    if np.dot(norm, cent) > 0:
                        norm = -norm
                    tangents[uid] = (norm, cent, True)
                else:
                    tangents[uid] = (norm, cent, False)
            else:
                tangents[uid] = (np.array([0., 1., 0.]), airy_xyz[uid], False)

        # 3. Compute Signed Point-to-Plane Distance
        pt2plane_signed = np.zeros(n_gemini, dtype=np.float64)
        is_planar = np.zeros(n_gemini, dtype=bool)
        for i in range(n_gemini):
            uid = idxs_a[i]
            norm_t, cent_t, is_pl = tangents[uid]
            is_planar[i] = is_pl
            if is_pl:
                pt2plane_signed[i] = np.dot(gemini_xyz[i] - cent_t, norm_t)
            else:
                pt2plane_signed[i] = dists_euclid[i]

        pt2plane_abs = np.abs(pt2plane_signed)

        # Gating: Far-field Tier 3 (>4m) requires closer agreement
        r_gemini = np.linalg.norm(gemini_xyz, axis=1)
        rejected_far = (r_gemini >= 4.0) & (dists_euclid > max(tau_m, 0.05))

        # Snapping condition: within tau_m of Airy tangent plane and search radius <= (tau_m + 0.10)
        overlap_mask = (pt2plane_abs <= tau_m) & is_planar & (dists_euclid <= (tau_m + 0.10)) & (~rejected_far)

        gemini_processed = gemini_xyz.copy()
        overlap_indices = np.where(overlap_mask)[0]
        for idx in overlap_indices:
            uid = idxs_a[idx]
            norm_t, cent_t, _ = tangents[uid]
            s = pt2plane_signed[idx]
            # Precise Projection: p_G' = p_G - [n^T (p_G - cent_A)] n
            gemini_processed[idx] = gemini_xyz[idx] - (s * norm_t)

        admitted_g = ~rejected_far
        fused_xyz = np.vstack((airy_xyz, gemini_processed[admitted_g]))
        airy_rgb = np.full((n_airy, 3), 160, dtype=np.uint8)
        fused_rgb = np.vstack((airy_rgb, gemini_rgb[admitted_g]))

        snapped_cnt = int(np.sum(overlap_mask))
        rejected_cnt = int(np.sum(rejected_far))
        gemini_only_cnt = int(np.sum(admitted_g & (~overlap_mask)))

        asps_meta = {
            "total_gemini_candidates": int(n_gemini),
            "snapped_overlap_points": snapped_cnt,
            "snapped_overlap_percent": float(round((snapped_cnt / n_gemini) * 100.0, 2)),
            "rejected_far_field_points": rejected_cnt,
            "rejected_far_field_percent": float(round((rejected_cnt / n_gemini) * 100.0, 2)),
            "retained_gemini_only_points": gemini_only_cnt,
            "retained_gemini_only_percent": float(round((gemini_only_cnt / n_gemini) * 100.0, 2)),
            "density_gain_over_airy": float(round(len(fused_xyz) / n_airy, 2))
        }

        return fused_xyz, fused_rgb, asps_meta

    def write_ply(self, filename, xyz, rgb=None):
        n = len(xyz)
        if rgb is not None:
            header = (
                "ply\n"
                "format binary_little_endian 1.0\n"
                f"element vertex {n}\n"
                "property float x\n"
                "property float y\n"
                "property float z\n"
                "property uchar red\n"
                "property uchar green\n"
                "property uchar blue\n"
                "end_header\n"
            ).encode('ascii')
            dt = np.dtype([
                ('x', '<f4'), ('y', '<f4'), ('z', '<f4'),
                ('r', 'u1'), ('g', 'u1'), ('b', 'u1')
            ])
            arr = np.empty(n, dtype=dt)
            arr['x'] = xyz[:, 0]
            arr['y'] = xyz[:, 1]
            arr['z'] = xyz[:, 2]
            arr['r'] = rgb[:, 0]
            arr['g'] = rgb[:, 1]
            arr['b'] = rgb[:, 2]
        else:
            header = (
                "ply\n"
                "format binary_little_endian 1.0\n"
                f"element vertex {n}\n"
                "property float x\n"
                "property float y\n"
                "property float z\n"
                "end_header\n"
            ).encode('ascii')
            dt = np.dtype([
                ('x', '<f4'), ('y', '<f4'), ('z', '<f4')
            ])
            arr = np.empty(n, dtype=dt)
            arr['x'] = xyz[:, 0]
            arr['y'] = xyz[:, 1]
            arr['z'] = xyz[:, 2]

        with open(filename, 'wb') as f:
            f.write(header)
            f.write(arr.tobytes())

    def write_pcd(self, filename, xyz, rgb=None):
        n = len(xyz)
        if rgb is not None:
            r = rgb[:, 0].astype(np.uint32)
            g = rgb[:, 1].astype(np.uint32)
            b = rgb[:, 2].astype(np.uint32)
            rgb_packed_int = (r << 16) | (g << 8) | b
            rgb_packed_float = rgb_packed_int.view(np.float32)

            header = (
                "# .PCD v0.7 - Point Cloud Data file format\n"
                "VERSION 0.7\n"
                "FIELDS x y z rgb\n"
                "SIZE 4 4 4 4\n"
                "TYPE F F F F\n"
                "COUNT 1 1 1 1\n"
                f"WIDTH {n}\n"
                "HEIGHT 1\n"
                "VIEWPOINT 0 0 0 1 0 0 0\n"
                f"POINTS {n}\n"
                "DATA binary\n"
            ).encode('ascii')

            dt = np.dtype([
                ('x', '<f4'), ('y', '<f4'), ('z', '<f4'),
                ('rgb', '<f4')
            ])
            arr = np.empty(n, dtype=dt)
            arr['x'] = xyz[:, 0]
            arr['y'] = xyz[:, 1]
            arr['z'] = xyz[:, 2]
            arr['rgb'] = rgb_packed_float
        else:
            header = (
                "# .PCD v0.7 - Point Cloud Data file format\n"
                "VERSION 0.7\n"
                "FIELDS x y z\n"
                "SIZE 4 4 4\n"
                "TYPE F F F\n"
                "COUNT 1 1 1\n"
                f"WIDTH {n}\n"
                "HEIGHT 1\n"
                "VIEWPOINT 0 0 0 1 0 0 0\n"
                f"POINTS {n}\n"
                "DATA binary\n"
            ).encode('ascii')

            dt = np.dtype([
                ('x', '<f4'), ('y', '<f4'), ('z', '<f4')
            ])
            arr = np.empty(n, dtype=dt)
            arr['x'] = xyz[:, 0]
            arr['y'] = xyz[:, 1]
            arr['z'] = xyz[:, 2]

        with open(filename, 'wb') as f:
            f.write(header)
            f.write(arr.tobytes())


def main():
    rclpy.init()
    duration = 15.0
    dataset = "M12_6_Production_Capture"
    if len(sys.argv) > 1:
        duration = float(sys.argv[1])
    if len(sys.argv) > 2:
        dataset = sys.argv[2]

    node = M126ProductionFusionNode(duration_sec=duration, dataset_name=dataset)

    # Spin in separate thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    time.sleep(1.0)
    node.start_recording()

    start_t = time.time()
    while (time.time() - start_t) < duration:
        time.sleep(0.5)
        elapsed = time.time() - start_t
        print(f"\r[M12.6 Live Session] Elapsed: {elapsed:.1f}s / {duration:.1f}s | Airy Sweeps: {node.sweep_count} | Depth Frames: {len(node.gemini_frames)}", end="", flush=True)
    print()

    node.stop_and_export()
    rclpy.shutdown()
    spin_thread.join(timeout=2.0)
    print("[M12.6 Execution Complete]")

if __name__ == '__main__':
    main()
