#!/usr/bin/env python3
"""
Milestone M12.8: Production MVP Acceptance Field Scan Runner

Executes a realistic, room-scale indoor field scan with live multi-modal
odometry, LiDAR point clouds, and SLERP-interpolated RGB-D integration.
Exports the standard production point cloud bundle:
  - pointcloud_fused_rgb.ply / .pcd
  - pointcloud_airy.ply / .pcd
  - pointcloud_stereo.ply / .pcd
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
import argparse
import threading
import struct
import numpy as np
from collections import deque
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, Image, CameraInfo
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge

# Calibrated Extrinsics (T_cam_lidar)
T_RGB_LIDAR = np.array([
    [ 0.000350, -0.999997,  0.002377,  0.023855],
    [-0.001316, -0.002378, -0.999996,  0.014534],
    [ 0.999999,  0.000347, -0.001317, -0.055265],
    [ 0.000000,  0.000000,  0.000000,  1.000000]
], dtype=np.float64)

T_DEPTH_LIDAR = np.array([
    [ 0.000000, -1.000000,  0.000000,  0.047640],
    [ 0.000000,  0.000000, -1.000000,  0.014311],
    [ 1.000000,  0.000000,  0.000000, -0.055700],
    [ 0.000000,  0.000000,  0.000000,  1.000000]
], dtype=np.float64)

# Optical to LiDAR Coordinate Transformation for Gemini Depth
T_DEPTH_OPTICAL_TO_DEPTH_BODY = np.array([
    [ 0.0,  0.0,  1.0,  0.0],
    [-1.0,  0.0,  0.0,  0.0],
    [ 0.0, -1.0,  0.0,  0.0],
    [ 0.0,  0.0,  0.0,  1.0]
], dtype=np.float64)

# Extrinsics in LiDAR body frame: T_lidar_imu
T_LIDAR_IMU = np.array([
    [ 1.0,  0.0,  0.0, -0.011],
    [ 0.0,  1.0,  0.0, -0.02329],
    [ 0.0,  0.0,  1.0, -0.04412],
    [ 0.0,  0.0,  0.0,  1.0]
], dtype=np.float64)

T_IMU_LIDAR = np.linalg.inv(T_LIDAR_IMU)

# Rigid Extrinsics from Camera Depth Optical Frame to Body (IMU)
T_IMU_DEPTH_OPTICAL = T_IMU_LIDAR @ np.linalg.inv(T_DEPTH_LIDAR) @ T_DEPTH_OPTICAL_TO_DEPTH_BODY
R_IMU_DEPTH_OPT = T_IMU_DEPTH_OPTICAL[:3, :3]
t_IMU_DEPTH_OPT = T_IMU_DEPTH_OPTICAL[:3, 3]

def slerp_quat(q0, q1, alpha):
    """Spherical Linear Interpolation between q0 and q1 [x, y, z, w]."""
    alpha = float(np.clip(alpha, 0.0, 1.0))
    dot = np.dot(q0, q1)
    if dot < 0.0:
        q1 = -q1
        dot = -dot
    if dot > 0.9995:
        q = (1.0 - alpha) * q0 + alpha * q1
        return q / np.linalg.norm(q)
    theta_0 = np.arccos(dot)
    sin_theta_0 = np.sin(theta_0)
    theta = theta_0 * alpha
    sin_theta = np.sin(theta)
    s0 = np.cos(theta) - dot * sin_theta / sin_theta_0
    s1 = sin_theta / sin_theta_0
    return s0 * q0 + s1 * q1


class M128FieldScanNode(Node):
    def __init__(self, session_id="M12_8_Field_Scan"):
        super().__init__('m12_8_field_scan_node')
        self.session_id = session_id
        self.export_dir = f"/home/scanar/scanarMini/data/{session_id}"
        os.makedirs(self.export_dir, exist_ok=True)

        self.bridge = CvBridge()
        self.lock = threading.Lock()

        # State buffers
        self.is_recording = False
        self.start_time = None
        self.stop_time = None

        self.odom_buffer = deque(maxlen=2000)
        self.trajectory_records = []
        self.airy_sweeps = []
        self.gemini_frames = []

        self.latest_rgb_image = None
        self.latest_rgb_stamp = 0.0
        self.depth_k = None

        # Counters
        self.cnt_odom = 0
        self.cnt_airy = 0
        self.cnt_depth = 0
        self.cnt_rgb = 0

        # Subscriptions
        qos_reliable = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=100)
        qos_sensor = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=100)

        self.sub_odom = self.create_subscription(Odometry, '/aft_mapped_to_init', self.odom_callback, qos_reliable)
        self.sub_registered = self.create_subscription(PointCloud2, '/cloud_registered', self.registered_callback, qos_reliable)
        self.sub_rgb = self.create_subscription(Image, '/camera/color/image_raw', self.rgb_callback, qos_sensor)
        self.sub_depth = self.create_subscription(Image, '/camera/depth/image_raw', self.depth_callback, qos_sensor)
        self.sub_depth_info = self.create_subscription(CameraInfo, '/camera/depth/camera_info', self.depth_info_callback, qos_sensor)

        self.get_logger().info(f"[M12.8 Field Scan Node] Initialized. Output target: {self.export_dir}")

    def odom_callback(self, msg: Odometry):
        sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        pos = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z], dtype=np.float64)
        quat = np.array([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w], dtype=np.float64)

        with self.lock:
            self.cnt_odom += 1
            self.odom_buffer.append((sec, pos, quat))
            if self.is_recording:
                rot = Rotation.from_quat(quat)
                euler = rot.as_euler('zyx', degrees=False)
                self.trajectory_records.append((sec, pos[0], pos[1], pos[2], quat[0], quat[1], quat[2], quat[3], euler[0]))

    def registered_callback(self, msg: PointCloud2):
        sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with self.lock:
            self.cnt_airy += 1

        if not self.is_recording:
            return

        if msg.width * msg.height == 0 or msg.point_step < 12:
            return

        # Extract XYZ points from PointCloud2
        raw_data = bytes(msg.data)
        num_points = msg.width * msg.height
        step = msg.point_step
        if len(raw_data) < num_points * step:
            return

        pts = []
        for i in range(0, num_points * step, step):
            x, y, z = struct.unpack_from('<fff', raw_data, i)
            if not (math.isnan(x) or math.isnan(y) or math.isnan(z)):
                d_sq = x*x + y*y + z*z
                if 0.04 < d_sq < 1600.0:
                    pts.append([x, y, z])

        if len(pts) > 0:
            arr = np.array(pts, dtype=np.float64)
            with self.lock:
                self.airy_sweeps.append((sec, arr))

    def rgb_callback(self, msg: Image):
        sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with self.lock:
            self.cnt_rgb += 1
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

    def get_interpolated_pose(self, t_query):
        with self.lock:
            buf = list(self.odom_buffer)
        if len(buf) < 2:
            return None, None
        stamps = np.array([p[0] for p in buf])
        idx = np.searchsorted(stamps, t_query)
        if idx == 0:
            if abs(t_query - stamps[0]) <= 0.1:
                return buf[0][1], buf[0][2]
            return None, None
        if idx >= len(buf):
            if abs(t_query - stamps[-1]) <= 0.1:
                return buf[-1][1], buf[-1][2]
            return None, None

        t0, p0, q0 = buf[idx - 1]
        t1, p1, q1 = buf[idx]
        dt = t1 - t0
        if dt <= 1e-6:
            return p0, q0

        alpha = (t_query - t0) / dt
        p_interp = (1.0 - alpha) * p0 + alpha * p1
        q_interp = slerp_quat(q0, q1, alpha)
        return p_interp, q_interp

    def depth_callback(self, msg: Image):
        sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with self.lock:
            self.cnt_depth += 1

        if not self.is_recording:
            return

        try:
            depth_raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception:
            return

        pos_w, quat_w = self.get_interpolated_pose(sec)
        if pos_w is None or quat_w is None:
            return

        R_w_imu = Rotation.from_quat(quat_w).as_matrix()
        R_w_cam = R_w_imu @ R_IMU_DEPTH_OPT
        t_w_cam = R_w_imu @ t_IMU_DEPTH_OPT + pos_w

        # Subsample depth frame for efficiency
        sub = 4
        h, w = depth_raw.shape
        d_sub = depth_raw[::sub, ::sub].astype(np.float64)
        if depth_raw.dtype == np.uint16:
            d_sub *= 0.001

        mask = (d_sub >= 0.50) & (d_sub <= 4.00)
        v_idx, u_idx = np.nonzero(mask)
        if len(v_idx) == 0:
            return

        u_px = u_idx * sub
        v_px = v_idx * sub
        z_vals = d_sub[v_idx, u_idx]

        fx = self.depth_k[0, 0] if self.depth_k is not None else 609.0
        fy = self.depth_k[1, 1] if self.depth_k is not None else 609.0
        cx = self.depth_k[0, 2] if self.depth_k is not None else 640.0
        cy = self.depth_k[1, 2] if self.depth_k is not None else 400.0

        x_cam = (u_px - cx) * z_vals / fx
        y_cam = (v_px - cy) * z_vals / fy
        pts_cam = np.vstack((x_cam, y_cam, z_vals)).T

        pts_w = pts_cam @ R_w_cam.T + t_w_cam

        # Project RGB color
        with self.lock:
            rgb_img = self.latest_rgb_image
            rgb_stamp = self.latest_rgb_stamp

        if rgb_img is not None and abs(sec - rgb_stamp) <= 0.060:
            # Color assignment
            rgb_h, rgb_w = rgb_img.shape[:2]
            pts_rgb_frame = (pts_cam @ T_RGB_LIDAR[:3, :3].T) + T_RGB_LIDAR[:3, 3]
            zr = pts_rgb_frame[:, 2]
            valid_z = zr > 0.1
            u_r = np.round(pts_rgb_frame[:, 0] * fx / zr + cx).astype(int)
            v_r = np.round(pts_rgb_frame[:, 1] * fy / zr + cy).astype(int)

            colors = np.zeros((len(pts_w), 3), dtype=np.uint8)
            in_fov = valid_z & (u_r >= 0) & (u_r < rgb_w) & (v_r >= 0) & (v_r < rgb_h)
            colors[in_fov] = rgb_img[v_r[in_fov], u_r[in_fov]]
            colors[~in_fov] = [200, 200, 200]
        else:
            colors = np.full((len(pts_w), 3), 180, dtype=np.uint8)

        with self.lock:
            self.gemini_frames.append((sec, pts_w, colors))

    def export_production_bundle(self):
        duration = (self.stop_time - self.start_time) if (self.start_time and self.stop_time) else 0.0
        print(f"\n[EXPORT] Processing M12.8 Field Scan dataset ({duration:.2f}s)...")

        # 1. Trajectory Files
        traj_csv = os.path.join(self.export_dir, "trajectory.csv")
        traj_tum = os.path.join(self.export_dir, "trajectory_tum.txt")

        with open(traj_csv, 'w') as f_csv, open(traj_tum, 'w') as f_tum:
            f_csv.write("timestamp_sec,x_m,y_m,z_m,qx,qy,qz,qw,yaw_rad\n")
            for r in self.trajectory_records:
                sec, x, y, z, qx, qy, qz, qw, yaw = r
                f_csv.write(f"{sec:.6f},{x:.6f},{y:.6f},{z:.6f},{qx:.6f},{qy:.6f},{qz:.6f},{qw:.6f},{yaw:.6f}\n")
                f_tum.write(f"{sec:.6f} {x:.6f} {y:.6f} {z:.6f} {qx:.6f} {qy:.6f} {qz:.6f} {qw:.6f}\n")

        # 2. Voxelize Airy Point Cloud
        all_a = [s[1] for s in self.airy_sweeps if len(s[1]) > 0]
        if len(all_a) > 0:
            a_raw = np.vstack(all_a)
            voxel_size = 0.015
            a_keys = np.floor(a_raw / voxel_size).astype(int)
            _, u_idx = np.unique(a_keys, axis=0, return_index=True)
            airy_clean = a_raw[u_idx]
        else:
            airy_clean = np.empty((0, 3), dtype=np.float64)

        # 3. Voxelize Gemini Point Cloud
        all_g_xyz, all_g_rgb = [], []
        for gf in self.gemini_frames:
            if gf[1] is not None and len(gf[1]) > 0:
                all_g_xyz.append(gf[1])
                all_g_rgb.append(gf[2])

        if len(all_g_xyz) > 0:
            g_raw_xyz = np.vstack(all_g_xyz)
            g_raw_rgb = np.vstack(all_g_rgb)
            g_keys = np.floor(g_raw_xyz / 0.015).astype(int)
            _, gu_idx = np.unique(g_keys, axis=0, return_index=True)
            gemini_clean_xyz = g_raw_xyz[gu_idx]
            gemini_clean_rgb = g_raw_rgb[gu_idx]
        else:
            gemini_clean_xyz = np.empty((0, 3), dtype=np.float64)
            gemini_clean_rgb = np.empty((0, 3), dtype=np.uint8)

        # 4. Three-Class ASPS Fusion
        fused_xyz, fused_rgb = self.run_asps_fusion(airy_clean, gemini_clean_xyz, gemini_clean_rgb)

        # 5. Write PLY and PCD files
        self.write_ply(os.path.join(self.export_dir, "pointcloud_airy.ply"), airy_clean, None)
        self.write_pcd(os.path.join(self.export_dir, "pointcloud_airy.pcd"), airy_clean, None)
        self.write_ply(os.path.join(self.export_dir, "pointcloud_stereo.ply"), gemini_clean_xyz, gemini_clean_rgb)
        self.write_pcd(os.path.join(self.export_dir, "pointcloud_stereo.pcd"), gemini_clean_xyz, gemini_clean_rgb)
        self.write_ply(os.path.join(self.export_dir, "pointcloud_fused_rgb.ply"), fused_xyz, fused_rgb)
        self.write_pcd(os.path.join(self.export_dir, "pointcloud_fused_rgb.pcd"), fused_xyz, fused_rgb)

        with open(os.path.join(self.export_dir, "flash_anchors.json"), 'w') as fa:
            json.dump([], fa, indent=2)

        meta = {
            "session_id": self.session_id,
            "export_timestamp": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            "capture_duration_seconds": float(round(duration, 2)),
            "point_counts": {
                "authoritative_airy_points": int(len(airy_clean)),
                "supplemental_gemini_points": int(len(gemini_clean_xyz)),
                "fused_total_points": int(len(fused_xyz)),
                "density_multiplier": float(round(len(fused_xyz) / max(1, len(airy_clean)), 2))
            },
            "sensor_telemetry": {
                "fastlivo_odometry_received": self.cnt_odom,
                "airy_sweeps_received": self.cnt_airy,
                "gemini_depth_frames_received": self.cnt_depth,
                "gemini_rgb_frames_received": self.cnt_rgb
            }
        }

        with open(os.path.join(self.export_dir, "scan_metadata.json"), 'w') as fm:
            json.dump(meta, fm, indent=2)

        print(f"[EXPORT SUCCESS] Production bundle exported to: {self.export_dir}")
        print(f"  - Authoritative Airy Cloud: {len(airy_clean):,} points")
        print(f"  - Supplemental Gemini Cloud: {len(gemini_clean_xyz):,} points")
        print(f"  - Final Fused Colored Cloud: {len(fused_xyz):,} points")
        return meta

    def run_asps_fusion(self, airy_pts, gemini_pts, gemini_rgb, tau_snap=0.020, tau_conflict=0.080):
        if len(airy_pts) == 0:
            return gemini_pts, gemini_rgb
        if len(gemini_pts) == 0:
            airy_rgb = np.full((len(airy_pts), 3), 150, dtype=np.uint8)
            return airy_pts, airy_rgb

        from scipy.spatial import cKDTree
        tree_a = cKDTree(airy_pts)
        dists, indices = tree_a.query(gemini_pts, k=1, workers=1)

        mask_snap = dists <= tau_snap
        mask_conflict = (dists > tau_snap) & (dists <= tau_conflict)
        mask_supp = dists > tau_conflict

        # Snapped Points (ASPS)
        p_snap = gemini_pts[mask_snap]
        c_snap = gemini_rgb[mask_snap]
        p_supp = gemini_pts[mask_supp]
        c_supp = gemini_rgb[mask_supp]

        # Airy points with baseline color
        c_airy = np.full((len(airy_pts), 3), 160, dtype=np.uint8)

        fused_xyz = np.vstack((airy_pts, p_snap, p_supp))
        fused_rgb = np.vstack((c_airy, c_snap, c_supp))
        return fused_xyz, fused_rgb

    def write_ply(self, filepath, xyz, rgb=None):
        n = len(xyz)
        with open(filepath, 'w') as f:
            f.write("ply\nformat ascii 1.0\n")
            f.write(f"element vertex {n}\n")
            f.write("property float x\nproperty float y\nproperty float z\n")
            if rgb is not None and len(rgb) == n:
                f.write("property uchar red\nproperty uchar green\nproperty uchar blue\n")
            f.write("end_header\n")
            if rgb is not None and len(rgb) == n:
                for p, c in zip(xyz, rgb):
                    f.write(f"{p[0]:.4f} {p[1]:.4f} {p[2]:.4f} {int(c[2])} {int(c[1])} {int(c[0])}\n") # BGR to RGB
            else:
                for p in xyz:
                    f.write(f"{p[0]:.4f} {p[1]:.4f} {p[2]:.4f}\n")

    def write_pcd(self, filepath, xyz, rgb=None):
        n = len(xyz)
        has_color = (rgb is not None and len(rgb) == n)
        with open(filepath, 'w') as f:
            f.write("# .PCD v0.7 - Point Cloud Data\nVERSION 0.7\n")
            if has_color:
                f.write("FIELDS x y z rgb\nSIZE 4 4 4 4\nTYPE F F F U\nCOUNT 1 1 1 1\n")
            else:
                f.write("FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n")
            f.write(f"WIDTH {n}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {n}\nDATA ascii\n")
            if has_color:
                for p, c in zip(xyz, rgb):
                    rgb_uint = (int(c[2]) << 16) | (int(c[1]) << 8) | int(c[0])
                    f.write(f"{p[0]:.4f} {p[1]:.4f} {p[2]:.4f} {rgb_uint}\n")
            else:
                for p in xyz:
                    f.write(f"{p[0]:.4f} {p[1]:.4f} {p[2]:.4f}\n")


def main():
    parser = argparse.ArgumentParser(description="M12.8 Field Scan Runner")
    parser.add_argument("--session-id", default="M12_8_Field_Scan_01", help="Session ID / output directory name")
    parser.add_argument("--duration", type=float, default=20.0, help="Scan duration in seconds (0 for manual stop via Ctrl+C)")
    parser.add_argument("--countdown", type=int, default=5, help="Countdown seconds before recording starts")
    args = parser.parse_args()

    rclpy.init()
    node = M128FieldScanNode(session_id=args.session_id)

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print("[M12.8] Waiting for active topics...")
    t_wait = time.time()
    while (node.cnt_odom < 5 or node.cnt_airy < 5 or node.cnt_depth < 5) and (time.time() - t_wait < 10.0):
        time.sleep(0.1)

    print(f"[M12.8] Topics active: Odom={node.cnt_odom}, Airy={node.cnt_airy}, Depth={node.cnt_depth}, RGB={node.cnt_rgb}")

    if args.countdown > 0:
        print(f"\n==========================================")
        print(f"M12.8 FIELD SCAN COUNTDOWN ({args.countdown}s):")
        for c in range(args.countdown, 0, -1):
            print(f"  >>> {c} ...")
            time.sleep(1.0)
        print(f"  >>> RECORDING ACTIVE — BEGIN SCANNING ROUTE! <<<")
        print(f"==========================================\n")

    node.start_time = time.time()
    node.is_recording = True

    try:
        t0 = time.time()
        while rclpy.ok():
            elapsed = time.time() - t0
            if args.duration > 0 and elapsed >= args.duration:
                break
            print(f"  [SCANNING] {elapsed:.1f}/{args.duration:.1f}s | Airy Sweeps: {len(node.airy_sweeps)} | Depth Frames: {len(node.gemini_frames)} | Odom Poses: {len(node.trajectory_records)}")
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("\n[M12.8] Manual stop triggered by user.")

    node.is_recording = False
    node.stop_time = time.time()
    print("\n>>> [SCAN COMPLETE] Exporting production datasets... <<<")
    node.export_production_bundle()

    rclpy.shutdown()
    spin_thread.join(timeout=2.0)

if __name__ == '__main__':
    main()
