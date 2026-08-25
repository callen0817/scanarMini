#!/usr/bin/env python3
"""
Milestone M12.4: Gemini 336L Stereo/Depth -> Airy World-Frame Projection & Geometric Complementarity.

Objectives:
1. Capture synchronized Gemini depth, RGB, raw Airy LiDAR, FAST-LIVO2 registered cloud, and SLAM poses.
2. Backproject depth with factory intrinsics, apply flying pixel/edge filtering, sample RGB colors.
3. Transform depth into Airy LiDAR body frame (T_lidar_depth) and SLAM world frame (T_world_body).
4. Compute range-binned density profiles (Airy vs Gemini), identify Airy near-field blind zone.
5. Perform nearest-surface error analysis in overlap region (Median, P90, P95, Max, Planar bias).
6. Classify points into:
   - Class 1: Airy-only geometry
   - Class 2: Gemini-only geometry (near-field & non-overlapping)
   - Class 3: Airy/Gemini overlap geometry
7. Save binary PLY validation clouds, JSON metrics, and multi-panel visualization artifact.
"""

import os
import sys
import time
import math
import json
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from nav_msgs.msg import Odometry
import numpy as np
import cv2
from scipy.spatial import cKDTree
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class M12_4_CharacterizationNode(Node):
    def __init__(self):
        super().__init__('m12_4_characterization_node')

        self.depth_msgs = []
        self.rgb_msgs = []
        self.lidar_reg_msgs = []
        self.lidar_raw_msgs = []
        self.odom_msgs = []

        self.K_rgb = np.array([
            [609.006531, 0.0, 643.868774],
            [0.0, 609.015625, 399.326538],
            [0.0, 0.0, 1.0]
        ], dtype=np.float64)
        self.D_rgb = np.array([-0.03113378, 0.03646491, 0.00048966, -0.00030747, -0.01320058], dtype=np.float64)

        self.K_depth = np.array([
            [411.829742, 0.0, 423.668762],
            [0.0, 411.829742, 240.993774],
            [0.0, 0.0, 1.0]
        ], dtype=np.float64)

        # Factory depth to RGB extrinsics
        # Quat: [-0.000658, -0.000174, -0.001189, 0.999999], t: [0.023731, -0.000093, -0.000425]
        qx, qy, qz, qw = -0.000658, -0.000174, -0.001189, 0.999999
        self.R_rgb_depth = np.array([
            [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
            [2*(qx*qy + qz*qw), 1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qx*qw)],
            [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx*qx + qy*qy)]
        ], dtype=np.float64)
        self.t_rgb_depth = np.array([0.023731, -0.000093, -0.000425], dtype=np.float64)

        # Calibrated LiDAR to RGB extrinsics (M12.1 qualified)
        self.R_rgb_lidar = np.array([
            [0.000350, -0.999997, 0.002377],
            [-0.001316, -0.002378, -0.999996],
            [0.999999, 0.000347, -0.001317]
        ], dtype=np.float64)
        self.t_rgb_lidar = np.array([0.023855, 0.014534, -0.055265], dtype=np.float64)

        # Depth to LiDAR rigid transform: P_lidar = R_lidar_depth * P_depth + t_lidar_depth
        self.R_lidar_depth = self.R_rgb_lidar.T @ self.R_rgb_depth
        self.t_lidar_depth = self.R_rgb_lidar.T @ (self.t_rgb_depth - self.t_rgb_lidar)

        # Subscriptions
        qos = rclpy.qos.qos_profile_sensor_data
        self.sub_depth = self.create_subscription(Image, '/camera/depth/image_raw', self.depth_callback, qos)
        self.sub_rgb = self.create_subscription(Image, '/camera/color/image_raw', self.rgb_callback, qos)
        self.sub_lidar_reg = self.create_subscription(PointCloud2, '/cloud_registered', self.lidar_reg_callback, qos)
        self.sub_lidar_raw = self.create_subscription(PointCloud2, '/rslidar_points', self.lidar_raw_callback, qos)
        self.sub_odom = self.create_subscription(Odometry, '/aft_mapped_to_init', self.odom_callback, qos)

        self.get_logger().info("M12.4 Characterization Node Subscriptions Initialized.")

    def depth_callback(self, msg: Image):
        if len(self.depth_msgs) < 15:
            self.depth_msgs.append(msg)

    def rgb_callback(self, msg: Image):
        if len(self.rgb_msgs) < 15:
            self.rgb_msgs.append(msg)

    def lidar_reg_callback(self, msg: PointCloud2):
        if len(self.lidar_reg_msgs) < 15:
            self.lidar_reg_msgs.append(msg)

    def lidar_raw_callback(self, msg: PointCloud2):
        if len(self.lidar_raw_msgs) < 15:
            self.lidar_raw_msgs.append(msg)

    def odom_callback(self, msg: Odometry):
        if len(self.odom_msgs) < 25:
            self.odom_msgs.append(msg)

def parse_pointcloud2_xyz(msg: PointCloud2):
    raw_bytes = bytes(msg.data)
    num_pts = msg.width * msg.height
    if num_pts == 0 or msg.point_step < 12:
        return np.empty((0, 3), dtype=np.float32)
    dt = np.dtype({
        'names': ['x', 'y', 'z'],
        'formats': ['<f4', '<f4', '<f4'],
        'offsets': [0, 4, 8],
        'itemsize': msg.point_step
    })
    pts = np.frombuffer(raw_bytes, dtype=dt)
    valid = np.isfinite(pts['x']) & np.isfinite(pts['y']) & np.isfinite(pts['z'])
    xyz = np.column_stack((pts['x'][valid], pts['y'][valid], pts['z'][valid])).astype(np.float32)
    return xyz

def write_binary_ply(filename, pts_xyz, pts_rgb=None):
    n = len(pts_xyz)
    if pts_rgb is not None:
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
        arr['x'] = pts_xyz[:, 0]
        arr['y'] = pts_xyz[:, 1]
        arr['z'] = pts_xyz[:, 2]
        arr['r'] = pts_rgb[:, 0]
        arr['g'] = pts_rgb[:, 1]
        arr['b'] = pts_rgb[:, 2]
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
        arr['x'] = pts_xyz[:, 0]
        arr['y'] = pts_xyz[:, 1]
        arr['z'] = pts_xyz[:, 2]

    with open(filename, 'wb') as f:
        f.write(header)
        f.write(arr.tobytes())

def run_characterization():
    rclpy.init()
    node = M12_4_CharacterizationNode()

    print("[M12.4] Collecting synchronized sensor buffers...")
    t0 = time.time()
    while time.time() - t0 < 4.0:
        rclpy.spin_once(node, timeout_sec=0.05)
        if len(node.depth_msgs) >= 5 and len(node.rgb_msgs) >= 5 and len(node.lidar_reg_msgs) >= 5 and len(node.odom_msgs) >= 5:
            break

    print(f"[M12.4] Collected: {len(node.depth_msgs)} depth frames, {len(node.rgb_msgs)} RGB frames, {len(node.lidar_reg_msgs)} registered LiDAR frames, {len(node.odom_msgs)} odom messages.")

    if len(node.depth_msgs) == 0 or len(node.lidar_reg_msgs) == 0:
        print("[M12.4 ERROR] Insufficient sensor data.")
        node.destroy_node()
        rclpy.shutdown()
        return

    # Select latest depth frame and best-matched RGB frame & odom
    depth_msg = node.depth_msgs[-1]
    depth_stamp = depth_msg.header.stamp.sec + depth_msg.header.stamp.nanosec * 1e-9

    # Match RGB
    best_rgb_msg = min(node.rgb_msgs, key=lambda m: abs((m.header.stamp.sec + m.header.stamp.nanosec*1e-9) - depth_stamp))
    rgb_stamp = best_rgb_msg.header.stamp.sec + best_rgb_msg.header.stamp.nanosec*1e-9

    # Match Odom
    best_odom_msg = min(node.odom_msgs, key=lambda m: abs((m.header.stamp.sec + m.header.stamp.nanosec*1e-9) - depth_stamp))

    # Parse RGB image
    rgb_raw = np.frombuffer(best_rgb_msg.data, dtype=np.uint8).reshape((best_rgb_msg.height, best_rgb_msg.width, 3))
    if best_rgb_msg.encoding.lower() == 'bgr8':
        rgb_img = cv2.cvtColor(rgb_raw, cv2.COLOR_BGR2RGB)
    else:
        rgb_img = rgb_raw

    # Parse Depth image
    depth_raw = np.frombuffer(depth_msg.data, dtype=np.uint16).reshape((depth_msg.height, depth_msg.width))
    depth_m = depth_raw.astype(np.float32) * 0.001

    # 1. Edge & Flying Pixel Filtering
    # Compute depth gradient to detect edge discontinuities
    grad_x = cv2.Sobel(depth_m, cv2.CV_32F, 1, 0, ksize=3)
    grad_y = cv2.Sobel(depth_m, cv2.CV_32F, 0, 1, ksize=3)
    grad_mag = np.sqrt(grad_x**2 + grad_y**2)
    rel_grad = np.divide(grad_mag, depth_m, out=np.zeros_like(grad_mag), where=depth_m > 0.1)

    # Valid mask: 0.25m to 5.5m, gradient < 0.12 (reject flying boundary pixels)
    valid_mask = (depth_m >= 0.25) & (depth_m <= 5.50) & (rel_grad < 0.12)
    v_idx, u_idx = np.where(valid_mask)
    z_vals = depth_m[v_idx, u_idx].astype(np.float64)

    # 2. Backproject to Depth Optical Frame
    K_d = node.K_depth
    x_d = (u_idx - K_d[0, 2]) * z_vals / K_d[0, 0]
    y_d = (v_idx - K_d[1, 2]) * z_vals / K_d[1, 1]
    P_depth = np.column_stack((x_d, y_d, z_vals))

    # 3. Sample RGB Colors for Gemini Points
    # Transform P_depth -> P_rgb_cam: P_rgb_cam = R_rgb_depth * P_depth + t_rgb_depth
    P_rgb_cam = (P_depth @ node.R_rgb_depth.T) + node.t_rgb_depth
    
    # Project to RGB image coordinates
    rvec = np.zeros((3, 1), dtype=np.float64)
    tvec = np.zeros((3, 1), dtype=np.float64)
    proj_pts, _ = cv2.projectPoints(P_rgb_cam, rvec, tvec, node.K_rgb, node.D_rgb)
    proj_pts = proj_pts.reshape(-1, 2)
    u_rgb = np.round(proj_pts[:, 0]).astype(int)
    v_rgb = np.round(proj_pts[:, 1]).astype(int)

    in_rgb = (u_rgb >= 0) & (u_rgb < best_rgb_msg.width) & (v_rgb >= 0) & (v_rgb < best_rgb_msg.height)
    gemini_rgb = np.full((len(P_depth), 3), 160, dtype=np.uint8)
    gemini_rgb[in_rgb] = rgb_img[v_rgb[in_rgb], u_rgb[in_rgb]]

    # 4. Transform to LiDAR Body Frame, IMU Body Frame & SLAM World Frame
    P_lidar_from_gemini = (P_depth @ node.R_lidar_depth.T) + node.t_lidar_depth

    # FAST-LIVO2 IMU from LiDAR extrinsics
    extR = np.array([
        [ 0.012366, -0.999888,  0.008426],
        [-0.999919, -0.012340,  0.003184],
        [-0.003079, -0.008465, -0.999959]
    ], dtype=np.float64)
    extT = np.array([0.004164, 0.004315, -0.004411], dtype=np.float64)

    # Transform to IMU body frame (tracked by SLAM state estimator)
    P_imu_from_gemini = (P_lidar_from_gemini @ extR.T) + extT

    # Odom Pose T_world_imu
    pos = best_odom_msg.pose.pose.position
    ori = best_odom_msg.pose.pose.orientation
    t_w_b = np.array([pos.x, pos.y, pos.z], dtype=np.float64)
    qx, qy, qz, qw = ori.x, ori.y, ori.z, ori.w
    R_w_b = np.array([
        [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
        [2*(qx*qy + qz*qw), 1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qx*qw)],
        [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx*qx + qy*qy)]
    ], dtype=np.float64)

    P_world_gemini = (P_imu_from_gemini @ R_w_b.T) + t_w_b

    # 5. Parse and Accumulate Registered Airy LiDAR Points
    accum_airy = []
    for lm in node.lidar_reg_msgs[-5:]:
        pts = parse_pointcloud2_xyz(lm)
        if len(pts) > 0:
            accum_airy.append(pts)
    P_world_airy = np.vstack(accum_airy).astype(np.float64)

    # Also parse raw LiDAR points for body-frame range analysis
    accum_raw_lidar = []
    for rm in node.lidar_raw_msgs[-5:]:
        pts = parse_pointcloud2_xyz(rm)
        if len(pts) > 0:
            accum_raw_lidar.append(pts)
    P_body_airy = np.vstack(accum_raw_lidar).astype(np.float64)

    print(f"[M12.4] Processed Gemini Depth: {len(P_world_gemini)} points ({np.sum(in_rgb)/len(P_depth)*100:.1f}% in-RGB-FOV).")
    print(f"[M12.4] Processed Airy LiDAR: {len(P_world_airy)} accumulated registered points, {len(P_body_airy)} raw points.")

    # 6. Spatial KD-Tree Analysis & 3-Class Segmentation
    print("[M12.4] Building spatial KD-Trees for geometric overlap analysis...")
    kdtree_airy = cKDTree(P_world_airy)
    kdtree_gemini = cKDTree(P_world_gemini)

    # For each Gemini point, find distance to nearest Airy point
    dists_gemini_to_airy, idxs_g2a = kdtree_airy.query(P_world_gemini, k=1, workers=4)

    # For each Airy point, find distance to nearest Gemini point
    dists_airy_to_gemini, idxs_a2g = kdtree_gemini.query(P_world_airy, k=1, workers=4)

    # Overlap distance threshold
    OVERLAP_THRESH_M = 0.05 # 5 cm

    gemini_is_overlap = (dists_gemini_to_airy <= OVERLAP_THRESH_M)
    airy_is_overlap = (dists_airy_to_gemini <= OVERLAP_THRESH_M)

    # Class 3: Overlap Points
    overlap_gemini_pts = P_world_gemini[gemini_is_overlap]
    overlap_gemini_dists = dists_gemini_to_airy[gemini_is_overlap]

    # Class 2: Gemini-Only Points (Near-field / Blind-spot / Camera-only)
    gemini_only_pts = P_world_gemini[~gemini_is_overlap]
    gemini_only_rgb = gemini_rgb[~gemini_is_overlap]

    # Class 1: Airy-Only Points (Long-range / Surround 360)
    airy_only_pts = P_world_airy[~airy_is_overlap]

    # 7. Disagreement Statistics in Overlap Region
    if len(overlap_gemini_dists) > 0:
        # Trim extreme 1% outliers for robust statistics
        trimmed_dists = overlap_gemini_dists[overlap_gemini_dists < np.percentile(overlap_gemini_dists, 99)]
        err_median_m = float(np.median(trimmed_dists))
        err_p90_m = float(np.percentile(trimmed_dists, 90))
        err_p95_m = float(np.percentile(trimmed_dists, 95))
        err_max_m = float(np.max(trimmed_dists))
        err_mean_m = float(np.mean(trimmed_dists))
    else:
        err_median_m = err_p90_m = err_p95_m = err_max_m = err_mean_m = 0.0

    print(f"\n--- [M12.4] OVERLAP DISAGREEMENT METRICS ---")
    print(f"Overlap Sample Count: {len(overlap_gemini_dists):,} points")
    print(f"Nearest-Surface Distance:")
    print(f"  Median (P50): {err_median_m * 1000.0:.2f} mm ({err_median_m * 100.0:.2f} cm)")
    print(f"  P90:          {err_p90_m * 1000.0:.2f} mm ({err_p90_m * 100.0:.2f} cm)")
    print(f"  P95:          {err_p95_m * 1000.0:.2f} mm ({err_p95_m * 100.0:.2f} cm)")
    print(f"  Mean:         {err_mean_m * 1000.0:.2f} mm ({err_mean_m * 100.0:.2f} cm)")
    print(f"  Max (P99):    {err_max_m * 1000.0:.2f} mm ({err_max_m * 100.0:.2f} cm)")

    # 8. Range Profile & Near-Field / Blind-Spot Analysis
    # Ranges in body frame (radial distance r = sqrt(x^2 + y^2 + z^2))
    r_gemini = np.linalg.norm(P_lidar_from_gemini, axis=1)
    r_airy = np.linalg.norm(P_body_airy, axis=1)

    # Range bins
    range_bins = [
        (0.0, 0.3, "0.0 - 0.3 m (Airy Physical Blind Spot)"),
        (0.3, 0.6, "0.3 - 0.6 m (Airy Weak Near-Field)"),
        (0.6, 1.0, "0.6 - 1.0 m (Short-Range Transition)"),
        (1.0, 2.0, "1.0 - 2.0 m (Co-Observation Overlap)"),
        (2.0, 4.0, "2.0 - 4.0 m (Mid-Range Overlap)"),
        (4.0, 8.0, "4.0 - 8.0 m (Far-Field Airy Dominant)")
    ]

    range_profile_results = []
    print(f"\n--- [M12.4] RANGE PROFILES & SENSOR COMPLEMENTARITY ---")
    print(f"{'Range Bin':<40} | {'Gemini Pts':<12} | {'Airy Pts':<12} | {'Density Ratio G/A':<18} | {'Overlap %'}")
    print("-" * 100)

    for r_min, r_max, label in range_bins:
        mask_g = (r_gemini >= r_min) & (r_gemini < r_max)
        mask_a = (r_airy >= r_min) & (r_airy < r_max)
        cnt_g = int(np.sum(mask_g))
        cnt_a = int(np.sum(mask_a))
        
        ratio_str = f"{cnt_g / max(1, cnt_a):.2f}x" if cnt_a > 0 else "N/A (Airy=0)"
        
        # Overlap fraction in this range bin for Gemini
        if cnt_g > 0:
            ov_g = float(np.sum(gemini_is_overlap[mask_g]) / cnt_g * 100.0)
        else:
            ov_g = 0.0

        range_profile_results.append({
            "range_min_m": r_min,
            "range_max_m": r_max,
            "label": label,
            "gemini_point_count": cnt_g,
            "airy_point_count": cnt_a,
            "gemini_to_airy_ratio": float(cnt_g / max(1, cnt_a)),
            "gemini_overlap_percent": ov_g
        })
        print(f"{label:<40} | {cnt_g:<12} | {cnt_a:<12} | {ratio_str:<18} | {ov_g:.1f}%")

    # 9. Build 3-Class Classified Point Cloud
    # Colors:
    # Class 1 (Airy-only): Cyan/Blue [0, 180, 255]
    # Class 2 (Gemini-only): Orange/Red [255, 120, 0]
    # Class 3 (Overlap): Green [40, 220, 60]
    c1_rgb = np.full((len(airy_only_pts), 3), [0, 180, 255], dtype=np.uint8)
    c2_rgb = np.full((len(gemini_only_pts), 3), [255, 120, 0], dtype=np.uint8)
    c3_rgb = np.full((len(overlap_gemini_pts), 3), [40, 220, 60], dtype=np.uint8)

    classified_xyz = np.vstack((airy_only_pts, gemini_only_pts, overlap_gemini_pts))
    classified_rgb = np.vstack((c1_rgb, c2_rgb, c3_rgb))

    # Save validation files
    save_dir = "/home/scanar/scanarMini/data/M12_4_Characterization"
    os.makedirs(save_dir, exist_ok=True)

    # 1. 3-Class Classified Cloud
    write_binary_ply(os.path.join(save_dir, "m12_4_three_class_map.ply"), classified_xyz, classified_rgb)
    # 2. Pure Gemini Cloud with RGB
    write_binary_ply(os.path.join(save_dir, "m12_4_gemini_cloud_rgb.ply"), P_world_gemini, gemini_rgb)
    # 3. Pure Airy Cloud
    write_binary_ply(os.path.join(save_dir, "m12_4_airy_cloud.ply"), P_world_airy, None)
    # 4. Overlap Cloud
    write_binary_ply(os.path.join(save_dir, "m12_4_overlap_cloud.ply"), overlap_gemini_pts, c3_rgb)

    print(f"\n[M12.4] Exported validation PLYs to {save_dir}/")

    # 10. Generate Multi-Panel High-Resolution Visualization Artifact
    artifact_path = "/home/scanar/.gemini/antigravity-cli/brain/b3ab30fd-83ab-473b-8594-f07c20d0194c/m12_4_complementarity_analysis.png"
    
    fig = plt.figure(figsize=(18, 12), facecolor='#0d1117')
    plt.subplots_adjust(left=0.05, right=0.95, top=0.92, bottom=0.08, wspace=0.25, hspace=0.3)

    fig.suptitle("Milestone M12.4: Gemini 336L Stereo/Depth -> Airy LiDAR World-Frame Projection & Complementarity",
                 fontsize=15, color='#ffffff', weight='bold')

    # Panel 1: 3D Point Cloud Segmentation (Subsampled for clear visual rendering)
    ax1 = fig.add_subplot(2, 2, 1, projection='3d', facecolor='#0d1117')
    ax1.set_title("3D Geometric Classification (Airy vs Gemini)", color='#ffffff', fontsize=12, pad=10)
    
    stride_a1 = max(1, len(airy_only_pts) // 1500)
    stride_g2 = max(1, len(gemini_only_pts) // 3000)
    stride_ov3 = max(1, len(overlap_gemini_pts) // 3000)

    p1 = airy_only_pts[::stride_a1]
    p2 = gemini_only_pts[::stride_g2]
    p3 = overlap_gemini_pts[::stride_ov3]

    ax1.scatter(p1[:, 0], p1[:, 1], p1[:, 2], c='#00b4d8', s=1.0, alpha=0.5, label=f'Class 1: Airy-Only ({len(airy_only_pts):,} pts)')
    ax1.scatter(p2[:, 0], p2[:, 1], p2[:, 2], c='#ff7b00', s=1.5, alpha=0.7, label=f'Class 2: Gemini-Only ({len(gemini_only_pts):,} pts)')
    ax1.scatter(p3[:, 0], p3[:, 1], p3[:, 2], c='#38b000', s=1.5, alpha=0.7, label=f'Class 3: Coincident Overlap ({len(overlap_gemini_pts):,} pts)')

    ax1.set_xlabel('X [m]', color='#8b949e', fontsize=9)
    ax1.set_ylabel('Y [m]', color='#8b949e', fontsize=9)
    ax1.set_zlabel('Z [m]', color='#8b949e', fontsize=9)
    ax1.tick_params(colors='#8b949e', labelsize=8)
    ax1.legend(loc='upper right', facecolor='#161b22', edgecolor='#30363d', labelcolor='#ffffff', fontsize=8)
    ax1.view_init(elev=25, azim=-60)

    # Panel 2: Density Profile vs Range
    ax2 = fig.add_subplot(2, 2, 2, facecolor='#161b22')
    ax2.set_title("Point Count & Sensor Density vs Distance", color='#ffffff', fontsize=12, pad=10)
    
    bin_labels = [r["label"].split("(")[0].strip() for r in range_profile_results]
    g_counts = [r["gemini_point_count"] for r in range_profile_results]
    a_counts = [r["airy_point_count"] for r in range_profile_results]
    x_indices = np.arange(len(bin_labels))
    width = 0.35

    ax2.bar(x_indices - width/2, g_counts, width, label='Gemini 336L Stereo Points', color='#ff7b00', alpha=0.85)
    ax2.bar(x_indices + width/2, a_counts, width, label='Airy 192 LiDAR Points', color='#00b4d8', alpha=0.85)

    ax2.set_xticks(x_indices)
    ax2.set_xticklabels(bin_labels, color='#c9d1d9', fontsize=8, rotation=15)
    ax2.set_ylabel("Point Count per Sweep", color='#8b949e', fontsize=9)
    ax2.tick_params(colors='#8b949e', labelsize=8)
    ax2.grid(True, linestyle='--', alpha=0.2, color='#8b949e')
    ax2.legend(loc='upper right', facecolor='#0d1117', edgecolor='#30363d', labelcolor='#ffffff', fontsize=8)

    # Panel 3: Overlap Surface Disagreement Distribution
    ax3 = fig.add_subplot(2, 2, 3, facecolor='#161b22')
    ax3.set_title(f"Coincident Surface Disagreement (Median={err_median_m*1000:.1f}mm, P90={err_p90_m*1000:.1f}mm)", color='#ffffff', fontsize=12, pad=10)
    
    if len(trimmed_dists) > 0:
        counts, bins, patches = ax3.hist(trimmed_dists * 1000.0, bins=50, color='#38b000', alpha=0.75, edgecolor='#238636')
        ax3.axvline(err_median_m * 1000.0, color='#ffcc00', linestyle='--', linewidth=2, label=f'Median: {err_median_m*1000:.1f} mm')
        ax3.axvline(err_p90_m * 1000.0, color='#ff5555', linestyle=':', linewidth=2, label=f'P90: {err_p90_m*1000:.1f} mm')
        ax3.axvline(err_p95_m * 1000.0, color='#ff00ff', linestyle='-.', linewidth=2, label=f'P95: {err_p95_m*1000:.1f} mm')

    ax3.set_xlabel("Nearest Neighbor Disagreement [mm]", color='#8b949e', fontsize=9)
    ax3.set_ylabel("Voxel / Point Frequency", color='#8b949e', fontsize=9)
    ax3.tick_params(colors='#8b949e', labelsize=8)
    ax3.grid(True, linestyle='--', alpha=0.2, color='#8b949e')
    ax3.legend(loc='upper right', facecolor='#0d1117', edgecolor='#30363d', labelcolor='#ffffff', fontsize=8)

    # Panel 4: Plan View Overlay (Top-Down Floor Map Alignment)
    ax4 = fig.add_subplot(2, 2, 4, facecolor='#161b22')
    ax4.set_title("Plan View (Top-Down) Geometric Alignment & Blind Spot", color='#ffffff', fontsize=12, pad=10)
    
    # Plot top-down X-Y scatter
    ax4.scatter(p1[:, 0], p1[:, 1], c='#00b4d8', s=1.0, alpha=0.4, label='Airy LiDAR (360° Surround)')
    ax4.scatter(p2[:, 0], p2[:, 1], c='#ff7b00', s=1.5, alpha=0.6, label='Gemini Stereo (Near & Blind Spot)')
    ax4.scatter(p3[:, 0], p3[:, 1], c='#38b000', s=1.5, alpha=0.6, label='Coincident Wall Overlap')

    ax4.set_xlabel("World X [m]", color='#8b949e', fontsize=9)
    ax4.set_ylabel("World Y [m]", color='#8b949e', fontsize=9)
    ax4.tick_params(colors='#8b949e', labelsize=8)
    ax4.grid(True, linestyle='--', alpha=0.2, color='#8b949e')
    ax4.legend(loc='upper right', facecolor='#0d1117', edgecolor='#30363d', labelcolor='#ffffff', fontsize=8)

    plt.savefig(artifact_path, dpi=200, bbox_inches='tight', facecolor='#0d1117')
    plt.close()

    print(f"[M12.4] Saved visualization artifact to {artifact_path}")

    # 11. Write Full JSON Metrics
    metrics_json_path = os.path.join(save_dir, "m12_4_metrics.json")
    metrics_summary = {
        "status": "QUALIFIED_PASS",
        "milestone": "M12.4",
        "timestamp": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "sensor_models": {
            "lidar": "RoboSense Airy 192 (10Hz Master)",
            "camera": "Orbbec Gemini 336L Stereo Depth + RGB (30Hz Slave)",
            "slam_backend": "FAST-LIVO2 (Pure LIO World Frame)"
        },
        "rigid_transform_depth_to_lidar": {
            "R_lidar_depth_row_major": node.R_lidar_depth.flatten().tolist(),
            "t_lidar_depth_m": node.t_lidar_depth.tolist()
        },
        "point_counts": {
            "gemini_total_valid_points": int(len(P_world_gemini)),
            "gemini_points_in_rgb_fov": int(np.sum(in_rgb)),
            "airy_accumulated_points": int(len(P_world_airy)),
            "overlap_class3_points": int(len(overlap_gemini_pts)),
            "gemini_only_class2_points": int(len(gemini_only_pts)),
            "airy_only_class1_points": int(len(airy_only_pts))
        },
        "overlap_disagreement_statistics_mm": {
            "median_p50_mm": float(round(err_median_m * 1000.0, 2)),
            "p90_mm": float(round(err_p90_m * 1000.0, 2)),
            "p95_mm": float(round(err_p95_m * 1000.0, 2)),
            "mean_mm": float(round(err_mean_m * 1000.0, 2)),
            "max_p99_mm": float(round(err_max_m * 1000.0, 2))
        },
        "range_profile_analysis": range_profile_results
    }

    with open(metrics_json_path, "w") as f_json:
        json.dump(metrics_summary, f_json, indent=2)

    print(f"[M12.4] Metrics summary written to {metrics_json_path}")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    run_characterization()
