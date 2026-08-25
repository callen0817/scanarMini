#!/usr/bin/env python3
"""
Milestone M12.3: Production Colorized Point Cloud Export & Verification
- Records a live scanning session from /cloud_registered_rgb and /aft_mapped_to_init
- Accumulates a world-frame 3D map with quality-aware voxel color aggregation (3cm voxel resolution)
- Exports:
    1. pointcloud_map_rgb.ply (Binary PLY, XYZ + RGB)
    2. pointcloud_map_rgb.pcd (Binary PCD, XYZ + RGB)
    3. pointcloud_map.ply (Uncolored geometry PLY)
    4. pointcloud_map.pcd (Uncolored geometry PCD)
    5. trajectory.csv
    6. trajectory_tum.txt
    7. flash_anchors.json
    8. scan_metadata.json
- Verifies exported files independently and renders 3D evaluation views.
"""

import os
import sys
import time
import math
import json
import struct
import numpy as np
import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, PointField, Image
from nav_msgs.msg import Odometry

class M123ExportRunner(Node):
    def __init__(self, duration_sec=15.0, dataset_name="M12_3_Production_Capture"):
        super().__init__('m12_3_export_runner')
        self.duration_sec = duration_sec
        self.dataset_name = dataset_name
        self.export_dir = f"/home/scanar/scanarMini/data/{dataset_name}"
        os.makedirs(self.export_dir, exist_ok=True)

        self.voxel_size = 0.03 # 3 cm resolution
        self.inv_voxel = 1.0 / self.voxel_size

        # Voxel Map: key (gx, gy, gz) -> [sum_x, sum_y, sum_z, count, sum_r, sum_g, sum_b, rgb_count]
        self.voxel_map = {}
        self.trajectory = [] # [(stamp, x, y, z, qx, qy, qz, qw, yaw)]
        self.flash_anchors = []

        self.total_raw_points_received = 0
        self.total_colored_points_received = 0
        self.sweep_count = 0
        self.sync_deltas = []

        self.start_time = None
        self.is_recording = False

        qos_sub = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.sub_cloud = self.create_subscription(
            PointCloud2,
            '/cloud_registered_rgb',
            self.cloud_callback,
            qos_sub
        )

        self.sub_odom = self.create_subscription(
            Odometry,
            '/aft_mapped_to_init',
            self.odom_callback,
            qos_sub
        )

        self.get_logger().info(f"M12.3 Export Runner initialized. Target session duration: {self.duration_sec}s")

    def odom_callback(self, msg: Odometry):
        if not self.is_recording:
            return
        sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        pz = msg.pose.pose.position.z
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self.trajectory.append((sec, px, py, pz, qx, qy, qz, qw, yaw))

    def cloud_callback(self, msg: PointCloud2):
        if not self.is_recording:
            return

        num_points = msg.width * msg.height
        if num_points == 0 or msg.point_step < 16:
            return

        self.sweep_count += 1
        raw_bytes = bytes(msg.data)
        dt = np.dtype({
            'names': ['x', 'y', 'z', 'rgb'],
            'formats': ['<f4', '<f4', '<f4', '<f4'],
            'offsets': [0, 4, 8, 12],
            'itemsize': msg.point_step
        })
        pts = np.frombuffer(raw_bytes, dtype=dt)
        valid = np.isfinite(pts['x']) & np.isfinite(pts['y']) & np.isfinite(pts['z'])
        pts_valid = pts[valid]
        if len(pts_valid) == 0:
            return

        xyz = np.column_stack((pts_valid['x'], pts_valid['y'], pts_valid['z'])).astype(np.float64)
        
        # Unpack RGB
        rgb_int = pts_valid['rgb'].view(np.uint32)
        r = ((rgb_int >> 16) & 0xFF).astype(np.float64)
        g = ((rgb_int >> 8) & 0xFF).astype(np.float64)
        b = (rgb_int & 0xFF).astype(np.float64)
        
        # Identify points with valid in-FOV RGB vs neutral (160, 160, 160)
        has_rgb = ~((r == 160.0) & (g == 160.0) & (b == 160.0))

        self.total_raw_points_received += len(xyz)
        self.total_colored_points_received += np.sum(has_rgb)

        # Accumulate into spatial voxel grid
        gx = np.floor(xyz[:, 0] * self.inv_voxel).astype(np.int32)
        gy = np.floor(xyz[:, 1] * self.inv_voxel).astype(np.int32)
        gz = np.floor(xyz[:, 2] * self.inv_voxel).astype(np.int32)

        for i in range(len(xyz)):
            key = (int(gx[i]), int(gy[i]), int(gz[i]))
            x, y, z = xyz[i]
            ri, gi, bi = r[i], g[i], b[i]
            is_valid_rgb = has_rgb[i]

            if key not in self.voxel_map:
                if is_valid_rgb:
                    self.voxel_map[key] = [x, y, z, 1, ri, gi, bi, 1]
                else:
                    self.voxel_map[key] = [x, y, z, 1, 160.0, 160.0, 160.0, 0]
            else:
                rec = self.voxel_map[key]
                rec[0] += x
                rec[1] += y
                rec[2] += z
                rec[3] += 1
                if is_valid_rgb:
                    rec[4] += ri
                    rec[5] += gi
                    rec[6] += bi
                    rec[7] += 1

        if self.sweep_count % 10 == 1:
            self.get_logger().info(
                f"[RECORDING] Sweep #{self.sweep_count} | Accumulated Voxels: {len(self.voxel_map)} | "
                f"Total Observations: {self.total_raw_points_received}"
            )

    def extract_accumulated_map(self):
        """Extract averaged points and quality-weighted colors."""
        n_voxels = len(self.voxel_map)
        if n_voxels == 0:
            return np.empty((0, 3), dtype=np.float32), np.empty((0, 3), dtype=np.uint8), 0

        xyz = np.empty((n_voxels, 3), dtype=np.float32)
        rgb = np.empty((n_voxels, 3), dtype=np.uint8)
        colored_count = 0

        for idx, (k, v) in enumerate(self.voxel_map.items()):
            cnt = v[3]
            xyz[idx] = [v[0] / cnt, v[1] / cnt, v[2] / cnt]
            
            rgb_cnt = v[7]
            if rgb_cnt > 0:
                colored_count += 1
                cr = np.clip(v[4] / rgb_cnt, 0.0, 255.0)
                cg = np.clip(v[5] / rgb_cnt, 0.0, 255.0)
                cb = np.clip(v[6] / rgb_cnt, 0.0, 255.0)
                rgb[idx] = [int(round(cr)), int(round(cg)), int(round(cb))]
            else:
                rgb[idx] = [160, 160, 160]

        return xyz, rgb, colored_count

    def export_all(self, duration):
        xyz, rgb, colored_voxels = self.extract_accumulated_map()
        n_points = len(xyz)
        coverage_pct = (colored_voxels / n_points * 100.0) if n_points > 0 else 0.0

        self.get_logger().info("==================================================")
        self.get_logger().info("M12.3 Exporting Production Dataset")
        self.get_logger().info(f"Target Directory: {self.export_dir}")
        self.get_logger().info(f"Total Accumulated Voxels: {n_points}")
        self.get_logger().info(f"Voxels with Real RGB Coverage: {colored_voxels} ({coverage_pct:.2f}%)")
        self.get_logger().info("==================================================")

        # 1. Binary PLY XYZ + RGB
        ply_rgb_file = os.path.join(self.export_dir, "pointcloud_map_rgb.ply")
        self.write_ply(ply_rgb_file, xyz, rgb)

        # 2. Binary PCD XYZ + RGB
        pcd_rgb_file = os.path.join(self.export_dir, "pointcloud_map_rgb.pcd")
        self.write_pcd(pcd_rgb_file, xyz, rgb)

        # 3. Uncolored Geometry PLY
        ply_geo_file = os.path.join(self.export_dir, "pointcloud_map.ply")
        self.write_ply(ply_geo_file, xyz, None)

        # 4. Uncolored Geometry PCD
        pcd_geo_file = os.path.join(self.export_dir, "pointcloud_map.pcd")
        self.write_pcd(pcd_geo_file, xyz, None)

        # 5. Trajectory CSV & TUM
        traj_csv = os.path.join(self.export_dir, "trajectory.csv")
        traj_tum = os.path.join(self.export_dir, "trajectory_tum.txt")
        with open(traj_csv, 'w') as f_csv, open(traj_tum, 'w') as f_tum:
            f_csv.write("index,timestamp,x_m,y_m,z_m,qx,qy,qz,qw,yaw_deg\n")
            for idx, (t, tx, ty, tz, qx, qy, qz, qw, tyaw) in enumerate(self.trajectory):
                f_csv.write(f"{idx},{t:.6f},{tx:.4f},{ty:.4f},{tz:.4f},{qx:.6f},{qy:.6f},{qz:.6f},{qw:.6f},{math.degrees(tyaw):.2f}\n")
                f_tum.write(f"{t:.6f} {tx:.4f} {ty:.4f} {tz:.4f} {qx:.6f} {qy:.6f} {qz:.6f} {qw:.6f}\n")

        # 6. Flash Anchors JSON
        anchors_json = os.path.join(self.export_dir, "flash_anchors.json")
        with open(anchors_json, 'w') as f_anchors:
            json.dump(self.flash_anchors, f_anchors, indent=2)

        # 7. Metadata JSON
        meta_json = os.path.join(self.export_dir, "scan_metadata.json")
        meta_data = {
            "session_id": str(self.dataset_name),
            "export_timestamp": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            "capture_duration_seconds": float(round(duration, 2)),
            "total_sweeps_recorded": int(self.sweep_count),
            "total_raw_points_accumulated": int(self.total_raw_points_received),
            "total_raw_rgb_observations": int(self.total_colored_points_received),
            "final_voxel_map_points": int(n_points),
            "final_points_with_rgb": int(colored_voxels),
            "accumulated_rgb_coverage_percent": float(round(coverage_pct, 2)),
            "voxel_resolution_m": float(self.voxel_size),
            "spatial_bounding_box": {
                "x_min_m": float(xyz[:, 0].min()) if n_points > 0 else 0.0,
                "x_max_m": float(xyz[:, 0].max()) if n_points > 0 else 0.0,
                "y_min_m": float(xyz[:, 1].min()) if n_points > 0 else 0.0,
                "y_max_m": float(xyz[:, 1].max()) if n_points > 0 else 0.0,
                "z_min_m": float(xyz[:, 2].min()) if n_points > 0 else 0.0,
                "z_max_m": float(xyz[:, 2].max()) if n_points > 0 else 0.0,
            },
            "calibration_provenance": {
                "extrinsic_T_rgb_airy_lidar": [0.023855, 0.014534, -0.055265],
                "extrinsic_R_rgb_airy_lidar_row_major": [
                    0.000350, -0.999997,  0.002377,
                    -0.001316, -0.002378, -0.999996,
                    0.999999,  0.000347, -0.001317
                ],
                "rgb_camera_intrinsics_K": [609.006531, 0.0, 643.868774, 0.0, 609.015625, 399.326538, 0.0, 0.0, 1.0],
                "rgb_camera_distortion_D": [-0.03113378, 0.03646491, 0.00048966, -0.00030747, -0.01320058],
                "lidar_model": "RoboSense Airy 192 (10Hz Master)",
                "imu_model": "RoboSense Internal IMU (200Hz)",
                "camera_model": "Orbbec Gemini 336L RGB (30Hz Slave, 1280x800)",
                "slam_backend": "FAST-LIVO2 (Pure LIO Tracking)"
            }
        }
        with open(meta_json, 'w') as f_meta:
            json.dump(meta_data, f_meta, indent=2)

        self.get_logger().info(f"[SAVE COMPLETE] Successfully exported 7 dataset artifacts to {self.export_dir}")
        return meta_data, xyz, rgb

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


def render_verification_views(export_dir, xyz, rgb, output_artifact_path):
    """Render 3D evaluation visualization of the accumulated map."""
    fig = plt.figure(figsize=(18, 8), facecolor='#161b22')
    
    # Colors normalized
    colors = rgb.astype(np.float32) / 255.0
    step = max(1, len(xyz) // 50000) # Render up to 50k points

    # View 1: 3D Isometric / Oblique
    ax1 = fig.add_subplot(1, 3, 1, projection='3d', facecolor='#0d1117')
    ax1.scatter(xyz[::step, 0], xyz[::step, 1], xyz[::step, 2], c=colors[::step], s=1.2, alpha=0.85)
    ax1.set_title('M12.3: 3D Oblique Perspective', color='white', fontsize=12, pad=10)
    ax1.set_xlabel('X (m)', color='gray')
    ax1.set_ylabel('Y (m)', color='gray')
    ax1.set_zlabel('Z (m)', color='gray')
    ax1.tick_params(colors='gray')
    ax1.view_init(elev=28, azim=-55)

    # View 2: Plan View (Top-Down)
    ax2 = fig.add_subplot(1, 3, 2, facecolor='#0d1117')
    ax2.scatter(xyz[::step, 0], xyz[::step, 1], c=colors[::step], s=1.5, alpha=0.9)
    ax2.set_title('M12.3: Plan View (Top-Down Floor Map)', color='white', fontsize=12, pad=10)
    ax2.set_xlabel('X (m)', color='gray')
    ax2.set_ylabel('Y (m)', color='gray')
    ax2.tick_params(colors='gray')
    ax2.grid(True, color='#21262d', linestyle='--', alpha=0.5)
    ax2.axis('equal')

    # View 3: Front Elevation
    ax3 = fig.add_subplot(1, 3, 3, facecolor='#0d1117')
    ax3.scatter(xyz[::step, 0], xyz[::step, 2], c=colors[::step], s=1.5, alpha=0.9)
    ax3.set_title('M12.3: Elevation View (X-Z Front)', color='white', fontsize=12, pad=10)
    ax3.set_xlabel('X (m)', color='gray')
    ax3.set_ylabel('Z (m)', color='gray')
    ax3.tick_params(colors='gray')
    ax3.grid(True, color='#21262d', linestyle='--', alpha=0.5)
    ax3.axis('equal')

    plt.tight_layout()
    plt.savefig(output_artifact_path, dpi=180, facecolor=fig.get_facecolor(), edgecolor='none')
    plt.close(fig)
    print(f"[VERIFY] Rendered 3D verification multi-view to {output_artifact_path}")


def verify_saved_ply(ply_path):
    """Independently re-open the saved PLY file and verify header and data."""
    print(f"\n[VERIFY] Independently opening and parsing: {ply_path}")
    with open(ply_path, 'rb') as f:
        header = b""
        while True:
            line = f.readline()
            header += line
            if line.strip() == b"end_header":
                break
        raw_data = f.read()

    header_str = header.decode('ascii')
    print("PLY Header Summary:")
    for l in header_str.splitlines():
        print(f"  {l}")

    # Parse binary payload
    dt = np.dtype([
        ('x', '<f4'), ('y', '<f4'), ('z', '<f4'),
        ('r', 'u1'), ('g', 'u1'), ('b', 'u1')
    ])
    pts = np.frombuffer(raw_data, dtype=dt)
    print(f"[VERIFY RESULT] Vertex count: {len(pts)}")
    print(f"[VERIFY RESULT] X range: [{pts['x'].min():.3f}, {pts['x'].max():.3f}] m")
    print(f"[VERIFY RESULT] Y range: [{pts['y'].min():.3f}, {pts['y'].max():.3f}] m")
    print(f"[VERIFY RESULT] Z range: [{pts['z'].min():.3f}, {pts['z'].max():.3f}] m")

    colored_mask = ~((pts['r'] == 160) & (pts['g'] == 160) & (pts['b'] == 160))
    n_colored = np.sum(colored_mask)
    pct_colored = (n_colored / len(pts) * 100.0) if len(pts) > 0 else 0.0

    print(f"[VERIFY RESULT] Points with real RGB coverage: {n_colored} ({pct_colored:.2f}%)")
    print(f"[VERIFY RESULT] R channel: min={pts['r'].min()}, max={pts['r'].max()}, mean={pts['r'].mean():.1f}, std={pts['r'].std():.1f}")
    print(f"[VERIFY RESULT] G channel: min={pts['g'].min()}, max={pts['g'].max()}, mean={pts['g'].mean():.1f}, std={pts['g'].std():.1f}")
    print(f"[VERIFY RESULT] B channel: min={pts['b'].min()}, max={pts['b'].max()}, mean={pts['b'].mean():.1f}, std={pts['b'].std():.1f}")

    assert len(pts) > 0, "Error: PLY has 0 vertices!"
    assert pts['r'].std() > 0.1, "Error: R channel has 0 variance!"
    assert pts['g'].std() > 0.1, "Error: G channel has 0 variance!"
    assert pts['b'].std() > 0.1, "Error: B channel has 0 variance!"
    print("✅ INDEPENDENT PLY VERIFICATION PASSED!")
    return len(pts), n_colored, pct_colored


def verify_saved_pcd(pcd_path):
    """Independently re-open the saved PCD file and verify header and data."""
    print(f"\n[VERIFY] Independently opening and parsing: {pcd_path}")
    with open(pcd_path, 'rb') as f:
        header = b""
        while True:
            line = f.readline()
            header += line
            if line.strip() == b"DATA binary":
                break
        raw_data = f.read()

    header_str = header.decode('ascii')
    print("PCD Header Summary:")
    for l in header_str.splitlines():
        print(f"  {l}")

    dt = np.dtype([
        ('x', '<f4'), ('y', '<f4'), ('z', '<f4'),
        ('rgb', '<f4')
    ])
    pts = np.frombuffer(raw_data, dtype=dt)
    print(f"[VERIFY RESULT] PCD Record count: {len(pts)}")
    assert len(pts) > 0, "Error: PCD has 0 records!"
    print("✅ INDEPENDENT PCD VERIFICATION PASSED!")
    return len(pts)


def main():
    rclpy.init()
    runner = M123ExportRunner(duration_sec=20.0, dataset_name="M12_3_Production_Capture")
    
    print("\n==================================================")
    print("Starting M12.3 Real Production Recording (20 Seconds)...")
    print("Accumulating /cloud_registered_rgb and /aft_mapped_to_init")
    print("==================================================")

    runner.is_recording = True
    runner.start_time = time.time()

    try:
        while time.time() - runner.start_time < runner.duration_sec:
            rclpy.spin_once(runner, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass

    duration = time.time() - runner.start_time
    runner.is_recording = False
    print(f"\nRecording completed ({duration:.2f}s). Exporting dataset...")

    meta_data, xyz, rgb = runner.export_all(duration)

    # Clean up ROS
    runner.destroy_node()
    rclpy.shutdown()

    # Independent Verification
    ply_rgb_file = os.path.join(runner.export_dir, "pointcloud_map_rgb.ply")
    pcd_rgb_file = os.path.join(runner.export_dir, "pointcloud_map_rgb.pcd")
    
    n_pts, n_col, pct_col = verify_saved_ply(ply_rgb_file)
    n_pcd = verify_saved_pcd(pcd_rgb_file)

    # Render multi-view visual artifact
    artifact_render = "/home/scanar/.gemini/antigravity-cli/brain/b3ab30fd-83ab-473b-8594-f07c20d0194c/m12_3_accumulated_map_render.png"
    render_verification_views(runner.export_dir, xyz, rgb, artifact_render)

    print("\n==================================================")
    print("🎉 M12.3 EXPORT & VERIFICATION COMPLETE!")
    print(f"Total Exported Voxels: {n_pts}")
    print(f"Map-Wide RGB Coverage: {n_col} points ({pct_col:.2f}%)")
    print("==================================================")

if __name__ == '__main__':
    main()
