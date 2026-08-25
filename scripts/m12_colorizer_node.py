#!/usr/bin/env python3
"""
Milestone M12.2: Live RGB Point Cloud Colorization Engine
Subscribes to:
  - /cloud_registered (sensor_msgs/PointCloud2, frame_id='camera_init')
  - /aft_mapped_to_init (nav_msgs/Odometry, world pose of body)
  - /camera/color/image_raw (sensor_msgs/Image, 1280x800 RGB)
  - /camera/color/camera_info (sensor_msgs/CameraInfo)

Publishes:
  - /cloud_registered_rgb (sensor_msgs/PointCloud2 with XYZ + RGB)

Preserves exact calibrated T_rgb_airy_lidar and camera intrinsics.
"""

import os
import sys
import time
import math
import struct
import cv2
import yaml
import psutil
import numpy as np
from collections import deque
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, PointCloud2, PointField, CameraInfo
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from cv_bridge import CvBridge

# Calibrated Extrinsic Matrix T_rgb_airy_lidar
T_RGB_LIDAR = np.array([
    [ 0.000350, -0.999997,  0.002377,  0.023855],
    [-0.001316, -0.002378, -0.999996,  0.014534],
    [ 0.999999,  0.000347, -0.001317, -0.055265],
    [ 0.000000,  0.000000,  0.000000,  1.000000]
], dtype=np.float64)

R_RGB_LIDAR = T_RGB_LIDAR[:3, :3]
T_RGB_LIDAR_VEC = T_RGB_LIDAR[:3, 3].reshape((3, 1))
RVEC_RGB_LIDAR, _ = cv2.Rodrigues(R_RGB_LIDAR)

# Default Intrinsics
K_DEFAULT = np.array([
    [609.006531,   0.000000, 643.868774],
    [  0.000000, 609.015625, 399.326538],
    [  0.000000,   0.000000,   1.000000]
], dtype=np.float64)

D_DEFAULT = np.array([-0.03113378, 0.03646491, 0.00048966, -0.00030747, -0.01320058], dtype=np.float64)


def quaternion_to_rotation_matrix(q):
    """Convert [qx, qy, qz, qw] to 3x3 rotation matrix."""
    qx, qy, qz, qw = q
    # Normalize
    n = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    if n < 1e-10:
        return np.eye(3)
    qx, qy, qz, qw = qx/n, qy/n, qz/n, qw/n

    return np.array([
        [1.0 - 2.0*(qy*qy + qz*qz), 2.0*(qx*qy - qz*qw),     2.0*(qx*qz + qy*qw)],
        [2.0*(qx*qy + qz*qw),     1.0 - 2.0*(qx*qx + qz*qz), 2.0*(qy*qz - qx*qw)],
        [2.0*(qx*qz - qy*qw),     2.0*(qy*qz + qx*qw),     1.0 - 2.0*(qx*qx + qy*qy)]
    ], dtype=np.float64)


def save_pointcloud_ply(filename, points_xyz, colors_rgb):
    """Write point cloud to standard binary-little-endian PLY."""
    n = len(points_xyz)
    if n == 0:
        return
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
    ).encode('utf-8')

    # Structured data
    dtype = np.dtype([
        ('x', '<f4'), ('y', '<f4'), ('z', '<f4'),
        ('r', 'u1'), ('g', 'u1'), ('b', 'u1')
    ])
    arr = np.empty(n, dtype=dtype)
    arr['x'] = points_xyz[:, 0].astype(np.float32)
    arr['y'] = points_xyz[:, 1].astype(np.float32)
    arr['z'] = points_xyz[:, 2].astype(np.float32)
    arr['r'] = colors_rgb[:, 0].astype(np.uint8)
    arr['g'] = colors_rgb[:, 1].astype(np.uint8)
    arr['b'] = colors_rgb[:, 2].astype(np.uint8)

    with open(filename, 'wb') as f:
        f.write(header)
        f.write(arr.tobytes())


class M12ColorizerNode(Node):
    def __init__(self):
        super().__init__('m12_live_colorizer')
        self.bridge = CvBridge()
        self.output_dir = "/home/scanar/scanarMini/data/m12_validation"
        os.makedirs(self.output_dir, exist_ok=True)

        self.K = K_DEFAULT
        self.D = D_DEFAULT
        self.camera_info_received = False

        # Ring buffers for temporal synchronization
        self.rgb_buffer = deque(maxlen=60) # Store ~2 seconds of RGB frames
        self.odom_buffer = deque(maxlen=200) # Store ~2 seconds of SLAM poses

        # Telemetry metrics
        self.cloud_in_count = 0
        self.cloud_out_count = 0
        self.rgb_in_count = 0
        self.dropped_frames = 0
        self.saved_artifact_count = 0
        self.t_start_time = time.time()

        # Rates
        self.last_stats_time = time.time()
        self.stats_interval = 2.0 # log every 2 seconds

        qos_sub = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # 1. Camera Info
        self.sub_info = self.create_subscription(
            CameraInfo,
            '/camera/color/camera_info',
            self.camera_info_callback,
            qos_sub
        )

        # 2. RGB Image
        self.sub_rgb = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.rgb_callback,
            qos_sub
        )

        # 3. Authoritative SLAM Pose
        self.sub_odom = self.create_subscription(
            Odometry,
            '/aft_mapped_to_init',
            self.odom_callback,
            qos_sub
        )

        # 4. SLAM Registered Point Cloud
        self.sub_cloud = self.create_subscription(
            PointCloud2,
            '/cloud_registered',
            self.cloud_callback,
            qos_sub
        )

        # Output Publisher
        self.pub_cloud_rgb = self.create_publisher(
            PointCloud2,
            '/cloud_registered_rgb',
            10
        )

        # Process monitor for CPU / RAM
        self.process = psutil.Process()

        self.get_logger().info("==================================================")
        self.get_logger().info("M12.2 Live RGB Point Cloud Colorizer Online")
        self.get_logger().info("Subscribing: /cloud_registered, /aft_mapped_to_init, /camera/color/image_raw")
        self.get_logger().info("Publishing:  /cloud_registered_rgb")
        self.get_logger().info("==================================================")

    def camera_info_callback(self, msg: CameraInfo):
        if not self.camera_info_received:
            self.K = np.array(msg.k, dtype=np.float64).reshape((3, 3))
            if len(msg.d) >= 5:
                self.D = np.array(msg.d[:5], dtype=np.float64)
            self.camera_info_received = True

    def rgb_callback(self, msg: Image):
        self.rgb_in_count += 1
        sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        try:
            enc = msg.encoding.lower()
            raw = bytes(msg.data)
            if "yuv" in enc or "yuyv" in enc:
                yuv = np.frombuffer(raw, dtype=np.uint8).reshape((msg.height, msg.width, 2))
                bgr = cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_YUY2)
            elif "bgr8" in enc:
                bgr = np.frombuffer(raw, dtype=np.uint8).reshape((msg.height, msg.width, 3)).copy()
            elif "rgb8" in enc:
                rgb = np.frombuffer(raw, dtype=np.uint8).reshape((msg.height, msg.width, 3))
                bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            else:
                bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            self.rgb_buffer.append((sec, bgr, msg.header))
        except Exception as e:
            self.get_logger().error(f"RGB Decode Exception: {e}")

    def odom_callback(self, msg: Odometry):
        sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        pos = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ], dtype=np.float64)
        quat = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ]
        R_wb = quaternion_to_rotation_matrix(quat)
        self.odom_buffer.append((sec, pos, R_wb))

    def get_closest_rgb(self, target_sec):
        if not self.rgb_buffer:
            return None, None, None, 999.0
        best_diff = 999.0
        best_item = None
        for item in self.rgb_buffer:
            diff = abs(item[0] - target_sec)
            if diff < best_diff:
                best_diff = diff
                best_item = item
        if best_item is not None:
            return best_item[0], best_item[1], best_item[2], (target_sec - best_item[0]) * 1000.0
        return None, None, None, 999.0

    def get_closest_odom(self, target_sec):
        if not self.odom_buffer:
            return None, None
        best_diff = 999.0
        best_item = None
        for item in self.odom_buffer:
            diff = abs(item[0] - target_sec)
            if diff < best_diff:
                best_diff = diff
                best_item = item
        if best_item is not None and best_diff < 0.15: # within 150ms
            return best_item[1], best_item[2] # pos, R_wb
        # Default identity if SLAM at origin
        return np.zeros(3, dtype=np.float64), np.eye(3, dtype=np.float64)

    def cloud_callback(self, msg: PointCloud2):
        self.cloud_in_count += 1
        t_proc_start = time.time()
        cloud_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        # 1. Match synchronized RGB frame
        rgb_sec, img_bgr, img_header, sync_delta_ms = self.get_closest_rgb(cloud_sec)
        if img_bgr is None:
            self.dropped_frames += 1
            return

        # 2. Match SLAM body pose
        pos_wb, R_wb = self.get_closest_odom(cloud_sec)

        num_points = msg.width * msg.height
        if num_points == 0 or msg.point_step < 12:
            return

        try:
            # Unpack PointCloud2
            raw_bytes = bytes(msg.data)
            dt = np.dtype({
                'names': ['x', 'y', 'z'],
                'formats': ['<f4', '<f4', '<f4'],
                'offsets': [0, 4, 8],
                'itemsize': msg.point_step
            })
            pts_raw = np.frombuffer(raw_bytes, dtype=dt)
            valid_mask = np.isfinite(pts_raw['x']) & np.isfinite(pts_raw['y']) & np.isfinite(pts_raw['z'])
            pts_valid = pts_raw[valid_mask]
            total_valid_pts = len(pts_valid)
            if total_valid_pts == 0:
                return

            # World points (N, 3)
            pts_w = np.column_stack((pts_valid['x'], pts_valid['y'], pts_valid['z'])).astype(np.float64)

            # Transform from World frame to Body (LiDAR) frame: p_b = R_wb.T * (p_w - pos_wb)
            pts_b = (R_wb.T @ (pts_w - pos_wb).T).T

            # Transform from LiDAR frame to RGB optical frame: p_cam = R_rgb_lidar * p_b + t_rgb_lidar
            pts_cam = (R_RGB_LIDAR @ pts_b.T + T_RGB_LIDAR_VEC).T
            z_cam = pts_cam[:, 2]

            # Rejection Breakdown
            behind_mask = z_cam <= 0.2
            rejections_behind = np.sum(behind_mask)

            in_front_mask = (z_cam > 0.2) & (z_cam < 30.0)
            pts_in_front_b = pts_b[in_front_mask]
            pts_in_front_w = pts_w[in_front_mask]
            pts_in_front_cam = pts_cam[in_front_mask]

            h, w, _ = img_bgr.shape
            rejections_out_of_bounds = 0
            successfully_colored = 0

            # Default colors (neutral grey for points outside RGB camera FOV)
            colors_rgb = np.full((total_valid_pts, 3), [160, 160, 160], dtype=np.uint8)

            if len(pts_in_front_cam) > 0:
                projected_2d, _ = cv2.projectPoints(
                    pts_in_front_b,
                    RVEC_RGB_LIDAR,
                    T_RGB_LIDAR_VEC,
                    self.K,
                    self.D
                )
                u = np.round(projected_2d[:, 0, 0]).astype(np.int32)
                v = np.round(projected_2d[:, 0, 1]).astype(np.int32)

                in_fov = (u >= 0) & (u < w) & (v >= 0) & (v < h)
                rejections_out_of_bounds = len(u) - np.sum(in_fov)
                successfully_colored = np.sum(in_fov)

                # Sample pixel color from RGB image (OpenCV is BGR -> convert to RGB)
                if successfully_colored > 0:
                    u_fov = u[in_fov]
                    v_fov = v[in_fov]
                    sampled_bgr = img_bgr[v_fov, u_fov]
                    sampled_rgb = sampled_bgr[:, [2, 1, 0]] # BGR to RGB

                    # Map back to global index
                    in_front_indices = np.where(in_front_mask)[0]
                    fov_indices = in_front_indices[in_fov]
                    colors_rgb[fov_indices] = sampled_rgb

            # Pack Float32 RGB for ROS 2 PointCloud2
            # rgb float packed: (R << 16) | (G << 8) | B
            r = colors_rgb[:, 0].astype(np.uint32)
            g = colors_rgb[:, 1].astype(np.uint32)
            b = colors_rgb[:, 2].astype(np.uint32)
            rgb_packed_int = (r << 16) | (g << 8) | b
            rgb_packed_float = rgb_packed_int.view(np.float32)

            # Build PointCloud2 structured array
            out_dtype = np.dtype([
                ('x', '<f4'),
                ('y', '<f4'),
                ('z', '<f4'),
                ('rgb', '<f4')
            ])
            out_arr = np.empty(total_valid_pts, dtype=out_dtype)
            out_arr['x'] = pts_w[:, 0].astype(np.float32)
            out_arr['y'] = pts_w[:, 1].astype(np.float32)
            out_arr['z'] = pts_w[:, 2].astype(np.float32)
            out_arr['rgb'] = rgb_packed_float

            # Create ROS 2 PointCloud2 Message
            out_msg = PointCloud2()
            out_msg.header = msg.header
            out_msg.header.frame_id = "camera_init"
            out_msg.height = 1
            out_msg.width = total_valid_pts
            out_msg.is_dense = True
            out_msg.is_bigendian = False
            out_msg.point_step = 16
            out_msg.row_step = total_valid_pts * 16
            out_msg.fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
            ]
            out_msg.data = out_arr.tobytes()

            self.pub_cloud_rgb.publish(out_msg)
            self.cloud_out_count += 1
            proc_time_ms = (time.time() - t_proc_start) * 1000.0

            # Save diagnostic snapshots
            if self.cloud_out_count % 15 == 1 and self.saved_artifact_count < 8:
                self.saved_artifact_count += 1
                prefix = f"m12_2_colorized_val_{self.saved_artifact_count:02d}"
                
                # 1. Source RGB
                cv2.imwrite(os.path.join(self.output_dir, f"{prefix}_rgb_source.png"), img_bgr)
                
                # 2. Colorized Point Cloud (.ply)
                ply_path = os.path.join(self.output_dir, f"{prefix}_cloud.ply")
                save_pointcloud_ply(ply_path, pts_w, colors_rgb)
                
                # 3. Projected 2D Verification Overlay
                overlay = img_bgr.copy()
                if successfully_colored > 0:
                    for px, py, crgb in zip(u_fov, v_fov, sampled_rgb):
                        c = (int(crgb[2]), int(crgb[1]), int(crgb[0])) # to BGR
                        cv2.circle(overlay, (px, py), 2, c, -1)
                
                cv2.rectangle(overlay, (10, 10), (580, 110), (15, 20, 25), -1)
                cv2.rectangle(overlay, (10, 10), (580, 110), (56, 185, 80), 1)
                cv2.putText(overlay, "M12.2: LIVE RGB POINT CLOUD COLORIZATION", (20, 32),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.52, (56, 185, 80), 2)
                cv2.putText(overlay, f"Total Registered Pts: {total_valid_pts} | Colored: {successfully_colored}", (20, 54),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.44, (200, 220, 240), 1)
                cv2.putText(overlay, f"Sync Delta: {sync_delta_ms:+.1f} ms | Latency: {proc_time_ms:.1f} ms", (20, 72),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.44, (56, 185, 80), 1)
                cv2.putText(overlay, f"Rejections: Behind={rejections_behind}, OutOfFOV={rejections_out_of_bounds}", (20, 92),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.40, (139, 148, 158), 1)
                cv2.imwrite(os.path.join(self.output_dir, f"{prefix}_overlay.png"), overlay)

                self.get_logger().info(f"[M12.2 ARTIFACT] Saved snapshot #{self.saved_artifact_count}: {ply_path} ({successfully_colored} colored pts)")

            # Periodic Status Reporting
            t_now = time.time()
            if t_now - self.last_stats_time >= self.stats_interval:
                dt_stats = t_now - self.last_stats_time
                cloud_hz = self.cloud_in_count / (t_now - self.t_start_time)
                rgb_hz = self.rgb_in_count / (t_now - self.t_start_time)
                cpu_pct = self.process.cpu_percent()
                mem_mb = self.process.memory_info().rss / (1024 * 1024)

                self.get_logger().info(
                    f"[M12.2 STATS] Rate: Cloud={cloud_hz:.1f}Hz, RGB={rgb_hz:.1f}Hz, Out={self.cloud_out_count/(t_now-self.t_start_time):.1f}Hz | "
                    f"Sync Delta: {sync_delta_ms:+.1f}ms | Latency: {proc_time_ms:.1f}ms | "
                    f"Pts: Valid={total_valid_pts}, Colored={successfully_colored}, OutOfFOV={rejections_out_of_bounds} | "
                    f"Host: CPU={cpu_pct:.1f}%, RAM={mem_mb:.1f}MB, Drops={self.dropped_frames}"
                )
                self.last_stats_time = t_now

        except Exception as e:
            self.get_logger().error(f"Colorizer Processing Exception: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = M12ColorizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
