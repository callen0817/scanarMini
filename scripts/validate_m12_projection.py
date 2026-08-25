#!/usr/bin/env python3
"""
Milestone M12.1: RGB <-> LiDAR Projection Validation Script
Projects raw/registered LiDAR points onto the 1280x800 RGB stream.

Calibrated Extrinsics (T_rgb_airy_lidar):
p_rgb = R_rgb_lidar * p_lidar + T_rgb_lidar

Intrinsics (Orbbec Gemini 336L RGB):
fx=609.0065, fy=609.0156, cx=643.8688, cy=399.3265
Distortion: plumb_bob [-0.031134, 0.036465, 0.000490, -0.000307, -0.013201]
"""

import os
import sys
import time
import math
import cv2
import yaml
import numpy as np
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from cv_bridge import CvBridge

# Calibrated Extrinsic Matrix T_rgb_airy_lidar from config/production_extrinsics.yaml
T_RGB_LIDAR_DEFAULT = np.array([
    [ 0.000350, -0.999997,  0.002377,  0.023855],
    [-0.001316, -0.002378, -0.999996,  0.014534],
    [ 0.999999,  0.000347, -0.001317, -0.055265],
    [ 0.000000,  0.000000,  0.000000,  1.000000]
], dtype=np.float64)

# Intrinsic Camera Matrix (1280x800)
K_DEFAULT = np.array([
    [609.006531,   0.000000, 643.868774],
    [  0.000000, 609.015625, 399.326538],
    [  0.000000,   0.000000,   1.000000]
], dtype=np.float64)

# Distortion Coefficients
D_DEFAULT = np.array([-0.03113378, 0.03646491, 0.00048966, -0.00030747, -0.01320058], dtype=np.float64)


class M12ProjectionValidator(Node):
    def __init__(self):
        super().__init__('m12_projection_validator')
        self.bridge = CvBridge()
        
        # Output directory for validation snapshots
        self.output_dir = "/home/scanar/scanarMini/data/m12_validation"
        os.makedirs(self.output_dir, exist_ok=True)

        self.T_rgb_lidar = T_RGB_LIDAR_DEFAULT
        self.R_rgb_lidar = self.T_rgb_lidar[:3, :3]
        self.t_rgb_lidar = self.T_rgb_lidar[:3, 3].reshape((3, 1))
        
        # Rodrigues rotation vector for cv2.projectPoints
        self.rvec, _ = cv2.Rodrigues(self.R_rgb_lidar)
        self.tvec = self.t_rgb_lidar

        self.K = K_DEFAULT
        self.D = D_DEFAULT
        self.camera_info_received = False

        self.last_image = None
        self.last_image_stamp = 0.0
        self.last_image_header = None
        self.frame_count = 0
        self.saved_count = 0

        # Subscriptions
        qos_sub = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        self.sub_info = self.create_subscription(
            CameraInfo,
            '/camera/color/camera_info',
            self.camera_info_callback,
            qos_sub
        )

        self.sub_rgb = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.rgb_callback,
            qos_sub
        )

        self.sub_lidar = self.create_subscription(
            PointCloud2,
            '/rslidar_points',
            self.lidar_callback,
            qos_sub
        )

        # Publisher for overlay stream
        self.pub_overlay = self.create_publisher(
            Image,
            '/camera/color/lidar_projection_overlay',
            10
        )

        self.get_logger().info("==================================================")
        self.get_logger().info("M12.1 RGB <-> LiDAR Projection Validator Initialized")
        self.get_logger().info(f"T_rgb_lidar Translation: {self.t_rgb_lidar.ravel()} m")
        self.get_logger().info(f"Snapshots will be saved to: {self.output_dir}")
        self.get_logger().info("==================================================")

    def camera_info_callback(self, msg: CameraInfo):
        if not self.camera_info_received:
            self.K = np.array(msg.k, dtype=np.float64).reshape((3, 3))
            if len(msg.d) >= 5:
                self.D = np.array(msg.d[:5], dtype=np.float64)
            self.camera_info_received = True
            self.get_logger().info(f"[CAMERA INFO] Loaded K: fx={self.K[0,0]:.2f}, fy={self.K[1,1]:.2f}, cx={self.K[0,2]:.2f}, cy={self.K[1,2]:.2f}")

    def rgb_callback(self, msg: Image):
        try:
            sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            enc = msg.encoding.lower()
            raw_bytes = bytes(msg.data)
            
            if "yuv" in enc or "yuyv" in enc:
                yuv = np.frombuffer(raw_bytes, dtype=np.uint8).reshape((msg.height, msg.width, 2))
                bgr = cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_YUY2)
            elif "bgr8" in enc:
                bgr = np.frombuffer(raw_bytes, dtype=np.uint8).reshape((msg.height, msg.width, 3)).copy()
            elif "rgb8" in enc:
                rgb = np.frombuffer(raw_bytes, dtype=np.uint8).reshape((msg.height, msg.width, 3))
                bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            else:
                bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            self.last_image = bgr
            self.last_image_stamp = sec
            self.last_image_header = msg.header
        except Exception as e:
            self.get_logger().error(f"RGB Callback Error: {e}")

    def lidar_callback(self, msg: PointCloud2):
        if self.last_image is None:
            return

        t_start = time.time()
        lidar_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        dt_sync_ms = (lidar_sec - self.last_image_stamp) * 1000.0

        # Snapshot the paired image
        img_bgr = self.last_image.copy()
        h, w, _ = img_bgr.shape

        # Unpack PointCloud2
        num_points = msg.width * msg.height
        if num_points == 0 or msg.point_step < 12:
            return

        try:
            raw_bytes = bytes(msg.data)
            dt = np.dtype({
                'names': ['x', 'y', 'z', 'intensity'],
                'formats': ['<f4', '<f4', '<f4', '<f4'],
                'offsets': [0, 4, 8, 12 if msg.point_step >= 16 else 8],
                'itemsize': msg.point_step
            })
            pts_raw = np.frombuffer(raw_bytes, dtype=dt)
            valid = np.isfinite(pts_raw['x']) & np.isfinite(pts_raw['y']) & np.isfinite(pts_raw['z'])
            pts_valid = pts_raw[valid]
            if len(pts_valid) == 0:
                return

            # Extract 3D points in LiDAR coordinates (N, 3)
            pts_lidar = np.column_stack((pts_valid['x'], pts_valid['y'], pts_valid['z'])).astype(np.float64)

            # Transform into camera frame: p_cam = R * p_lidar + t
            pts_cam = (self.R_rgb_lidar @ pts_lidar.T + self.t_rgb_lidar).T # Shape (N, 3)
            z_cam = pts_cam[:, 2]

            # Filter points in front of camera (Z > 0.2m and Z < 25m)
            fov_mask = (z_cam > 0.2) & (z_cam < 25.0)
            pts_cam_fov = pts_cam[fov_mask]
            pts_lidar_fov = pts_lidar[fov_mask]
            depths_fov = z_cam[fov_mask]

            if len(pts_cam_fov) == 0:
                return

            # Project using OpenCV with distortion
            projected_2d, _ = cv2.projectPoints(
                pts_lidar_fov,
                self.rvec,
                self.tvec,
                self.K,
                self.D
            )
            u = projected_2d[:, 0, 0]
            v = projected_2d[:, 0, 1]

            # Inside image bounds filter
            in_bounds = (u >= 0) & (u < w) & (v >= 0) & (v < h)
            u_valid = u[in_bounds].astype(np.int32)
            v_valid = v[in_bounds].astype(np.int32)
            d_valid = depths_fov[in_bounds]

            # Colormap based on depth (0.3m to 6.0m dynamic range)
            d_min, d_max = 0.3, 6.0
            norm_d = np.clip((d_valid - d_min) / (d_max - d_min), 0.0, 1.0)
            # Turbo / Jet colormap (Red = near, Blue = far)
            colormap_lut = cv2.applyColorMap((norm_d * 255).astype(np.uint8), cv2.COLORMAP_TURBO)

            # Draw projected LiDAR points on image
            overlay = img_bgr.copy()
            for px, py, color, dist in zip(u_valid, v_valid, colormap_lut[:, 0, :], d_valid):
                # Adaptive radius: 2px for close range (<2m), 1px for far range
                radius = 2 if dist < 2.5 else 1
                c = (int(color[0]), int(color[1]), int(color[2]))
                cv2.circle(overlay, (px, py), radius, c, -1)

            # Blend with original image for crisp visualization (alpha=0.75 overlay, beta=0.25 original)
            blended = cv2.addWeighted(overlay, 0.85, img_bgr, 0.15, 0)

            # HUD Telemetry Banner
            self.frame_count += 1
            cv2.rectangle(blended, (10, 10), (560, 100), (15, 20, 25), -1)
            cv2.rectangle(blended, (10, 10), (560, 100), (56, 185, 80), 1)

            cv2.putText(blended, "M12.1: RGB <-> LiDAR PROJECTION QUALIFICATION", (20, 32),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.52, (56, 185, 80), 2)
            cv2.putText(blended, f"LiDAR Sweep Pts: {num_points} | Projected in FOV: {len(u_valid)}", (20, 54),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.44, (200, 220, 240), 1)
            cv2.putText(blended, f"Sync Delta: {dt_sync_ms:+.1f} ms | Frame Rate: {1.0/(time.time()-t_start):.1f} Hz", (20, 72),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.44, (56, 185, 80) if abs(dt_sync_ms) < 50 else (0, 165, 255), 1)
            cv2.putText(blended, f"Depth Color Range: 0.3m (Red) -> 6.0m (Blue)", (20, 90),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.40, (139, 148, 158), 1)

            # Publish overlay message
            out_msg = self.bridge.cv2_to_imgmsg(blended, encoding="bgr8")
            out_msg.header = self.last_image_header
            self.pub_overlay.publish(out_msg)

            # Periodically or on first 5 frames save high-res snapshots
            if self.frame_count % 15 == 1 and self.saved_count < 10:
                self.saved_count += 1
                out_path = os.path.join(self.output_dir, f"m12_projection_val_{self.saved_count:02d}.png")
                cv2.imwrite(out_path, blended)
                self.get_logger().info(f"[M12.1 SAVED] Saved validation frame #{self.saved_count} to: {out_path} ({len(u_valid)} projected points)")

            if self.frame_count % 30 == 1:
                self.get_logger().info(f"[M12.1] Frame #{self.frame_count} | Projected {len(u_valid)} pts | Sync {dt_sync_ms:+.1f}ms | Compute {((time.time()-t_start)*1000):.1f}ms")

        except Exception as e:
            self.get_logger().error(f"LiDAR Processing Error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = M12ProjectionValidator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
