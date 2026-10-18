#!/usr/bin/env python3
import os
import sys
import time
import json
import queue
import threading
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, Image, Imu, CameraInfo
from nav_msgs.msg import Odometry

class LightweightCaptureRecorder(Node):
    def __init__(self, output_dir):
        super().__init__('lightweight_capture_recorder')
        self.output_dir = output_dir
        os.makedirs(os.path.join(output_dir, 'depth'), exist_ok=True)
        os.makedirs(os.path.join(output_dir, 'color'), exist_ok=True)
        os.makedirs(os.path.join(output_dir, 'lidar'), exist_ok=True)
        
        self.write_queue = queue.Queue(maxsize=1000)
        self.running = True
        self.worker_thread = threading.Thread(target=self._writer_loop, daemon=True)
        self.worker_thread.start()
        
        self.traj_file = open(os.path.join(output_dir, 'trajectory.csv'), 'w')
        self.traj_file.write('timestamp,x,y,z,qx,qy,qz,qw\n')
        
        self.tum_file = open(os.path.join(output_dir, 'trajectory.tum'), 'w')
        
        self.lidar_idx = 0
        self.depth_idx = 0
        self.color_idx = 0
        self.odom_cnt = 0
        self.start_time = time.time()
        
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.last_depth_stamp = 0.0
        self.last_color_stamp = 0.0
        self.min_img_interval = 0.14  # ~7 FPS keyframes for camera post-processing

        self.create_subscription(PointCloud2, '/rslidar_points', self.cb_lidar, qos_sensor)
        self.create_subscription(Image, '/camera/depth/image_raw', self.cb_depth, qos_sensor)
        self.create_subscription(Image, '/camera/color/image_raw', self.cb_color, qos_sensor)
        self.create_subscription(CameraInfo, '/camera/depth/camera_info', self.cb_d_info, qos_reliable)
        self.create_subscription(CameraInfo, '/camera/color/camera_info', self.cb_c_info, qos_reliable)
        self.create_subscription(Odometry, '/aft_mapped_to_init', self.cb_odom, qos_sensor)
        
        self.d_info_saved = False
        self.c_info_saved = False
        self.get_logger().info(f'Lightweight Capture Recorder initialized at {output_dir}')

    def cb_d_info(self, msg):
        if not self.d_info_saved:
            info = {
                'width': msg.width, 'height': msg.height,
                'k': list(msg.k), 'd': list(msg.d),
                'distortion_model': msg.distortion_model
            }
            with open(os.path.join(self.output_dir, 'depth_camera_info.json'), 'w') as f:
                json.dump(info, f, indent=2)
            self.d_info_saved = True

    def cb_c_info(self, msg):
        if not self.c_info_saved:
            info = {
                'width': msg.width, 'height': msg.height,
                'k': list(msg.k), 'd': list(msg.d),
                'distortion_model': msg.distortion_model
            }
            with open(os.path.join(self.output_dir, 'color_camera_info.json'), 'w') as f:
                json.dump(info, f, indent=2)
            self.c_info_saved = True

    def cb_lidar(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        fields_meta = [
            {'name': f.name, 'offset': int(f.offset), 'datatype': int(f.datatype), 'count': int(f.count)}
            for f in msg.fields
        ]
        is_be = bool(msg.is_bigendian)
        data_bytes = bytes(msg.data)
        item = ('lidar', self.lidar_idx, stamp, int(msg.point_step), fields_meta, is_be, data_bytes)
        self.lidar_idx += 1
        try:
            self.write_queue.put_nowait(item)
        except queue.Full:
            pass

    def cb_depth(self, msg):
        now = time.time()
        if now - self.last_depth_stamp < self.min_img_interval:
            return
        self.last_depth_stamp = now
        
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        d_img = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width).copy()
        item = ('depth', self.depth_idx, stamp, d_img)
        self.depth_idx += 1
        try:
            self.write_queue.put_nowait(item)
        except queue.Full:
            pass

    def cb_color(self, msg):
        now = time.time()
        if now - self.last_color_stamp < self.min_img_interval:
            return
        self.last_color_stamp = now
        
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if msg.encoding == 'rgb8':
            c_img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
            c_bgr = cv2.cvtColor(c_img, cv2.COLOR_RGB2RGB if False else cv2.COLOR_RGB2BGR)
        elif msg.encoding == 'bgr8':
            c_bgr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3).copy()
        else:
            c_bgr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)[:, :, :3].copy()
            
        item = ('color', self.color_idx, stamp, c_bgr)
        self.color_idx += 1
        try:
            self.write_queue.put_nowait(item)
        except queue.Full:
            pass

    def cb_odom(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        line_csv = f'{stamp:.6f},{p.x:.6f},{p.y:.6f},{p.z:.6f},{q.x:.6f},{q.y:.6f},{q.z:.6f},{q.w:.6f}\n'
        line_tum = f'{stamp:.6f} {p.x:.6f} {p.y:.6f} {p.z:.6f} {q.x:.6f} {q.y:.6f} {q.z:.6f} {q.w:.6f}\n'
        self.traj_file.write(line_csv)
        self.tum_file.write(line_tum)
        self.odom_cnt += 1

    def _writer_loop(self):
        lidar_index_entries = []
        depth_index_entries = []
        color_index_entries = []
        
        while self.running or not self.write_queue.empty():
            try:
                item = self.write_queue.get(timeout=0.1)
            except queue.Empty:
                continue
                
            kind = item[0]
            if kind == 'lidar':
                _, idx, stamp, pt_step, fields_meta, is_be, data_bytes = item
                fn = f'lidar_{idx:06d}.bin'
                fp = os.path.join(self.output_dir, 'lidar', fn)
                with open(fp, 'wb') as f:
                    f.write(data_bytes)
                lidar_index_entries.append({
                    'index': idx, 'stamp': stamp, 'file': fn,
                    'point_step': pt_step, 'is_bigendian': is_be, 'fields': fields_meta
                })
            elif kind == 'depth':
                _, idx, stamp, d_img = item
                fn = f'depth_{idx:06d}.png'
                fp = os.path.join(self.output_dir, 'depth', fn)
                cv2.imwrite(fp, d_img)
                depth_index_entries.append({'index': idx, 'stamp': stamp, 'file': fn})
            elif kind == 'color':
                _, idx, stamp, c_bgr = item
                fn = f'color_{idx:06d}.jpg'
                fp = os.path.join(self.output_dir, 'color', fn)
                cv2.imwrite(fp, c_bgr, [cv2.IMWRITE_JPEG_QUALITY, 90])
                color_index_entries.append({'index': idx, 'stamp': stamp, 'file': fn})
                
        # Write indices
        with open(os.path.join(self.output_dir, 'lidar_index.json'), 'w') as f:
            json.dump(lidar_index_entries, f)
        with open(os.path.join(self.output_dir, 'depth_index.json'), 'w') as f:
            json.dump(depth_index_entries, f)
        with open(os.path.join(self.output_dir, 'color_index.json'), 'w') as f:
            json.dump(color_index_entries, f)

    def close(self):
        self.running = False
        self.worker_thread.join()
        self.traj_file.close()
        self.tum_file.close()
        duration = time.time() - self.start_time
        meta = {
            'duration_s': round(duration, 2),
            'lidar_frames': self.lidar_idx,
            'depth_frames': self.depth_idx,
            'color_frames': self.color_idx,
            'odom_poses': self.odom_cnt,
            'lidar_rate_hz': round(self.lidar_idx / (duration + 1e-6), 2),
            'depth_rate_hz': round(self.depth_idx / (duration + 1e-6), 2),
            'color_rate_hz': round(self.color_idx / (duration + 1e-6), 2),
            'odom_rate_hz': round(self.odom_cnt / (duration + 1e-6), 2)
        }
        with open(os.path.join(self.output_dir, 'recorder_metadata.json'), 'w') as f:
            json.dump(meta, f, indent=2)
        return meta

def main():
    out_dir = sys.argv[1] if len(sys.argv) > 1 else '/home/scanar/scanarMini/diagnostics/m12_9_room_dataset'
    rclpy.init()
    node = LightweightCaptureRecorder(out_dir)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        meta = node.close()
        node.destroy_node()
        rclpy.shutdown()
        print('Recorder closed cleanly. Summary:', meta)

if __name__ == '__main__':
    main()
