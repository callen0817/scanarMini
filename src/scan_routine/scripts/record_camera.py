#!/usr/bin/env python3
import os
import sys
import json
import time
import queue
import threading
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo

class CameraRecorder(Node):
    def __init__(self, out_dir):
        super().__init__('camera_recorder')
        self.out_dir = out_dir
        os.makedirs(os.path.join(out_dir, 'depth'), exist_ok=True)
        os.makedirs(os.path.join(out_dir, 'color'), exist_ok=True)
        
        self.write_q = queue.Queue(maxsize=300)
        self.running = True
        self.worker = threading.Thread(target=self._writer, daemon=True)
        self.worker.start()
        
        self.last_depth = 0.0
        self.last_color = 0.0
        self.min_dt = 0.12 # ~8 Hz keyframes
        
        self.d_idx = 0
        self.c_idx = 0
        self.d_info_saved = False
        self.c_info_saved = False
        
        qos_sensor = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)
        qos_reliable = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)
        
        self.create_subscription(Image, '/camera/depth/image_raw', self.cb_depth, qos_sensor)
        self.create_subscription(Image, '/camera/color/image_raw', self.cb_color, qos_sensor)
        self.create_subscription(CameraInfo, '/camera/depth/camera_info', self.cb_d_info, qos_reliable)
        self.create_subscription(CameraInfo, '/camera/color/camera_info', self.cb_c_info, qos_reliable)

    def cb_d_info(self, msg):
        if not self.d_info_saved:
            info = {'width': msg.width, 'height': msg.height, 'k': list(msg.k), 'd': list(msg.d), 'distortion_model': msg.distortion_model}
            with open(os.path.join(self.out_dir, 'depth_camera_info.json'), 'w') as f:
                json.dump(info, f, indent=2)
            self.d_info_saved = True

    def cb_c_info(self, msg):
        if not self.c_info_saved:
            info = {'width': msg.width, 'height': msg.height, 'k': list(msg.k), 'd': list(msg.d), 'distortion_model': msg.distortion_model}
            with open(os.path.join(self.out_dir, 'color_camera_info.json'), 'w') as f:
                json.dump(info, f, indent=2)
            self.c_info_saved = True

    def cb_depth(self, msg):
        now = time.time()
        if now - self.last_depth < self.min_dt:
            return
        self.last_depth = now
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        d_img = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width).copy()
        try:
            self.write_q.put_nowait(('depth', self.d_idx, stamp, d_img))
            self.d_idx += 1
        except queue.Full:
            pass

    def cb_color(self, msg):
        now = time.time()
        if now - self.last_color < self.min_dt:
            return
        self.last_color = now
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if msg.encoding == 'rgb8':
            c_img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
            c_bgr = cv2.cvtColor(c_img, cv2.COLOR_RGB2BGR)
        else:
            c_bgr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3).copy()
        try:
            self.write_q.put_nowait(('color', self.c_idx, stamp, c_bgr))
            self.c_idx += 1
        except queue.Full:
            pass

    def _writer(self):
        d_entries = []
        c_entries = []
        last_flush = time.time()
        while self.running or not self.write_q.empty():
            try:
                item = self.write_q.get(timeout=0.1)
            except queue.Empty:
                continue
            kind = item[0]
            if kind == 'depth':
                _, idx, stamp, img = item
                fn = f'depth_{idx:06d}.png'
                cv2.imwrite(os.path.join(self.out_dir, 'depth', fn), img)
                d_entries.append({'index': idx, 'stamp': stamp, 'file': fn})
            elif kind == 'color':
                _, idx, stamp, img = item
                fn = f'color_{idx:06d}.jpg'
                cv2.imwrite(os.path.join(self.out_dir, 'color', fn), img, [cv2.IMWRITE_JPEG_QUALITY, 90])
                c_entries.append({'index': idx, 'stamp': stamp, 'file': fn})
                
            if time.time() - last_flush > 2.0:
                with open(os.path.join(self.out_dir, 'depth_index.json'), 'w') as f:
                    json.dump(d_entries, f)
                with open(os.path.join(self.out_dir, 'color_index.json'), 'w') as f:
                    json.dump(c_entries, f)
                last_flush = time.time()
                
        with open(os.path.join(self.out_dir, 'depth_index.json'), 'w') as f:
            json.dump(d_entries, f)
        with open(os.path.join(self.out_dir, 'color_index.json'), 'w') as f:
            json.dump(c_entries, f)

    def close(self):
        self.running = False
        self.worker.join()

def main():
    out_dir = sys.argv[1] if len(sys.argv) > 1 else '/tmp/test_rec'
    rclpy.init()
    node = CameraRecorder(out_dir)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
