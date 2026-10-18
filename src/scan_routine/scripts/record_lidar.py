#!/usr/bin/env python3
import os
import sys
import json
import time
import queue
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry

class LidarRecorder(Node):
    def __init__(self, out_dir, require_active_odom=True):
        super().__init__('lidar_recorder')
        self.out_dir = out_dir
        self.require_active_odom = require_active_odom
        os.makedirs(os.path.join(out_dir, 'lidar'), exist_ok=True)
        self.traj_file = open(os.path.join(out_dir, 'trajectory.csv'), 'w')
        self.traj_file.write('timestamp,x,y,z,qx,qy,qz,qw\n')
        self.tum_file = open(os.path.join(out_dir, 'trajectory.tum'), 'w')
        self.entries = []
        self.idx = 0
        self.odom_cnt = 0
        self.last_odom_wall_time = 0.0

        self.write_q = queue.Queue(maxsize=500)
        self.running = True
        self.worker = threading.Thread(target=self._writer, daemon=True)
        self.worker.start()

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=50)
        self.create_subscription(PointCloud2, '/rslidar_points', self.cb_lidar, qos)
        self.create_subscription(Odometry, '/aft_mapped_to_init', self.cb_odom, qos)

    def cb_lidar(self, msg):
        # Gate LiDAR disk writes on active SLAM odometry to prevent unindexed .bin bloat
        if self.require_active_odom and (time.time() - self.last_odom_wall_time > 2.0):
            return

        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        fields_meta = [{
            'name': f.name,
            'offset': int(f.offset),
            'datatype': int(f.datatype),
            'count': int(f.count)
        } for f in msg.fields]

        try:
            self.write_q.put_nowait((
                'lidar',
                self.idx,
                stamp,
                int(msg.point_step),
                bool(msg.is_bigendian),
                fields_meta,
                bytes(msg.data)
            ))
            self.idx += 1
        except queue.Full:
            pass

    def cb_odom(self, msg):
        self.last_odom_wall_time = time.time()
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        try:
            self.write_q.put_nowait(('odom', stamp, p.x, p.y, p.z, q.x, q.y, q.z, q.w))
            self.odom_cnt += 1
        except queue.Full:
            pass

    def _writer(self):
        sweeps_since_flush = 0
        while self.running or not self.write_q.empty():
            try:
                item = self.write_q.get(timeout=0.1)
            except queue.Empty:
                continue

            kind = item[0]
            if kind == 'lidar':
                _, idx, stamp, pt_step, is_be, fields_meta, raw_bytes = item
                fn = f'lidar_{idx:06d}.bin'
                fp = os.path.join(self.out_dir, 'lidar', fn)
                with open(fp, 'wb') as f:
                    f.write(raw_bytes)
                self.entries.append({
                    'index': idx,
                    'stamp': stamp,
                    'file': fn,
                    'point_step': pt_step,
                    'is_bigendian': is_be,
                    'fields': fields_meta
                })
                sweeps_since_flush += 1
                if sweeps_since_flush >= 20:
                    self._flush_index()
                    sweeps_since_flush = 0
            elif kind == 'odom':
                _, stamp, px, py, pz, qx, qy, qz, qw = item
                self.traj_file.write(f'{stamp:.6f},{px:.6f},{py:.6f},{pz:.6f},{qx:.6f},{qy:.6f},{qz:.6f},{qw:.6f}\n')
                self.tum_file.write(f'{stamp:.6f} {px:.6f} {py:.6f} {pz:.6f} {qx:.6f} {qy:.6f} {qz:.6f} {qw:.6f}\n')

        self.traj_file.flush()
        self.tum_file.flush()
        self._flush_index()

    def _flush_index(self):
        tmp_p = os.path.join(self.out_dir, 'lidar_index.tmp')
        final_p = os.path.join(self.out_dir, 'lidar_index.json')
        with open(tmp_p, 'w') as f:
            json.dump(self.entries, f)
        os.replace(tmp_p, final_p)

    def close(self):
        self.running = False
        self.worker.join()
        self.traj_file.close()
        self.tum_file.close()

def main():
    out_dir = sys.argv[1] if len(sys.argv) > 1 else '/tmp/test_rec'
    require_odom = '--allow-unaligned' not in sys.argv
    rclpy.init()
    node = LidarRecorder(out_dir, require_active_odom=require_odom)
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
