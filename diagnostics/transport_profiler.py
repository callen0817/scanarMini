#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time
import numpy as np
from sensor_msgs.msg import PointCloud2, Image, Imu
from nav_msgs.msg import Odometry

class TransportProfiler(Node):
    def __init__(self):
        super().__init__('transport_profiler')
        
        self.stats = {
            'lidar': {'count': 0, 'last_recv': None, 'gaps': [], 'header_stamps': [], 'recv_stamps': []},
            'imu': {'count': 0, 'last_recv': None, 'gaps': [], 'header_stamps': [], 'recv_stamps': []},
            'depth': {'count': 0, 'last_recv': None, 'gaps': [], 'header_stamps': [], 'recv_stamps': []},
            'color': {'count': 0, 'last_recv': None, 'gaps': [], 'header_stamps': [], 'recv_stamps': []},
            'odom': {'count': 0, 'last_recv': None, 'gaps': [], 'header_stamps': [], 'recv_stamps': []},
        }
        
        self.create_subscription(PointCloud2, '/rslidar_points', self.cb_lidar, 10)
        self.create_subscription(Imu, '/rslidar_imu_data', self.cb_imu, 200)
        self.create_subscription(Image, '/camera/depth/image_raw', self.cb_depth, 30)
        self.create_subscription(Image, '/camera/color/image_raw', self.cb_color, 30)
        self.create_subscription(Odometry, '/aft_mapped_to_init', self.cb_odom, 10)
        
        self.timer = self.create_timer(5.0, self.report_progress)
        self.start_time = time.time()
        self.get_logger().info('TransportProfiler initialized and listening to streams...')

    def _update_stream(self, key, msg):
        now = time.time()
        s = self.stats[key]
        s['count'] += 1
        if s['last_recv'] is not None:
            s['gaps'].append((now - s['last_recv']) * 1000.0)
        s['last_recv'] = now
        
        h_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        s['header_stamps'].append(h_stamp)
        s['recv_stamps'].append(now)

    def cb_lidar(self, msg): self._update_stream('lidar', msg)
    def cb_imu(self, msg): self._update_stream('imu', msg)
    def cb_depth(self, msg): self._update_stream('depth', msg)
    def cb_color(self, msg): self._update_stream('color', msg)
    def cb_odom(self, msg): self._update_stream('odom', msg)

    def report_progress(self):
        elapsed = time.time() - self.start_time
        if elapsed > 0:
            self.get_logger().info(
                f'[{elapsed:4.1f}s] '
                f'LiDAR: {self.stats["lidar"]["count"]/elapsed:4.1f} Hz ({self.stats["lidar"]["count"]} cnt) | '
                f'IMU: {self.stats["imu"]["count"]/elapsed:5.1f} Hz ({self.stats["imu"]["count"]} cnt) | '
                f'Depth: {self.stats["depth"]["count"]/elapsed:4.1f} Hz ({self.stats["depth"]["count"]} cnt) | '
                f'Color: {self.stats["color"]["count"]/elapsed:4.1f} Hz ({self.stats["color"]["count"]} cnt) | '
                f'Odom: {self.stats["odom"]["count"]/elapsed:4.1f} Hz ({self.stats["odom"]["count"]} cnt)'
            )

def main():
    rclpy.init()
    node = TransportProfiler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
