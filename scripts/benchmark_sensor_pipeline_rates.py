#!/usr/bin/env python3
"""
High-Performance Multi-Threaded ROS 2 Stream Rate Benchmark
Independently measures Publisher Source Rates and M12.7 Ingestion Rates:
- /rslidar_points
- /rslidar_imu_data
- /aft_mapped_to_init
- /cloud_registered
- /camera/depth/image_raw
- /camera/color/image_raw
Computes:
- Total message counts
- True measured rate (Hz)
- Median dt (ms)
- P95 dt (ms)
- Min dt and Max dt (ms)
- Gaps (> 2x nominal dt)
"""

import os
import sys
import time
import json
import numpy as np
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import PointCloud2, Image, Imu
from nav_msgs.msg import Odometry

class StreamMonitor(Node):
    def __init__(self):
        super().__init__('m12_7_rate_benchmark_node')
        
        self.topics = [
            ('/rslidar_points', PointCloud2, 'rslidar_points', 10.0),
            ('/rslidar_imu_data', Imu, 'rslidar_imu_data', 200.0),
            ('/aft_mapped_to_init', Odometry, 'fastlivo_odom', 10.0),
            ('/cloud_registered', PointCloud2, 'cloud_registered', 10.0),
            ('/camera/depth/image_raw', Image, 'camera_depth', 30.0),
            ('/camera/color/image_raw', Image, 'camera_rgb', 30.0),
        ]
        
        self.timestamps = {k[2]: [] for k in self.topics}
        self.lock = threading.Lock()
        
        qos_best_effort = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=50)
        qos_reliable = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=50)

        self.subs = []
        for topic_name, msg_type, tag, nom_rate in self.topics:
            cb_grp = MutuallyExclusiveCallbackGroup()
            # Try best effort first
            sub = self.create_subscription(
                msg_type,
                topic_name,
                lambda msg, t=tag: self.record_arrival(t),
                qos_best_effort,
                callback_group=cb_grp
            )
            self.subs.append(sub)

    def record_arrival(self, tag):
        t = time.perf_counter()
        with self.lock:
            self.timestamps[tag].append(t)

def benchmark_rates(duration=10.0):
    rclpy.init()
    node = StreamMonitor()
    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    print(f"\n[BENCHMARK] Sampling all production topics for {duration:.1f} seconds using MultiThreadedExecutor (8 threads)...")
    time.sleep(duration)

    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()

    results = {}
    print("\n" + "="*85)
    print(f"{'Topic Name':<28} | {'Nominal':<7} | {'Count':<6} | {'Hz':<7} | {'Med dt':<8} | {'P95 dt':<8} | {'Gaps':<5}")
    print("="*85)

    for topic_name, _, tag, nom_hz in node.topics:
        ts = node.timestamps[tag]
        cnt = len(ts)
        if cnt > 1:
            diffs_ms = np.diff(ts) * 1000.0
            actual_duration = ts[-1] - ts[0]
            hz = float(cnt - 1) / actual_duration if actual_duration > 0 else 0.0
            med_dt = float(np.median(diffs_ms))
            p95_dt = float(np.percentile(diffs_ms, 95))
            min_dt = float(np.min(diffs_ms))
            max_dt = float(np.max(diffs_ms))
            nominal_dt = 1000.0 / nom_hz
            gaps = int(np.sum(diffs_ms > (nominal_dt * 2.0)))
        else:
            hz, med_dt, p95_dt, min_dt, max_dt, gaps = 0.0, 0.0, 0.0, 0.0, 0.0, 0

        results[topic_name] = {
            "tag": tag,
            "nominal_hz": nom_hz,
            "messages_received": cnt,
            "measured_rate_hz": round(hz, 2),
            "median_dt_ms": round(med_dt, 2),
            "p95_dt_ms": round(p95_dt, 2),
            "min_dt_ms": round(min_dt, 2),
            "max_dt_ms": round(max_dt, 2),
            "gap_count": gaps
        }

        print(f"{topic_name:<28} | {nom_hz:>5.1f}Hz | {cnt:>6} | {hz:>5.2f}Hz | {med_dt:>6.2f}ms | {p95_dt:>6.2f}ms | {gaps:>5}")

    print("="*85)
    with open("/home/scanar/scanarMini/data/sensor_rate_benchmark.json", 'w') as f:
        json.dump(results, f, indent=2)
    return results

if __name__ == '__main__':
    dur = float(sys.argv[1]) if len(sys.argv) > 1 else 10.0
    benchmark_rates(dur)
