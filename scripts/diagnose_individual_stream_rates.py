#!/usr/bin/env python3
"""
Diagnoses Individual ROS 2 Stream Rates in Isolation
Subscribes to each topic individually for 5 seconds to eliminate inter-topic callback contention
and measure pure publisher output rates.
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

from sensor_msgs.msg import PointCloud2, Image, Imu
from nav_msgs.msg import Odometry

TOPICS_TO_TEST = [
    ('/rslidar_imu_data', Imu, 'IMU Stream', 200.0),
    ('/rslidar_points', PointCloud2, 'Raw Airy LiDAR Points', 10.0),
    ('/camera/color/image_raw', Image, 'Gemini Color RGB Stream', 30.0),
    ('/camera/depth/image_raw', Image, 'Gemini Depth Stream', 30.0),
    ('/aft_mapped_to_init', Odometry, 'FAST-LIVO2 Odometry', 10.0),
    ('/cloud_registered', PointCloud2, 'FAST-LIVO2 Registered Cloud', 10.0),
]

class SingleTopicListener(Node):
    def __init__(self, topic_name, msg_type):
        super().__init__('single_topic_listener')
        self.timestamps = []
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=100)
        self.sub = self.create_subscription(msg_type, topic_name, self.cb, qos)

    def cb(self, msg):
        self.timestamps.append(time.perf_counter())

def test_single_topic(topic_name, msg_type, tag, nom_hz, duration=5.0):
    node = SingleTopicListener(topic_name, msg_type)
    t_start = time.time()
    while (time.time() - t_start) < duration:
        rclpy.spin_once(node, timeout_sec=0.01)

    node.destroy_node()
    ts = node.timestamps
    cnt = len(ts)
    if cnt > 1:
        diffs_ms = np.diff(ts) * 1000.0
        dur_actual = ts[-1] - ts[0]
        hz = float(cnt - 1) / dur_actual if dur_actual > 0 else 0.0
        med_dt = float(np.median(diffs_ms))
        p95_dt = float(np.percentile(diffs_ms, 95))
        min_dt = float(np.min(diffs_ms))
        max_dt = float(np.max(diffs_ms))
        gaps = int(np.sum(diffs_ms > (1000.0 / nom_hz * 2.0)))
    else:
        hz, med_dt, p95_dt, min_dt, max_dt, gaps = 0.0, 0.0, 0.0, 0.0, 0.0, 0

    return {
        "topic": topic_name,
        "tag": tag,
        "nominal_hz": nom_hz,
        "count": cnt,
        "isolated_hz": round(hz, 2),
        "med_dt_ms": round(med_dt, 2),
        "p95_dt_ms": round(p95_dt, 2),
        "min_dt_ms": round(min_dt, 2),
        "max_dt_ms": round(max_dt, 2),
        "gap_count": gaps
    }

def main():
    rclpy.init()
    print("="*95)
    print("             ISOLATED PER-TOPIC PUBLISHER RATE DIAGNOSTIC            ")
    print("="*95)
    print(f"{'Topic Name':<28} | {'Nominal':<7} | {'Count':<6} | {'Isolated Hz':<11} | {'Med dt':<8} | {'P95 dt':<8} | {'Gaps':<5}")
    print("="*95)

    all_results = {}
    for top, mtype, tag, nom in TOPICS_TO_TEST:
        res = test_single_topic(top, mtype, tag, nom, duration=5.0)
        all_results[top] = res
        print(f"{top:<28} | {nom:>5.1f}Hz | {res['count']:>6} | {res['isolated_hz']:>9.2f}Hz | {res['med_dt_ms']:>6.2f}ms | {res['p95_dt_ms']:>6.2f}ms | {res['gap_count']:>5}")

    print("="*95)
    rclpy.shutdown()

    with open("/home/scanar/scanarMini/data/isolated_topic_rates.json", 'w') as f:
        json.dump(all_results, f, indent=2)

if __name__ == '__main__':
    main()
