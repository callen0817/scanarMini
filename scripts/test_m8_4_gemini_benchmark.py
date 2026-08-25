#!/usr/bin/env python3
"""
M8.4: Orbbec Gemini 336L Multi-Stream USB 3.2 Capability Benchmark
Measures actual published frame rates, cadence jitter, valid depth %, and system load
across multiple multi-stream candidate profiles.
"""

import sys
import time
import math
import numpy as np
import psutil
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

class GeminiBenchmarkNode(Node):
    def __init__(self, candidate_name="Candidate", duration_sec=20.0):
        super().__init__('m8_gemini_benchmark_node')
        self.candidate_name = candidate_name
        self.duration_sec = duration_sec
        self.start_time = None
        
        # Stream metrics
        self.rgb_stamps = []
        self.depth_stamps = []
        self.left_ir_stamps = []
        self.right_ir_stamps = []
        self.depth_valid_ratios = []

        # Subscriptions
        self.sub_rgb = self.create_subscription(Image, '/camera/color/image_raw', self.rgb_cb, qos_profile_sensor_data)
        self.sub_depth = self.create_subscription(Image, '/camera/depth/image_raw', self.depth_cb, qos_profile_sensor_data)
        self.sub_left = self.create_subscription(Image, '/camera/left_ir/image_raw', self.left_cb, qos_profile_sensor_data)
        self.sub_right = self.create_subscription(Image, '/camera/right_ir/image_raw', self.right_cb, qos_profile_sensor_data)

    def rgb_cb(self, msg: Image):
        now = time.time()
        if self.start_time is None:
            self.start_time = now
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.rgb_stamps.append(stamp)

    def depth_cb(self, msg: Image):
        now = time.time()
        if self.start_time is None:
            self.start_time = now
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.depth_stamps.append(stamp)
        
        # Calculate valid depth percentage (sample 1 in 10 frames to reduce CPU load)
        if len(self.depth_stamps) % 10 == 1 and msg.data:
            try:
                # 16UC1 format
                depth_arr = np.frombuffer(msg.data, dtype=np.uint16)
                valid_cnt = np.count_nonzero(depth_arr > 0)
                ratio = valid_cnt / len(depth_arr) * 100.0
                self.depth_valid_ratios.append(ratio)
            except Exception:
                pass

    def left_cb(self, msg: Image):
        now = time.time()
        if self.start_time is None:
            self.start_time = now
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.left_ir_stamps.append(stamp)

    def right_cb(self, msg: Image):
        now = time.time()
        if self.start_time is None:
            self.start_time = now
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.right_ir_stamps.append(stamp)

def calculate_stream_stats(stamps, name):
    if len(stamps) < 2:
        return f"  {name:10s}: ❌ NO DATA ({len(stamps)} frames)"
    dur = stamps[-1] - stamps[0]
    hz = (len(stamps) - 1) / dur if dur > 0 else 0.0
    dt_ms = np.diff(stamps) * 1000.0
    median_dt = np.median(dt_ms)
    p95_dt = np.percentile(dt_ms, 95)
    p99_dt = np.percentile(dt_ms, 99)
    return (f"  {name:10s}: {hz:5.1f} FPS | Median Δt: {median_dt:5.1f} ms | "
            f"P95: {p95_dt:5.1f} ms | Frames: {len(stamps)} in {dur:4.1f}s")

def benchmark_profile(candidate_name, duration_sec=20.0):
    print(f"\nEvaluating: {candidate_name} (Sampling for {duration_sec:.0f}s)...")
    node = GeminiBenchmarkNode(candidate_name, duration_sec)
    
    start_wall = time.time()
    cpu_samples = []
    
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.05)
        now = time.time()
        if node.start_time is not None:
            if now - node.start_time >= duration_sec:
                break
        else:
            if now - start_wall > 12.0:
                print("  ⚠️ Warning: Timeout waiting for camera frames.")
                break
        if int(now * 2) % 2 == 0:
            cpu_samples.append(psutil.cpu_percent(interval=None))
            
    print(calculate_stream_stats(node.rgb_stamps, "RGB"))
    print(calculate_stream_stats(node.depth_stamps, "Depth"))
    print(calculate_stream_stats(node.left_ir_stamps, "Left IR"))
    print(calculate_stream_stats(node.right_ir_stamps, "Right IR"))
    
    if node.depth_valid_ratios:
        print(f"  Valid Depth %: {np.mean(node.depth_valid_ratios):.1f}% ± {np.std(node.depth_valid_ratios):.1f}%")
    if cpu_samples:
        print(f"  CPU Utilization: {np.mean(cpu_samples):.1f}% (Jetson system load)")
    node.destroy_node()

def main():
    rclpy.init()
    try:
        duration = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0
        name = sys.argv[2] if len(sys.argv) > 2 else "Active Candidate"
        benchmark_profile(name, duration)
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
