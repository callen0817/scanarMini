#!/usr/bin/env python3
"""
M8.5 Rerun: High-Fidelity Gap Analysis & Stereo Frame Pairing Benchmark
Ultra-lightweight subscriber capturing pure ROS header timestamps without deserialization overhead.
Measures per-stream gap distributions (>1.5x, >2x, >3x nominal) and computes exact hardware stereo pairing metrics.
"""

import sys
import time
import numpy as np
import psutil
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, Imu, Image

class HighPrecisionGapBenchmarkNode(Node):
    def __init__(self, duration_sec=300.0):
        super().__init__('m8_5_gap_benchmark_node')
        self.duration_sec = duration_sec
        self.start_wall_time = None
        
        # Pure timestamp arrays (seconds as float64)
        self.stamps_lidar = []
        self.stamps_imu = []
        self.stamps_left_ir = []
        self.stamps_right_ir = []
        self.stamps_depth = []
        self.stamps_rgb = []
        
        self.cpu_samples = []
        self.ram_samples = []

        # High-throughput sensor QoS
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=100
        )

        # Ultra-lightweight callbacks: ONLY extract header stamp
        self.sub_lidar = self.create_subscription(PointCloud2, '/rslidar_points', self.cb_lidar, qos)
        self.sub_imu = self.create_subscription(Imu, '/rslidar_imu_data', self.cb_imu, qos)
        self.sub_left = self.create_subscription(Image, '/camera/left_ir/image_raw', self.cb_left, qos)
        self.sub_right = self.create_subscription(Image, '/camera/right_ir/image_raw', self.cb_right, qos)
        self.sub_depth = self.create_subscription(Image, '/camera/depth/image_raw', self.cb_depth, qos)
        self.sub_rgb = self.create_subscription(Image, '/camera/color/image_raw', self.cb_rgb, qos)

    def cb_lidar(self, msg: PointCloud2):
        if self.start_wall_time is None: self.start_wall_time = time.time()
        self.stamps_lidar.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)

    def cb_imu(self, msg: Imu):
        if self.start_wall_time is None: self.start_wall_time = time.time()
        self.stamps_imu.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)

    def cb_left(self, msg: Image):
        if self.start_wall_time is None: self.start_wall_time = time.time()
        self.stamps_left_ir.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)

    def cb_right(self, msg: Image):
        if self.start_wall_time is None: self.start_wall_time = time.time()
        self.stamps_right_ir.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)

    def cb_depth(self, msg: Image):
        if self.start_wall_time is None: self.start_wall_time = time.time()
        self.stamps_depth.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)

    def cb_rgb(self, msg: Image):
        if self.start_wall_time is None: self.start_wall_time = time.time()
        self.stamps_rgb.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)

def analyze_stream_gaps(stamps, name, nominal_hz):
    nominal_dt_ms = 1000.0 / nominal_hz
    n = len(stamps)
    if n < 5:
        return f"  {name:14s}: ❌ NO DATA ({n} frames)"
    
    stamps_arr = np.array(stamps)
    total_span_s = stamps_arr[-1] - stamps_arr[0]
    delivered_hz = (n - 1) / total_span_s if total_span_s > 0 else 0.0
    expected_count = int(total_span_s * nominal_hz)
    
    dt_ms = np.diff(stamps_arr) * 1000.0
    med_dt = np.median(dt_ms)
    p95_dt = np.percentile(dt_ms, 95)
    p99_dt = np.percentile(dt_ms, 99)
    max_dt = np.max(dt_ms)
    
    # Gap counts
    gaps_1_5x = np.count_nonzero(dt_ms > 1.5 * nominal_dt_ms)
    gaps_2_0x = np.count_nonzero(dt_ms > 2.0 * nominal_dt_ms)
    gaps_3_0x = np.count_nonzero(dt_ms > 3.0 * nominal_dt_ms)
    
    out = [
        f"\n--- [{name}] Nominal: {nominal_hz:4.1f} Hz ({nominal_dt_ms:.1f} ms) ---",
        f"  Total Frames:    {n:6d} (Expected ~{expected_count:6d} in {total_span_s:5.1f}s)",
        f"  Delivered Rate:  {delivered_hz:6.2f} Hz ({n/expected_count*100.0:.1f}% of nominal)",
        f"  Cadence (ms):    Median: {med_dt:5.1f} | P95: {p95_dt:5.1f} | P99: {p99_dt:5.1f} | Max: {max_dt:5.1f}",
        f"  Gaps > 1.5x:     {gaps_1_5x:5d} ({gaps_1_5x/len(dt_ms)*100.0:4.2f}%)",
        f"  Gaps > 2.0x:     {gaps_2_0x:5d} ({gaps_2_0x/len(dt_ms)*100.0:4.2f}%)",
        f"  Gaps > 3.0x:     {gaps_3_0x:5d} ({gaps_3_0x/len(dt_ms)*100.0:4.2f}%)"
    ]
    return "\n".join(out)

def analyze_stereo_pairing(left_stamps, right_stamps, max_pair_tol_ms=2.0):
    print("\n" + "="*80)
    print("                     STEREO HARDWARE PAIRING ANALYSIS")
    print("="*80)
    
    if len(left_stamps) == 0 or len(right_stamps) == 0:
        print("  ❌ Cannot analyze stereo pairing: One or both streams empty.")
        return
        
    left_arr = np.array(left_stamps)
    right_arr = np.array(right_stamps)
    
    # Two-pointer matching for paired frames within tolerance
    paired_deltas_ms = []
    r_idx = 0
    len_r = len(right_arr)
    unpaired_left = 0
    
    for l_t in left_arr:
        # Advance right pointer until right_t >= l_t - tol
        while r_idx < len_r and right_arr[r_idx] < l_t - (max_pair_tol_ms * 1e-3):
            r_idx += 1
        if r_idx < len_r:
            delta = abs(right_arr[r_idx] - l_t) * 1000.0
            if delta <= max_pair_tol_ms:
                paired_deltas_ms.append(delta)
            else:
                unpaired_left += 1
        else:
            unpaired_left += 1
            
    unpaired_right = len_r - len(paired_deltas_ms)
    
    print(f"  Total Left Frames:   {len(left_arr)}")
    print(f"  Total Right Frames:  {len(right_arr)}")
    print(f"  Matched Stereo Pairs:{len(paired_deltas_ms)} ({len(paired_deltas_ms)/len(left_arr)*100.0:.2f}%)")
    print(f"  Unpaired Left:       {unpaired_left}")
    print(f"  Unpaired Right:      {unpaired_right}")
    
    if paired_deltas_ms:
        deltas = np.array(paired_deltas_ms)
        print(f"\n  Left-Right Sync Offset (ms):")
        print(f"    Median Δt: {np.median(deltas):.3f} ms")
        print(f"    P95 Δt:    {np.percentile(deltas, 95):.3f} ms")
        print(f"    P99 Δt:    {np.percentile(deltas, 99):.3f} ms")
        print(f"    Max Δt:    {np.max(deltas):.3f} ms")
    print("="*80)

def main():
    rclpy.init()
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 300.0
    print("="*80)
    print(f"   M8.5 RERUN: FULL-STACK GAP & STEREO PAIRING QUALIFICATION ({duration/60:.1f} MIN)")
    print("="*80)
    
    node = HighPrecisionGapBenchmarkNode(duration)
    start_time = time.time()
    last_print = start_time
    
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            now = time.time()
            if node.start_wall_time is not None:
                elapsed = now - node.start_wall_time
                if elapsed >= duration:
                    break
                if now - last_print >= 30.0:
                    cpu = psutil.cpu_percent(interval=None)
                    ram = psutil.virtual_memory().percent
                    node.cpu_samples.append(cpu)
                    node.ram_samples.append(ram)
                    print(f"  [{elapsed:4.0f}/{duration:.0f}s] LiDAR: {len(node.stamps_lidar)} | IMU: {len(node.stamps_imu)} | "
                          f"Left: {len(node.stamps_left_ir)} | Right: {len(node.stamps_right_ir)} | RGB: {len(node.stamps_rgb)} | CPU: {cpu:.1f}%")
                    last_print = now
            else:
                if now - start_time > 15.0:
                    print("❌ Timeout: No sensor messages received.")
                    break
    except KeyboardInterrupt:
        print("\nBenchmark terminated by operator.")
        
    print("\n" + "="*80)
    print("                   PER-STREAM GAP ANALYSIS RESULTS")
    print("="*80)
    print(analyze_stream_gaps(node.stamps_lidar, "Airy LiDAR", 10.0))
    print(analyze_stream_gaps(node.stamps_imu, "Airy IMU", 200.0))
    print(analyze_stream_gaps(node.stamps_left_ir, "Left IR", 60.0))
    print(analyze_stream_gaps(node.stamps_right_ir, "Right IR", 60.0))
    print(analyze_stream_gaps(node.stamps_depth, "Depth", 60.0))
    print(analyze_stream_gaps(node.stamps_rgb, "RGB", 30.0))
    
    analyze_stereo_pairing(node.stamps_left_ir, node.stamps_right_ir)
    
    if node.cpu_samples:
        print(f"\nAverage Jetson CPU Load: {np.mean(node.cpu_samples):.1f}% ± {np.std(node.cpu_samples):.1f}% (Peak: {np.max(node.cpu_samples):.1f}%)")
        print(f"Average Memory Usage:    {np.mean(node.ram_samples):.1f}%\n")
        
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
