#!/usr/bin/env python3
"""
M8.5: Full-Stack Concurrent Sensor Throughput Benchmark (5-10 Minute Qualification)
Measures simultaneous performance of Airy LiDAR + Airy IMU + Gemini Left IR + Right IR + Depth + RGB.
Records delivered Hz, cadence jitter, valid depth %, network interface stats, CPU/RAM utilization, and dropped messages.
"""

import sys
import time
import subprocess
import numpy as np
import psutil
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, Imu, Image

class FullStackThroughputNode(Node):
    def __init__(self, target_duration_sec=300.0):
        super().__init__('m8_5_throughput_node')
        self.target_duration_sec = target_duration_sec
        self.start_wall_time = None
        
        # Stream Timestamps
        self.lidar_stamps = []
        self.lidar_point_counts = []
        self.imu_stamps = []
        self.rgb_stamps = []
        self.depth_stamps = []
        self.left_ir_stamps = []
        self.right_ir_stamps = []
        self.depth_valid_ratios = []

        # System stats
        self.cpu_samples = []
        self.ram_samples = []

        # Subscriptions
        self.sub_lidar = self.create_subscription(PointCloud2, '/rslidar_points', self.lidar_cb, qos_profile_sensor_data)
        self.sub_imu = self.create_subscription(Imu, '/rslidar_imu_data', self.imu_cb, qos_profile_sensor_data)
        self.sub_rgb = self.create_subscription(Image, '/camera/color/image_raw', self.rgb_cb, qos_profile_sensor_data)
        self.sub_depth = self.create_subscription(Image, '/camera/depth/image_raw', self.depth_cb, qos_profile_sensor_data)
        self.sub_left = self.create_subscription(Image, '/camera/left_ir/image_raw', self.left_cb, qos_profile_sensor_data)
        self.sub_right = self.create_subscription(Image, '/camera/right_ir/image_raw', self.right_cb, qos_profile_sensor_data)

    def lidar_cb(self, msg: PointCloud2):
        now = time.time()
        if self.start_wall_time is None:
            self.start_wall_time = now
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.lidar_stamps.append(stamp)
        self.lidar_point_counts.append(msg.width * msg.height)

    def imu_cb(self, msg: Imu):
        now = time.time()
        if self.start_wall_time is None:
            self.start_wall_time = now
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.imu_stamps.append(stamp)

    def rgb_cb(self, msg: Image):
        now = time.time()
        if self.start_wall_time is None:
            self.start_wall_time = now
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.rgb_stamps.append(stamp)

    def depth_cb(self, msg: Image):
        now = time.time()
        if self.start_wall_time is None:
            self.start_wall_time = now
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.depth_stamps.append(stamp)
        
        if len(self.depth_stamps) % 30 == 1 and msg.data:
            try:
                arr = np.frombuffer(msg.data, dtype=np.uint16)
                valid = np.count_nonzero(arr > 0)
                self.depth_valid_ratios.append(valid / len(arr) * 100.0)
            except Exception:
                pass

    def left_cb(self, msg: Image):
        now = time.time()
        if self.start_wall_time is None:
            self.start_wall_time = now
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.left_ir_stamps.append(stamp)

    def right_cb(self, msg: Image):
        now = time.time()
        if self.start_wall_time is None:
            self.start_wall_time = now
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.right_ir_stamps.append(stamp)

def get_net_stats(interface="enP8p1s0"):
    try:
        out = subprocess.check_output(f"ip -s link show {interface}", shell=True).decode()
        lines = out.strip().split("\n")
        rx_line = lines[3].split()
        tx_line = lines[5].split()
        rx_bytes, rx_packets, rx_errs, rx_drop = int(rx_line[0]), int(rx_line[1]), int(rx_line[2]), int(rx_line[3])
        tx_bytes, tx_packets, tx_errs, tx_drop = int(tx_line[0]), int(tx_line[1]), int(tx_line[2]), int(tx_line[3])
        return {
            'rx_bytes': rx_bytes, 'rx_packets': rx_packets, 'rx_errs': rx_errs, 'rx_drop': rx_drop,
            'tx_bytes': tx_bytes, 'tx_packets': tx_packets, 'tx_errs': tx_errs, 'tx_drop': tx_drop
        }
    except Exception as e:
        return {}

def format_stream_stat(stamps, name, expected_hz):
    if len(stamps) < 5:
        return f"  {name:12s}: ❌ NO DATA ({len(stamps)} received)"
    stamps_arr = np.array(stamps)
    dur = stamps_arr[-1] - stamps_arr[0]
    hz = (len(stamps_arr) - 1) / dur if dur > 0 else 0.0
    dt_ms = np.diff(stamps_arr) * 1000.0
    med_dt = np.median(dt_ms)
    p95_dt = np.percentile(dt_ms, 95)
    p99_dt = np.percentile(dt_ms, 99)
    status = "✅ PASS" if (0.85 * expected_hz <= hz <= 1.15 * expected_hz) else "⚠️ WARNING"
    return (f"  {name:12s}: {hz:6.2f} Hz (Exp: {expected_hz:4.1f}) | Median Δt: {med_dt:5.1f} ms | "
            f"P95: {p95_dt:5.1f} ms | Count: {len(stamps):6d} | {status}")

def run_benchmark(duration_sec=300.0):
    print("="*85)
    print(f"      M8.5 FULL-STACK CONCURRENT SENSOR THROUGHPUT QUALIFICATION ({duration_sec/60:.1f} MIN)")
    print("="*85)
    
    net_start = get_net_stats("enP8p1s0")
    node = FullStackThroughputNode(duration_sec)
    
    start_time = time.time()
    last_print = start_time
    
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            now = time.time()
            
            if node.start_wall_time is not None:
                elapsed = now - node.start_wall_time
                if elapsed >= duration_sec:
                    break
                if now - last_print >= 30.0:
                    cpu = psutil.cpu_percent(interval=None)
                    ram = psutil.virtual_memory().percent
                    node.cpu_samples.append(cpu)
                    node.ram_samples.append(ram)
                    print(f"  [Progress {elapsed:4.0f}/{duration_sec:.0f}s] LiDAR: {len(node.lidar_stamps)} | "
                          f"IMU: {len(node.imu_stamps)} | Stereo: {len(node.left_ir_stamps)} | "
                          f"RGB: {len(node.rgb_stamps)} | CPU: {cpu:.1f}% | RAM: {ram:.1f}%")
                    last_print = now
            else:
                if now - start_time > 15.0:
                    print("❌ Timeout: Did not receive messages from all sensors within 15 seconds.")
                    break
    except KeyboardInterrupt:
        print("\nBenchmark interrupted by operator.")
        
    net_end = get_net_stats("enP8p1s0")
    total_dur = (node.lidar_stamps[-1] - node.lidar_stamps[0]) if len(node.lidar_stamps) > 1 else (time.time() - start_time)
    
    print("\n" + "="*85)
    print("                             M8.5 BENCHMARK RESULTS")
    print("="*85)
    print(format_stream_stat(node.lidar_stamps, "Airy LiDAR", 10.0))
    print(format_stream_stat(node.imu_stamps, "Airy IMU", 200.0))
    print(format_stream_stat(node.left_ir_stamps, "Left IR (848x480)", 60.0))
    print(format_stream_stat(node.right_ir_stamps, "Right IR (848x480)", 60.0))
    print(format_stream_stat(node.depth_stamps, "Depth (848x480)", 60.0))
    print(format_stream_stat(node.rgb_stamps, "RGB (1280x800)", 30.0))
    
    if node.depth_valid_ratios:
        print(f"\n  Active Depth Validity: {np.mean(node.depth_valid_ratios):.2f}% ± {np.std(node.depth_valid_ratios):.2f}%")
    if node.lidar_point_counts:
        print(f"  Mean Point Density:    {np.mean(node.lidar_point_counts):.0f} pts/sweep")
        
    if net_start and net_end:
        rx_bytes_diff = net_end['rx_bytes'] - net_start['rx_bytes']
        rx_pkts_diff = net_end['rx_packets'] - net_start['rx_packets']
        rx_mbps = (rx_bytes_diff * 8) / (total_dur * 1e6) if total_dur > 0 else 0.0
        rx_errs = net_end['rx_errs'] - net_start['rx_errs']
        rx_drop = net_end['rx_drop'] - net_start['rx_drop']
        print("\n--- Dedicated Ethernet Interface Telemetry (enP8p1s0) ---")
        print(f"  Throughput:            {rx_mbps:.2f} Mbps (Bandwidth Utilization on 100BASE-TX: {rx_mbps:.1f}%)")
        print(f"  Packets Transferred:   {rx_pkts_diff} pkts ({rx_pkts_diff/total_dur:.1f} pkts/s)")
        print(f"  Interface Drops:       {rx_drop} (Target: 0)")
        print(f"  Interface Errors:      {rx_errs} (Target: 0)")
        
    if node.cpu_samples:
        print("\n--- System Compute Load (Jetson Orin) ---")
        print(f"  Average CPU Load:      {np.mean(node.cpu_samples):.1f}% ± {np.std(node.cpu_samples):.1f}%")
        print(f"  Peak CPU Load:         {np.max(node.cpu_samples):.1f}%")
        print(f"  System Memory (RAM):   {np.mean(node.ram_samples):.1f}%")
    print("="*85 + "\n")
    node.destroy_node()

def main():
    rclpy.init()
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 300.0
    try:
        run_benchmark(duration)
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
