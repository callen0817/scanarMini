#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, Imu, Image
import time
import numpy as np
import sys

class M9ClockAlignmentVerifier(Node):
    def __init__(self, duration_sec=60):
        super().__init__('m9_clock_alignment_verifier')
        self.duration_sec = duration_sec
        self.start_time = time.time()
        
        self.lidar_records = []
        self.imu_records = []
        self.cam_records = []
        
        self.create_subscription(PointCloud2, '/rslidar_points', self.lidar_cb, qos_profile_sensor_data)
        self.create_subscription(Imu, '/rslidar_imu_data', self.imu_cb, qos_profile_sensor_data)
        self.create_subscription(Image, '/camera/left_ir/image_raw', self.cam_cb, qos_profile_sensor_data)
        
        self.get_logger().info(f"M9 Clock Alignment Verifier started. Listening for {duration_sec} s...")

    def stamp_to_sec(self, stamp):
        return stamp.sec + stamp.nanosec * 1e-9

    def lidar_cb(self, msg):
        recv_t = time.time()
        stamp_t = self.stamp_to_sec(msg.header.stamp)
        self.lidar_records.append((recv_t, stamp_t, recv_t - stamp_t))

    def imu_cb(self, msg):
        recv_t = time.time()
        stamp_t = self.stamp_to_sec(msg.header.stamp)
        self.imu_records.append((recv_t, stamp_t, recv_t - stamp_t))

    def cam_cb(self, msg):
        recv_t = time.time()
        stamp_t = self.stamp_to_sec(msg.header.stamp)
        self.cam_records.append((recv_t, stamp_t, recv_t - stamp_t))

def main():
    duration = int(sys.argv[1]) if len(sys.argv) > 1 else 30
    rclpy.init()
    node = M9ClockAlignmentVerifier(duration_sec=duration)
    
    t0 = time.time()
    while rclpy.ok() and (time.time() - t0) < duration:
        rclpy.spin_once(node, timeout_sec=0.1)
        
    print("\n" + "="*80)
    print(f" MILESTONE M9A: CLOCK ALIGNMENT & EPOCH STABILITY REPORT (Duration: {duration} s)")
    print("="*80)
    
    for name, records in [("Airy LiDAR (/rslidar_points)", node.lidar_records),
                          ("Airy IMU (/rslidar_imu_data)", node.imu_records),
                          ("Gemini Left IR (/camera/left_ir/image_raw)", node.cam_records)]:
        if len(records) < 10:
            print(f"  - {name}: INSUFFICIENT DATA ({len(records)} msgs)")
            continue
            
        recv_arr = np.array([r[0] for r in records])
        stamp_arr = np.array([r[1] for r in records])
        diff_arr = np.array([r[2] for r in records]) * 1000.0 # ms
        
        # Linear regression to compute drift rate
        t_span = recv_arr[-1] - recv_arr[0]
        drift_rate_ppm = 0.0
        if t_span > 1.0:
            slope, intercept = np.polyfit(recv_arr - recv_arr[0], diff_arr, 1) # ms/s
            drift_rate_ppm = slope * 1000.0 # us/s = ppm
            
        print(f"\n📊 {name}:")
        print(f"   • Total Samples:        {len(records)}")
        print(f"   • Mean Ingestion Latency (Host - Stamp): {np.mean(diff_arr):.2f} ms (Std: {np.std(diff_arr):.2f} ms)")
        print(f"   • Median Ingestion Latency:             {np.median(diff_arr):.2f} ms (Min: {np.min(diff_arr):.2f} ms, Max: {np.max(diff_arr):.2f} ms)")
        print(f"   • Measured Clock Drift Rate:            {drift_rate_ppm:.3f} ppm (us/s)")
        print(f"   • Total Relative Epoch Divergence:      {(diff_arr[-1] - diff_arr[0]):.3f} ms over {t_span:.1f} s")

    # Cross-sensor relative timestamp offset analysis
    if len(node.lidar_records) > 10 and len(node.cam_records) > 10:
        lidar_stamps = np.array([r[1] for r in node.lidar_records])
        cam_stamps = np.array([r[1] for r in node.cam_records])
        
        # For each lidar sweep, find nearest camera frame stamp
        offsets = []
        for ls in lidar_stamps:
            idx = np.argmin(np.abs(cam_stamps - ls))
            offsets.append((ls - cam_stamps[idx]) * 1000.0)
            
        offsets = np.array(offsets)
        print(f"\n🔄 LiDAR-Camera Relative Timestamp Phase (Nearest Match):")
        print(f"   • Median Offset: {np.median(offsets):.2f} ms")
        print(f"   • Mean Offset:   {np.mean(offsets):.2f} ms (Std: {np.std(offsets):.2f} ms)")
        print(f"   • Max Phase Gap: {np.max(np.abs(offsets)):.2f} ms (Expected < 8.3 ms at 60 FPS)")

    print("\n" + "="*80 + "\n")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
