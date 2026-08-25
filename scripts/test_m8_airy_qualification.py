#!/usr/bin/env python3
"""
M8.1 & M8.2: Physical Sensor Qualification Script for RoboSense Airy LiDAR & Native IMU
Runs standalone telemetry collection for >= 60 seconds.
Measures LiDAR frame rate and exact IMU statistics (mean, median, jitter, gaps, gravity vector).
"""

import sys
import time
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, Imu

class AiryQualificationNode(Node):
    def __init__(self, target_duration_sec=65.0):
        super().__init__('m8_airy_qualification_node')
        self.target_duration_sec = target_duration_sec
        self.start_time = None
        
        # LiDAR metrics
        self.lidar_msg_count = 0
        self.lidar_timestamps = []
        self.lidar_point_counts = []
        self.lidar_x_bounds = []
        self.lidar_y_bounds = []
        self.lidar_z_bounds = []
        
        # IMU metrics
        self.imu_msg_count = 0
        self.imu_wall_times = []
        self.imu_stamps_sec = []
        self.accel_x_list = []
        self.accel_y_list = []
        self.accel_z_list = []
        self.gyro_x_list = []
        self.gyro_y_list = []
        self.gyro_z_list = []

        self.sub_lidar = self.create_subscription(
            PointCloud2,
            '/rslidar_points',
            self.lidar_callback,
            qos_profile_sensor_data
        )

        self.sub_imu = self.create_subscription(
            Imu,
            '/rslidar_imu_data',
            self.imu_callback,
            qos_profile_sensor_data
        )
        
        self.get_logger().info(f"[M8 QUALIFICATION] Listening to /rslidar_points and /rslidar_imu_data for {self.target_duration_sec:.1f}s...")

    def lidar_callback(self, msg: PointCloud2):
        now = time.time()
        if self.start_time is None:
            self.start_time = now
            
        self.lidar_msg_count += 1
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.lidar_timestamps.append(stamp)
        num_points = msg.width * msg.height
        self.lidar_point_counts.append(num_points)
        
        if self.lidar_msg_count % 50 == 1:
            self.get_logger().info(f"[M8.1 LiDAR] Sweep #{self.lidar_msg_count} | {num_points} points | frame_id='{msg.header.frame_id}' | stamp={stamp:.3f}")

    def imu_callback(self, msg: Imu):
        now = time.time()
        if self.start_time is None:
            self.start_time = now

        self.imu_msg_count += 1
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        self.imu_wall_times.append(now)
        self.imu_stamps_sec.append(stamp)
        
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        self.accel_x_list.append(ax)
        self.accel_y_list.append(ay)
        self.accel_z_list.append(az)
        
        gx = msg.angular_velocity.x
        gy = msg.angular_velocity.y
        gz = msg.angular_velocity.z
        self.gyro_x_list.append(gx)
        self.gyro_y_list.append(gy)
        self.gyro_z_list.append(gz)
        
        if self.imu_msg_count % 500 == 1:
            amag = math.sqrt(ax*ax + ay*ay + az*az)
            self.get_logger().info(f"[M8.2 IMU] Sample #{self.imu_msg_count} | a=[{ax:+.2f}, {ay:+.2f}, {az:+.2f}] |a|={amag:.2f} m/s² | w=[{gx:+.3f}, {gy:+.3f}, {gz:+.3f}] rad/s")

def analyze_and_report(node: AiryQualificationNode):
    print("\n" + "="*80)
    print("           SCANAR MINI PHYSICAL SENSOR QUALIFICATION REPORT (M8.1 & M8.2)")
    print("="*80)
    
    # -------------------------------------------------------------
    # M8.1: LiDAR Analysis
    # -------------------------------------------------------------
    print("\n--- [M8.1: RoboSense Airy LiDAR Telemetry] ---")
    if node.lidar_msg_count == 0:
        print("❌ FAILED: Zero /rslidar_points messages received.")
    else:
        l_dur = node.lidar_timestamps[-1] - node.lidar_timestamps[0] if len(node.lidar_timestamps) > 1 else 0.0
        l_hz = (node.lidar_msg_count - 1) / l_dur if l_dur > 0 else 0.0
        l_dt = np.diff(node.lidar_timestamps) * 1000.0 if len(node.lidar_timestamps) > 1 else np.array([0.0])
        
        print(f"Total LiDAR Sweeps:    {node.lidar_msg_count}")
        print(f"Duration:              {l_dur:.3f} s")
        print(f"Mean Rate:             {l_hz:.2f} Hz (Nominal: 10.0 Hz)")
        print(f"Median Interval:       {np.median(l_dt):.2f} ms")
        print(f"Mean Point Count:      {np.mean(node.lidar_point_counts):.0f} pts/sweep (Min: {np.min(node.lidar_point_counts)}, Max: {np.max(node.lidar_point_counts)})")
        print(f"Status:                {'✅ PASS' if (9.0 <= l_hz <= 11.0) else '⚠️ WARNING'}")

    # -------------------------------------------------------------
    # M8.2: Native IMU Analysis
    # -------------------------------------------------------------
    print("\n--- [M8.2: RoboSense Airy Native IMU Telemetry & Metrology] ---")
    if node.imu_msg_count < 100:
        print(f"❌ FAILED: Insufficient IMU samples received ({node.imu_msg_count} samples).")
        return
        
    stamps = np.array(node.imu_stamps_sec)
    i_dur = stamps[-1] - stamps[0]
    dt_ms = np.diff(stamps) * 1000.0
    
    mean_hz = (node.imu_msg_count - 1) / i_dur if i_dur > 0 else 0.0
    median_dt = np.median(dt_ms)
    mean_dt = np.mean(dt_ms)
    std_dt = np.std(dt_ms)
    p95_dt = np.percentile(dt_ms, 95)
    p99_dt = np.percentile(dt_ms, 99)
    min_dt = np.min(dt_ms)
    max_dt = np.max(dt_ms)
    
    # Check monotonicity and duplicates
    duplicates = np.sum(dt_ms == 0.0)
    non_monotonic = np.sum(dt_ms < 0.0)
    gaps_1_5x = np.sum(dt_ms > 1.5 * median_dt)
    gaps_2_0x = np.sum(dt_ms > 2.0 * median_dt)
    
    # Accelerometer statistics
    ax = np.array(node.accel_x_list)
    ay = np.array(node.accel_y_list)
    az = np.array(node.accel_z_list)
    amag = np.sqrt(ax**2 + ay**2 + az**2)
    
    ax_mean, ax_std = np.mean(ax), np.std(ax)
    ay_mean, ay_std = np.mean(ay), np.std(ay)
    az_mean, az_std = np.mean(az), np.std(az)
    amag_mean, amag_std = np.mean(amag), np.std(amag)
    
    # Gyroscope statistics
    gx = np.array(node.gyro_x_list)
    gy = np.array(node.gyro_y_list)
    gz = np.array(node.gyro_z_list)
    gmag = np.sqrt(gx**2 + gy**2 + gz**2)
    
    gx_mean, gx_std = np.mean(gx), np.std(gx)
    gy_mean, gy_std = np.mean(gy), np.std(gy)
    gz_mean, gz_std = np.mean(gz), np.std(gz)
    gmag_mean = np.mean(gmag)
    
    print(f"Total IMU Samples:     {node.imu_msg_count}")
    print(f"Duration:              {i_dur:.3f} s")
    print(f"Mean Rate:             {mean_hz:.2f} Hz")
    print(f"Median Interval:       {median_dt:.3f} ms")
    print(f"Mean Interval:         {mean_dt:.3f} ms (Std: {std_dt:.3f} ms)")
    print(f"Interval P95 / P99:    {p95_dt:.3f} ms / {p99_dt:.3f} ms")
    print(f"Interval Min / Max:    {min_dt:.3f} ms / {max_dt:.3f} ms")
    print(f"Duplicate Stamps:      {duplicates}")
    print(f"Non-Monotonic Stamps:  {non_monotonic}")
    print(f"Gaps > 1.5x Median:    {gaps_1_5x}")
    print(f"Gaps > 2.0x Median:    {gaps_2_0x}")
    print("\n--- Accelerometer Vector (Stationary Base-Down Level Pose) ---")
    print(f"  a_x:  {ax_mean:+.4f} ± {ax_std:.4f} m/s²")
    print(f"  a_y:  {ay_mean:+.4f} ± {ay_std:.4f} m/s²")
    print(f"  a_z:  {az_mean:+.4f} ± {az_std:.4f} m/s² (Expected dominant: -9.81 m/s²)")
    print(f"  |a|:  {amag_mean:.4f} ± {amag_std:.4f} m/s² (Expected magnitude: ~9.81 m/s²)")
    print("\n--- Gyroscope Bias (Stationary) ---")
    print(f"  w_x:  {gx_mean:+.5f} ± {gx_std:.5f} rad/s ({math.degrees(gx_mean):+.3f}°/s)")
    print(f"  w_y:  {gy_mean:+.5f} ± {gy_std:.5f} rad/s ({math.degrees(gy_mean):+.3f}°/s)")
    print(f"  w_z:  {gz_mean:+.5f} ± {gz_std:.5f} rad/s ({math.degrees(gz_mean):+.3f}°/s)")
    print(f"  |w|:  {gmag_mean:.5f} rad/s ({math.degrees(gmag_mean):.3f}°/s)")
    
    # Validation gates
    rate_ok = (180.0 <= mean_hz <= 220.0)
    jitter_ok = (duplicates == 0 and non_monotonic == 0 and gaps_2_0x == 0)
    gravity_ok = (8.5 <= abs(az_mean) <= 11.0 and az_mean < 0.0 and (9.0 <= amag_mean <= 10.5))
    gyro_ok = (gmag_mean < 0.05) # < 2.8 deg/s
    
    print("\n--- M8.2 Quality Gate Summary ---")
    print(f"  [Gate 1] IMU Rate Stability (180-220 Hz):       {'✅ PASS' if rate_ok else '❌ FAIL'}")
    print(f"  [Gate 2] Timing Integrity (0 dup/non-monotonic): {'✅ PASS' if jitter_ok else '❌ FAIL'}")
    print(f"  [Gate 3] Native -Z Gravity Vector (az ~ -9.81):   {'✅ PASS' if gravity_ok else '❌ FAIL'}")
    print(f"  [Gate 4] Stationary Gyroscope Magnitude (~0):   {'✅ PASS' if gyro_ok else '❌ FAIL'}")
    print("="*80 + "\n")

def main():
    rclpy.init()
    node = AiryQualificationNode(target_duration_sec=65.0)
    
    start_wall = time.time()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.start_time is not None:
                elapsed = time.time() - node.start_time
                if elapsed >= node.target_duration_sec:
                    node.get_logger().info(f"[M8 QUALIFICATION] Target duration {node.target_duration_sec}s reached.")
                    break
            else:
                # If no message received after 15 seconds, timeout
                if time.time() - start_wall > 15.0:
                    node.get_logger().error("[M8 QUALIFICATION] Timeout: No messages received on /rslidar_points or /rslidar_imu_data.")
                    break
    except KeyboardInterrupt:
        pass
    finally:
        analyze_and_report(node)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
