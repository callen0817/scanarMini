#!/usr/bin/env python3
"""
scanR Milestone M7.1 — Static Baseline Qualification Suite
Records a 5-minute (300 s) stationary dataset with full ROS 2 bag and high-rate telemetry analysis.
Calculates:
- 3D Position drift (X, Y, Z, Euclidean, drift rate cm/min)
- Attitude drift (Roll, Pitch, Yaw in degrees)
- Gravity vector & acceleration stability
- Message continuity, Hz, and zero-NaN qualification
"""

import os
import sys
import time
import math
import signal
import subprocess
import csv
import json
import numpy as np

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, PointCloud2

def quat_to_euler(x, y, z, w):
    """Converts quaternion (x, y, z, w) to Euler angles (roll, pitch, yaw) in degrees."""
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)

class M7StaticBaselineNode(Node):
    def __init__(self, output_csv_path):
        super().__init__('m7_static_baseline_node')
        self.output_csv_path = output_csv_path
        
        self.odom_sub = self.create_subscription(Odometry, '/aft_mapped_to_init', self.odom_cb, 50)
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_cb, 100)
        self.lidar_sub = self.create_subscription(PointCloud2, '/rslidar_points', self.lidar_cb, 10)
        
        self.odom_records = []
        self.imu_records = []
        self.lidar_count = 0
        self.nan_detected = False
        
        self.csv_file = open(self.output_csv_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            "timestamp", "elapsed_s", 
            "pos_x", "pos_y", "pos_z", 
            "quat_x", "quat_y", "quat_z", "quat_w",
            "roll_deg", "pitch_deg", "yaw_deg",
            "drift_euclidean_mm", "drift_rate_cm_min"
        ])
        
        self.first_pose = None
        self.last_pose = None
        self.start_time = None
        self.latest_imu = None

    def odom_cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        if any(math.isnan(v) for v in [p.x, p.y, p.z, q.x, q.y, q.z, q.w]):
            self.nan_detected = True
            print(f"[FATAL] NaN detected in Odometry message at t={t}")
            return
            
        roll, pitch, yaw = quat_to_euler(q.x, q.y, q.z, q.w)
        
        if self.first_pose is None:
            self.first_pose = (t, p.x, p.y, p.z, roll, pitch, yaw)
            self.start_time = t
            
        self.last_pose = (t, p.x, p.y, p.z, roll, pitch, yaw)
        elapsed = t - self.start_time
        
        dx = p.x - self.first_pose[1]
        dy = p.y - self.first_pose[2]
        dz = p.z - self.first_pose[3]
        drift_mm = math.sqrt(dx*dx + dy*dy + dz*dz) * 1000.0
        drift_rate_cm_min = (drift_mm / 10.0) / (elapsed / 60.0) if elapsed > 1.0 else 0.0
        
        self.odom_records.append({
            "t": t, "elapsed": elapsed,
            "x": p.x, "y": p.y, "z": p.z,
            "dx": dx, "dy": dy, "dz": dz,
            "drift_mm": drift_mm,
            "roll": roll, "pitch": pitch, "yaw": yaw
        })
        
        self.csv_writer.writerow([
            f"{t:.6f}", f"{elapsed:.3f}",
            f"{p.x:.6f}", f"{p.y:.6f}", f"{p.z:.6f}",
            f"{q.x:.6f}", f"{q.y:.6f}", f"{q.z:.6f}", f"{q.w:.6f}",
            f"{roll:.4f}", f"{pitch:.4f}", f"{yaw:.4f}",
            f"{drift_mm:.3f}", f"{drift_rate_cm_min:.4f}"
        ])

    def imu_cb(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        ax, ay, az = msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z
        gx, gy, gz = msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z
        norm_a = math.sqrt(ax*ax + ay*ay + az*az)
        
        self.latest_imu = (t, ax, ay, az, gx, gy, gz, norm_a)
        self.imu_records.append(self.latest_imu)

    def lidar_cb(self, msg):
        self.lidar_count += 1

    def close(self):
        if self.csv_file and not self.csv_file.closed:
            self.csv_file.flush()
            self.csv_file.close()

def run_m7_1_test(duration_sec=300):
    os.environ["ROS_DOMAIN_ID"] = "2"
    
    reports_dir = "/home/scanar/scanR/reports/m7_validation"
    os.makedirs(reports_dir, exist_ok=True)
    
    bag_path = os.path.join(reports_dir, "m7_1_static_baseline_bag")
    csv_path = os.path.join(reports_dir, "m7_1_static_telemetry.csv")
    json_path = os.path.join(reports_dir, "m7_1_static_results.json")
    
    subprocess.run(["rm", "-rf", bag_path], check=False)
    
    print("=" * 80)
    print(f"      scanR Milestone M7.1 — Static Baseline Qualification Run ({duration_sec} s)      ")
    print("=" * 80)
    print(f"Target Duration: {duration_sec} seconds (5.0 minutes)")
    print(f"Bag Destination: {bag_path}")
    print(f"CSV Telemetry:   {csv_path}")
    print("-" * 80)
    
    topics = [
        "/imu",
        "/rslidar_points",
        "/aft_mapped_to_init",
        "/LIVO2/imu_propagate",
        "/cloud_registered",
        "/tf",
        "/tf_static"
    ]
    
    print("[M7.1] Starting ROS 2 bag recorder...")
    bag_env = os.environ.copy()
    bag_env["ROS_DOMAIN_ID"] = "2"
    bag_cmd = ["ros2", "bag", "record", "-o", bag_path] + topics
    bag_proc = subprocess.Popen(
        bag_cmd,
        env=bag_env,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL
    )
    
    rclpy.init()
    node = M7StaticBaselineNode(csv_path)
    
    start_wall = time.time()
    last_print = start_wall
    
    print("\n🚀 RECORDING IN PROGRESS — KEEP RIG ABSOLUTELY STATIONARY!")
    
    try:
        while rclpy.ok() and (time.time() - start_wall < duration_sec):
            rclpy.spin_once(node, timeout_sec=0.05)
            
            now = time.time()
            if now - last_print >= 15.0:
                elapsed = now - start_wall
                remaining = duration_sec - elapsed
                pct = (elapsed / duration_sec) * 100.0
                
                if node.odom_records:
                    latest = node.odom_records[-1]
                    drift = latest["drift_mm"]
                    yaw = latest["yaw"]
                    print(f"[{elapsed:5.1f}s / {duration_sec}s | {pct:4.1f}%] "
                          f"Odom Poses: {len(node.odom_records):4d} | "
                          f"IMU: {len(node.imu_records):5d} | "
                          f"LiDAR: {node.lidar_count:4d} | "
                          f"Drift: {drift:6.2f} mm ({drift/10:.3f} cm) | "
                          f"Yaw: {yaw:6.2f}°")
                else:
                    print(f"[{elapsed:5.1f}s / {duration_sec}s] Waiting for first odom pose...")
                    
                last_print = now
                
    except KeyboardInterrupt:
        print("\n[M7.1] User interrupted run early.")
    finally:
        print("\n[M7.1] Finalizing ROS 2 bag recording...")
        bag_proc.send_signal(signal.SIGINT)
        try:
            bag_proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            bag_proc.kill()
            
        node.close()
        rclpy.shutdown()
        
    actual_duration = time.time() - start_wall
    print(f"\n✅ Static baseline recording finished after {actual_duration:.1f} s.")
    
    if not node.odom_records:
        print("[ERROR] No odometry records received during test!")
        return False
        
    drifts = [r["drift_mm"] for r in node.odom_records]
    yaws = [r["yaw"] for r in node.odom_records]
    rolls = [r["roll"] for r in node.odom_records]
    pitches = [r["pitch"] for r in node.odom_records]
    
    initial = node.odom_records[0]
    final = node.odom_records[-1]
    
    net_dx = (final["x"] - initial["x"]) * 1000.0
    net_dy = (final["y"] - initial["y"]) * 1000.0
    net_dz = (final["z"] - initial["z"]) * 1000.0
    net_drift_mm = math.sqrt(net_dx*net_dx + net_dy*net_dy + net_dz*net_dz)
    max_drift_mm = max(drifts)
    mean_drift_mm = float(np.mean(drifts))
    std_drift_mm = float(np.std(drifts))
    
    yaw_drift_deg = final["yaw"] - initial["yaw"]
    pitch_drift_deg = final["pitch"] - initial["pitch"]
    roll_drift_deg = final["roll"] - initial["roll"]
    
    if node.imu_records:
        acc_norms = [r[7] for r in node.imu_records]
        mean_gravity = float(np.mean(acc_norms))
        std_gravity = float(np.std(acc_norms))
        gyro_xs = [r[4] for r in node.imu_records]
        gyro_ys = [r[5] for r in node.imu_records]
        gyro_zs = [r[6] for r in node.imu_records]
        mean_gyro_bias = [float(np.mean(gyro_xs)), float(np.mean(gyro_ys)), float(np.mean(gyro_zs))]
    else:
        mean_gravity, std_gravity = 0.0, 0.0
        mean_gyro_bias = [0.0, 0.0, 0.0]
        
    drift_rate_cm_per_min = (net_drift_mm / 10.0) / (actual_duration / 60.0)
    
    results = {
        "status": "PASS" if not node.nan_detected and max_drift_mm < 500.0 else "FAIL",
        "test_name": "M7.1_Static_Baseline",
        "duration_sec": actual_duration,
        "odom_samples": len(node.odom_records),
        "imu_samples": len(node.imu_records),
        "lidar_samples": node.lidar_count,
        "odom_rate_hz": len(node.odom_records) / actual_duration,
        "imu_rate_hz": len(node.imu_records) / actual_duration,
        "lidar_rate_hz": node.lidar_count / actual_duration,
        "initial_pose": {"x": initial["x"], "y": initial["y"], "z": initial["z"], "roll": initial["roll"], "pitch": initial["pitch"], "yaw": initial["yaw"]},
        "final_pose": {"x": final["x"], "y": final["y"], "z": final["z"], "roll": final["roll"], "pitch": final["pitch"], "yaw": final["yaw"]},
        "displacement_mm": {"dx": net_dx, "dy": net_dy, "dz": net_dz, "net_euclidean": net_drift_mm},
        "max_drift_mm": max_drift_mm,
        "mean_drift_mm": mean_drift_mm,
        "std_drift_mm": std_drift_mm,
        "drift_rate_cm_min": drift_rate_cm_per_min,
        "attitude_drift_deg": {"roll": roll_drift_deg, "pitch": pitch_drift_deg, "yaw": yaw_drift_deg},
        "gravity_magnitude": {"mean": mean_gravity, "std": std_gravity},
        "gyro_bias_rad_s": mean_gyro_bias,
        "nan_detected": node.nan_detected
    }
    
    with open(json_path, 'w') as jf:
        json.dump(results, jf, indent=2)
        
    print("\n" + "=" * 80)
    print("                 scanR M7.1 STATIC BASELINE SUMMARY REPORT               ")
    print("=" * 80)
    print(f"Status:                      {results['status']}")
    print(f"Duration:                    {actual_duration:.2f} s ({actual_duration/60:.2f} min)")
    print(f"Total Odom Frames Processed: {len(node.odom_records)} ({results['odom_rate_hz']:.1f} Hz)")
    print(f"Total IMU Samples:           {len(node.imu_records)} ({results['imu_rate_hz']:.1f} Hz)")
    print(f"Total LiDAR Scans:           {node.lidar_count} ({results['lidar_rate_hz']:.1f} Hz)")
    print("-" * 80)
    print(f"Initial Position (X,Y,Z):    [{initial['x']:.4f}, {initial['y']:.4f}, {initial['z']:.4f}] m")
    print(f"Final Position   (X,Y,Z):    [{final['x']:.4f}, {final['y']:.4f}, {final['z']:.4f}] m")
    print(f"Net Position Drift:          {net_drift_mm:.2f} mm ({net_drift_mm/10:.3f} cm)")
    print(f"Max Position Drift:          {max_drift_mm:.2f} mm ({max_drift_mm/10:.3f} cm)")
    print(f"Drift Rate:                  {drift_rate_cm_per_min:.3f} cm / min")
    print("-" * 80)
    print(f"Attitude Drift (R, P, Y):    [{roll_drift_deg:.3f}°, {pitch_drift_deg:.3f}°, {yaw_drift_deg:.3f}°]")
    print(f"Mean Gravity Magnitude:      {mean_gravity:.4f} ± {std_gravity:.4f} m/s²")
    print(f"Mean Gyro Bias (X,Y,Z):      [{mean_gyro_bias[0]:.6f}, {mean_gyro_bias[1]:.6f}, {mean_gyro_bias[2]:.6f}] rad/s")
    print(f"Zero-NaN Filter Check:       {'PASSED (0 NaNs)' if not node.nan_detected else 'FAILED (NaNs detected)'}")
    print("=" * 80)
    print(f"Bag Archive:    {bag_path}")
    print(f"Telemetry CSV:  {csv_path}")
    print(f"Results JSON:   {json_path}")
    print("=" * 80)
    return True

if __name__ == '__main__':
    dur = 300
    if len(sys.argv) > 1:
        dur = int(sys.argv[1])
    run_m7_1_test(dur)
