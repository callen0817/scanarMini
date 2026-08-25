#!/usr/bin/env python3
"""
scanR Milestone M7.2 — Isolated Single-Axis Rotation Qualification Suite
Records and audits live Euler rotations (Roll, Pitch, Yaw) to verify physical-to-estimator frame alignment and sign correctness.
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
from sensor_msgs.msg import Imu

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

class M7RotationValidatorNode(Node):
    def __init__(self, output_csv_path):
        super().__init__('m7_rotation_validator_node')
        self.output_csv_path = output_csv_path
        
        self.odom_sub = self.create_subscription(Odometry, '/aft_mapped_to_init', self.odom_cb, 50)
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_cb, 100)
        
        self.odom_records = []
        self.imu_records = []
        self.nan_detected = False
        
        self.csv_file = open(self.output_csv_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            "timestamp", "elapsed_s", 
            "pos_x", "pos_y", "pos_z", 
            "quat_x", "quat_y", "quat_z", "quat_w",
            "roll_deg", "pitch_deg", "yaw_deg",
            "d_roll_deg", "d_pitch_deg", "d_yaw_deg"
        ])
        
        self.baseline_samples = []
        self.baseline_established = False
        self.r0, self.p0, self.y0 = 0.0, 0.0, 0.0
        self.x0, self.y_pos0, self.z0 = 0.0, 0.0, 0.0
        self.start_time = None
        self.latest_state = None

    def odom_cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        if any(math.isnan(v) for v in [p.x, p.y, p.z, q.x, q.y, q.z, q.w]):
            self.nan_detected = True
            return
            
        r, pt, y = quat_to_euler(q.x, q.y, q.z, q.w)
        
        if self.start_time is None:
            self.start_time = t
            
        elapsed = t - self.start_time
        
        if not self.baseline_established:
            self.baseline_samples.append((p.x, p.y, p.z, r, pt, y))
            if len(self.baseline_samples) >= 30:
                bx = [s[0] for s in self.baseline_samples]
                by = [s[1] for s in self.baseline_samples]
                bz = [s[2] for s in self.baseline_samples]
                br = [s[3] for s in self.baseline_samples]
                bp = [s[4] for s in self.baseline_samples]
                byaw = [s[5] for s in self.baseline_samples]
                self.x0, self.y_pos0, self.z0 = float(np.mean(bx)), float(np.mean(by)), float(np.mean(bz))
                self.r0, self.p0, self.y0 = float(np.mean(br)), float(np.mean(bp)), float(np.mean(byaw))
                self.baseline_established = True
                print(f"\n[BASELINE LOCKED] Initial Euler: Roll={self.r0:.2f}°, Pitch={self.p0:.2f}°, Yaw={self.y0:.2f}°")
                print(">>> READY: Perform isolated motions (Yaw Left/Right, Pitch Up/Down, Roll CW/CCW) <<<\n")
            return
            
        dr = r - self.r0
        dp = pt - self.p0
        dy = y - self.y0
        
        self.latest_state = {
            "t": t, "elapsed": elapsed,
            "x": p.x - self.x0, "y": p.y - self.y_pos0, "z": p.z - self.z0,
            "roll": r, "pitch": pt, "yaw": y,
            "d_roll": dr, "d_pitch": dp, "d_yaw": dy
        }
        self.odom_records.append(self.latest_state)
        
        self.csv_writer.writerow([
            f"{t:.6f}", f"{elapsed:.3f}",
            f"{p.x:.6f}", f"{p.y:.6f}", f"{p.z:.6f}",
            f"{q.x:.6f}", f"{q.y:.6f}", f"{q.z:.6f}", f"{q.w:.6f}",
            f"{r:.4f}", f"{pt:.4f}", f"{y:.4f}",
            f"{dr:.4f}", f"{dp:.4f}", f"{dy:.4f}"
        ])

    def imu_cb(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        gx, gy, gz = msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z
        self.imu_records.append((t, gx, gy, gz))

    def close(self):
        if self.csv_file and not self.csv_file.closed:
            self.csv_file.flush()
            self.csv_file.close()

def run_m7_2_test(duration_sec=75):
    os.environ["ROS_DOMAIN_ID"] = "2"
    reports_dir = "/home/scanar/scanR/reports/m7_validation"
    os.makedirs(reports_dir, exist_ok=True)
    
    bag_path = os.path.join(reports_dir, "m7_2_rotations_bag")
    csv_path = os.path.join(reports_dir, "m7_2_rotations_telemetry.csv")
    json_path = os.path.join(reports_dir, "m7_2_rotations_results.json")
    
    subprocess.run(["rm", "-rf", bag_path], check=False)
    
    print("=" * 80)
    print(f"      scanR Milestone M7.2 — Isolated Rotation Qualification Run ({duration_sec} s)      ")
    print("=" * 80)
    print("Protocol Sequence:")
    print("  1. [t= 0.. 5s] Baseline Lock (Keep Stationary)")
    print("  2. [t= 5..20s] Yaw Left (CCW) -> return -> Yaw Right (CW) -> return")
    print("  3. [t=20..40s] Pitch Up (Tilt Up) -> return -> Pitch Down (Tilt Down) -> return")
    print("  4. [t=40..60s] Roll Right (CW) -> return -> Roll Left (CCW) -> return")
    print("  5. [t=60..75s] Stationary Hold")
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
    
    print("[M7.2] Starting ROS 2 bag recorder...")
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
    node = M7RotationValidatorNode(csv_path)
    
    start_wall = time.time()
    last_print = 0
    
    try:
        while rclpy.ok() and (time.time() - start_wall < duration_sec):
            rclpy.spin_once(node, timeout_sec=0.05)
            now = time.time()
            if now - last_print >= 0.25 and node.latest_state:
                s = node.latest_state
                elapsed = now - start_wall
                
                # Tag active dominant motion
                tag = "STATIONARY"
                if abs(s["d_yaw"]) > 8.0:
                    tag = f"YAW {'LEFT (CCW)' if s['d_yaw'] > 0 else 'RIGHT (CW)'} [{s['d_yaw']:+5.1f}°]"
                elif abs(s["d_pitch"]) > 8.0:
                    tag = f"PITCH {'UP' if s['d_pitch'] > 0 else 'DOWN'} [{s['d_pitch']:+5.1f}°]"
                elif abs(s["d_roll"]) > 8.0:
                    tag = f"ROLL {'RIGHT' if s['d_roll'] > 0 else 'LEFT'} [{s['d_roll']:+5.1f}°]"
                
                print(f"\r[{elapsed:4.1f}s/{duration_sec}s] "
                      f"ΔR: {s['d_roll']:+5.1f}° | ΔP: {s['d_pitch']:+5.1f}° | ΔY: {s['d_yaw']:+5.1f}° | "
                      f"Pos: [{s['x']:+4.2f}, {s['y']:+4.2f}, {s['z']:+4.2f}]m | "
                      f"Mode: {tag:<22}", end='', flush=True)
                last_print = now
                
    except KeyboardInterrupt:
        print("\n[M7.2] Session completed early by user.")
    finally:
        print("\n\n[M7.2] Finalizing bag archive...")
        bag_proc.send_signal(signal.SIGINT)
        try:
            bag_proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            bag_proc.kill()
            
        node.close()
        rclpy.shutdown()
        
    actual_duration = time.time() - start_wall
    if not node.odom_records:
        print("[ERROR] No odometry records captured.")
        return False
        
    d_rolls = [r["d_roll"] for r in node.odom_records]
    d_pitches = [r["d_pitch"] for r in node.odom_records]
    d_yaws = [r["d_yaw"] for r in node.odom_records]
    
    max_yaw_pos = max(d_yaws)
    max_yaw_neg = min(d_yaws)
    max_pitch_pos = max(d_pitches)
    max_pitch_neg = min(d_pitches)
    max_roll_pos = max(d_rolls)
    max_roll_neg = min(d_rolls)
    
    results = {
        "status": "EVALUATION_READY",
        "duration_sec": actual_duration,
        "total_odom_samples": len(node.odom_records),
        "total_imu_samples": len(node.imu_records),
        "yaw_excursions_deg": {"max_positive_ccw": max_yaw_pos, "max_negative_cw": max_yaw_neg},
        "pitch_excursions_deg": {"max_positive": max_pitch_pos, "max_negative": max_pitch_neg},
        "roll_excursions_deg": {"max_positive": max_roll_pos, "max_negative": max_roll_neg},
        "nan_detected": node.nan_detected
    }
    
    with open(json_path, 'w') as jf:
        json.dump(results, jf, indent=2)
        
    print("\n" + "=" * 80)
    print("                 scanR M7.2 ROTATION AUDIT SUMMARY REPORT                ")
    print("=" * 80)
    print(f"Total Duration:               {actual_duration:.1f} s")
    print(f"Total Odometry Steps:         {len(node.odom_records)}")
    print("-" * 80)
    print(f"Yaw Dynamic Range:            [{max_yaw_neg:+6.2f}°, {max_yaw_pos:+6.2f}°]")
    print(f"Pitch Dynamic Range:          [{max_pitch_neg:+6.2f}°, {max_pitch_pos:+6.2f}°]")
    print(f"Roll Dynamic Range:           [{max_roll_neg:+6.2f}°, {max_roll_pos:+6.2f}°]")
    print(f"Filter Resets / NaNs:         {'0 (PASSED)' if not node.nan_detected else 'FAILED'}")
    print("=" * 80)
    print(f"Bag Archive:    {bag_path}")
    print(f"Telemetry CSV:  {csv_path}")
    print(f"Results JSON:   {json_path}")
    print("=" * 80)
    return True

if __name__ == '__main__':
    dur = 75
    if len(sys.argv) > 1:
        dur = int(sys.argv[1])
    run_m7_2_test(dur)
