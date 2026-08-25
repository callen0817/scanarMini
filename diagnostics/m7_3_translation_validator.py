#!/usr/bin/env python3
"""
scanR Milestone M7.3 — 5-Meter Linear Translation Qualification Suite
Records and audits live position and orientation during straight-line walking to verify scale, heading, and translation integrity.
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

class M7TranslationValidatorNode(Node):
    def __init__(self, output_csv_path):
        super().__init__('m7_translation_validator_node')
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
            "dx_m", "dy_m", "dz_m", "dist_3d_m", "dist_2d_m",
            "quat_x", "quat_y", "quat_z", "quat_w",
            "roll_deg", "pitch_deg", "yaw_deg",
            "d_roll_deg", "d_pitch_deg", "d_yaw_deg"
        ])
        
        self.baseline_samples = []
        self.baseline_established = False
        self.r0, self.p0, self.y0 = 0.0, 0.0, 0.0
        self.x0, self.y0_pos, self.z0 = 0.0, 0.0, 0.0
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
                self.x0, self.y0_pos, self.z0 = float(np.mean(bx)), float(np.mean(by)), float(np.mean(bz))
                self.r0, self.p0, self.y0 = float(np.mean(br)), float(np.mean(bp)), float(np.mean(byaw))
                self.baseline_established = True
                print(f"\n[BASELINE LOCKED] Origin: [{self.x0:.3f}, {self.y0_pos:.3f}, {self.z0:.3f}] m | Yaw={self.y0:.2f}°")
                print(">>> START WALKING: Move straight forward ~5 meters, then hold stationary <<<\n")
            return
            
        dx = p.x - self.x0
        dy = p.y - self.y0_pos
        dz = p.z - self.z0
        dist_3d = math.sqrt(dx*dx + dy*dy + dz*dz)
        dist_2d = math.sqrt(dx*dx + dy*dy)
        
        dr = r - self.r0
        dp = pt - self.p0
        dyaw = y - self.y0
        
        self.latest_state = {
            "t": t, "elapsed": elapsed,
            "dx": dx, "dy": dy, "dz": dz,
            "dist_3d": dist_3d, "dist_2d": dist_2d,
            "roll": r, "pitch": pt, "yaw": y,
            "d_roll": dr, "d_pitch": dp, "d_yaw": dyaw
        }
        self.odom_records.append(self.latest_state)
        
        self.csv_writer.writerow([
            f"{t:.6f}", f"{elapsed:.3f}",
            f"{p.x:.6f}", f"{p.y:.6f}", f"{p.z:.6f}",
            f"{dx:.4f}", f"{dy:.4f}", f"{dz:.4f}", f"{dist_3d:.4f}", f"{dist_2d:.4f}",
            f"{q.x:.6f}", f"{q.y:.6f}", f"{q.z:.6f}", f"{q.w:.6f}",
            f"{r:.4f}", f"{pt:.4f}", f"{y:.4f}",
            f"{dr:.4f}", f"{dp:.4f}", f"{dyaw:.4f}"
        ])

    def imu_cb(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        gx, gy, gz = msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z
        self.imu_records.append((t, gx, gy, gz))

    def close(self):
        if self.csv_file and not self.csv_file.closed:
            self.csv_file.flush()
            self.csv_file.close()

def run_m7_3_test(duration_sec=50):
    os.environ["ROS_DOMAIN_ID"] = "2"
    reports_dir = "/home/scanar/scanR/reports/m7_validation"
    os.makedirs(reports_dir, exist_ok=True)
    
    bag_path = os.path.join(reports_dir, "m7_3_translation_bag")
    csv_path = os.path.join(reports_dir, "m7_3_translation_telemetry.csv")
    json_path = os.path.join(reports_dir, "m7_3_translation_results.json")
    
    subprocess.run(["rm", "-rf", bag_path], check=False)
    
    print("=" * 80)
    print(f"      scanR Milestone M7.3 — 5-Meter Straight Translation Run ({duration_sec} s)      ")
    print("=" * 80)
    print("Protocol Sequence:")
    print("  1. [t= 0.. 5s] Origin Baseline Lock (Hold still at start marker)")
    print("  2. [t= 5..35s] Straight Walk (~5 meters along forward heading)")
    print("  3. [t=35..50s] Endpoint Hold (Hold still at 5m marker)")
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
    
    print("[M7.3] Starting ROS 2 bag recorder...")
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
    node = M7TranslationValidatorNode(csv_path)
    
    start_wall = time.time()
    last_print = 0
    
    try:
        while rclpy.ok() and (time.time() - start_wall < duration_sec):
            rclpy.spin_once(node, timeout_sec=0.05)
            now = time.time()
            if now - last_print >= 0.25 and node.latest_state:
                s = node.latest_state
                elapsed = now - start_wall
                print(f"\r[{elapsed:4.1f}s/{duration_sec}s] "
                      f"3D Dist: {s['dist_3d']:5.2f}m | 2D Dist: {s['dist_2d']:5.2f}m | "
                      f"Displacement [ΔX, ΔY, ΔZ]: [{s['dx']:+5.2f}, {s['dy']:+5.2f}, {s['dz']:+5.2f}]m | "
                      f"ΔYaw: {s['d_yaw']:+5.1f}°", end='', flush=True)
                last_print = now
                
    except KeyboardInterrupt:
        print("\n[M7.3] Session completed early by user.")
    finally:
        print("\n\n[M7.3] Finalizing bag archive...")
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
        
    dists_3d = [r["dist_3d"] for r in node.odom_records]
    dists_2d = [r["dist_2d"] for r in node.odom_records]
    dxs = [r["dx"] for r in node.odom_records]
    dys = [r["dy"] for r in node.odom_records]
    dzs = [r["dz"] for r in node.odom_records]
    dyaws = [r["d_yaw"] for r in node.odom_records]
    
    # Endpoint stats (last 30 samples)
    end_3d = float(np.mean(dists_3d[-30:])) if len(dists_3d) >= 30 else dists_3d[-1]
    end_2d = float(np.mean(dists_2d[-30:])) if len(dists_2d) >= 30 else dists_2d[-1]
    end_dx = float(np.mean(dxs[-30:])) if len(dxs) >= 30 else dxs[-1]
    end_dy = float(np.mean(dys[-30:])) if len(dys) >= 30 else dys[-1]
    end_dz = float(np.mean(dzs[-30:])) if len(dzs) >= 30 else dzs[-1]
    max_yaw_dev = float(np.max(np.abs(dyaws)))
    
    results = {
        "status": "EVALUATION_READY",
        "duration_sec": actual_duration,
        "total_odom_samples": len(node.odom_records),
        "total_imu_samples": len(node.imu_records),
        "measured_endpoint_distance_3d_m": end_3d,
        "measured_endpoint_distance_2d_m": end_2d,
        "endpoint_displacement_m": {"dx": end_dx, "dy": end_dy, "dz": end_dz},
        "max_yaw_deviation_deg": max_yaw_dev,
        "nan_detected": node.nan_detected
    }
    
    with open(json_path, 'w') as jf:
        json.dump(results, jf, indent=2)
        
    print("\n" + "=" * 80)
    print("                 scanR M7.3 TRANSLATION AUDIT SUMMARY REPORT              ")
    print("=" * 80)
    print(f"Total Duration:               {actual_duration:.1f} s")
    print(f"Total Odometry Steps:         {len(node.odom_records)}")
    print("-" * 80)
    print(f"Endpoint 3D Distance:         {end_3d:.3f} m ({end_3d*100:.1f} cm)")
    print(f"Endpoint 2D Distance:         {end_2d:.3f} m ({end_2d*100:.1f} cm)")
    print(f"Displacement Vector [X,Y,Z]:  [{end_dx:+.3f}, {end_dy:+.3f}, {end_dz:+.3f}] m")
    print(f"Max Heading Deviation:        {max_yaw_dev:.2f}°")
    print(f"Filter Resets / NaNs:         {'0 (PASSED)' if not node.nan_detected else 'FAILED'}")
    print("=" * 80)
    print(f"Bag Archive:    {bag_path}")
    print(f"Telemetry CSV:  {csv_path}")
    print(f"Results JSON:   {json_path}")
    print("=" * 80)
    return True

if __name__ == '__main__':
    dur = 50
    if len(sys.argv) > 1:
        dur = int(sys.argv[1])
    run_m7_3_test(dur)
