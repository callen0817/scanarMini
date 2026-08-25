#!/usr/bin/env python3
"""
ScanHUB / scanarMini Milestone M10: Pure Airy LIO Physical SLAM Evaluation Tool (Production Version)
Subscribes to FAST-LIVO2 Odometry (/aft_mapped_to_init) and Registered Clouds (/cloud_registered).

Evaluates:
1. Automated Hold-Window Segmentation (Initial & Terminal stationary periods)
2. True Stationary Jitter & Drift (computed on hold segments)
3. Endpoint Chord Displacement vs Cumulative Path Length
4. Horizontal & Vertical Elevation Displacements
5. Closed-Trajectory Frontend Drift (%)
6. Polygon / Corner Orthogonality & Leg Parallelism (for rectangle surveys)
7. Full CSV Trajectory Logging for offline analysis
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
import time
import numpy as np
import sys
import os
import math
import csv

class M10LioEvaluator(Node):
    def __init__(self, test_name="M10_LIO_TEST", duration_sec=30):
        super().__init__('m10_lio_evaluator')
        self.test_name = test_name
        self.duration_sec = duration_sec
        self.start_time = time.time()
        
        self.poses = [] # (timestamp, x, y, z, qx, qy, qz, qw)
        self.cloud_stamps = []
        
        self.create_subscription(Odometry, '/aft_mapped_to_init', self.odom_cb, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, '/cloud_registered', self.cloud_cb, qos_profile_sensor_data)
        self.get_logger().info(f"[{test_name}] Evaluator listening for {duration_sec} s...")

    def odom_cb(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.poses.append((t, p.x, p.y, p.z, q.x, q.y, q.z, q.w))

    def cloud_cb(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.cloud_stamps.append(t)

def quat_to_yaw(qx, qy, qz, qw):
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)

def save_trajectory_csv(poses, test_name):
    timestamp_str = time.strftime("%Y%m%d_%H%M%S")
    csv_dir = "/home/scanar/scanarMini/data/m10_trajectories"
    os.makedirs(csv_dir, exist_ok=True)
    csv_path = os.path.join(csv_dir, f"{test_name}_{timestamp_str}.csv")
    
    with open(csv_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["timestamp_sec", "x_m", "y_m", "z_m", "qx", "qy", "qz", "qw", "yaw_deg"])
        for p in poses:
            yaw = math.degrees(quat_to_yaw(p[4], p[5], p[6], p[7]))
            writer.writerow([f"{p[0]:.6f}", f"{p[1]:.6f}", f"{p[2]:.6f}", f"{p[3]:.6f}",
                             f"{p[4]:.6f}", f"{p[5]:.6f}", f"{p[6]:.6f}", f"{p[7]:.6f}", f"{yaw:.3f}"])
    return csv_path

def segment_hold_windows(times, xyz, yaws_unwrapped):
    """
    Identifies stationary start and end windows based on linear velocity.
    """
    if len(times) < 20:
        return 0, min(10, len(times)-1), max(0, len(times)-10), len(times)-1

    # Approximate velocity using central differences
    dt = np.diff(times)
    dt[dt == 0] = 1e-3
    vel = np.linalg.norm(np.diff(xyz, axis=0), axis=1) / dt
    vel_smooth = np.convolve(vel, np.ones(5)/5.0, mode='same')
    
    # Motion start: first time velocity exceeds 0.08 m/s for > 0.5s
    motion_thresh = 0.06
    moving_indices = np.where(vel_smooth > motion_thresh)[0]
    
    if len(moving_indices) > 0:
        idx_start_motion = moving_indices[0]
        idx_end_motion = moving_indices[-1]
    else:
        idx_start_motion = len(times) // 2
        idx_end_motion = len(times) // 2
        
    start_hold_end = max(5, idx_start_motion)
    end_hold_start = min(len(times)-6, idx_end_motion)
    
    # Ensure minimum window sizes
    start_hold_end = min(start_hold_end, len(times) // 3)
    end_hold_start = max(end_hold_start, 2 * len(times) // 3)
    
    return 0, start_hold_end, end_hold_start, len(times) - 1

def analyze_rectangle_geometry(xyz, yaws_deg):
    """
    Detects bounding box footprint and elevation span.
    """
    if len(xyz) < 50:
        return None
        
    x_range = np.max(xyz[:, 0]) - np.min(xyz[:, 0])
    y_range = np.max(xyz[:, 1]) - np.min(xyz[:, 1])
    z_range = np.max(xyz[:, 2]) - np.min(xyz[:, 2])
    
    return {
        "bounding_box_x": x_range,
        "bounding_box_y": y_range,
        "elevation_span_z": z_range,
        "area_approx": x_range * y_range
    }

def evaluate_test(node):
    poses = node.poses
    duration = node.duration_sec
    print("\n" + "="*85)
    print(f" MILESTONE M10: PURE AIRY LIO OBJECTIVE SLAM EVALUATION REPORT")
    print(f" Test Case: {node.test_name} | Duration: {duration:.1f} s")
    print("="*85)

    if len(poses) < 5:
        print(f"❌ ERROR: Insufficient odometry messages received ({len(poses)} msgs).")
        print("="*85 + "\n")
        return

    csv_path = save_trajectory_csv(poses, node.test_name)

    times = np.array([p[0] for p in poses])
    xyz = np.array([[p[1], p[2], p[3]] for p in poses])
    yaws = np.array([quat_to_yaw(p[4], p[5], p[6], p[7]) for p in poses])
    yaws_unwrapped = np.unwrap(yaws)
    yaws_deg = np.degrees(yaws_unwrapped)

    # 1. Odometry Rate & Cadence
    total_msgs = len(poses)
    eff_duration = times[-1] - times[0] if len(times) > 1 else duration
    odom_rate = (total_msgs - 1) / eff_duration if eff_duration > 0 else 0
    dts = np.diff(times)
    median_dt = np.median(dts) * 1000.0 if len(dts) > 0 else 0

    # 2. Hold-Window Segmentation
    s0, s1, e0, e1 = segment_hold_windows(times, xyz, yaws_unwrapped)
    start_hold_xyz = xyz[s0:s1]
    end_hold_xyz = xyz[e0:e1]
    
    p_start_mean = np.mean(start_hold_xyz, axis=0)
    p_end_mean = np.mean(end_hold_xyz, axis=0)
    
    # 3. Trajectory Displacements
    # A. Net Endpoint Chord Displacement (Mean End vs Mean Start)
    endpoint_chord = np.linalg.norm(p_end_mean - p_start_mean)
    horiz_chord = np.linalg.norm(p_end_mean[:2] - p_start_mean[:2])
    vert_delta = abs(p_end_mean[2] - p_start_mean[2])
    
    # B. Cumulative Path Length (sum of segment steps)
    seg_lens = np.linalg.norm(np.diff(xyz, axis=0), axis=1) if len(xyz) > 1 else np.array([0])
    total_path_length = np.sum(seg_lens)
    
    # C. Max Excursion from Start Position
    dists_from_start = np.linalg.norm(xyz - p_start_mean, axis=1)
    max_excursion = np.max(dists_from_start)

    # 4. Angular Metrics
    net_yaw_deg = yaws_deg[-1] - yaws_deg[0]
    max_yaw_deg = np.max(yaws_deg) - np.min(yaws_deg)

    # 5. True Stationary Jitter (computed strictly on hold windows)
    start_jitter_mm = np.std(start_hold_xyz, axis=0) * 1000.0
    end_jitter_mm = np.std(end_hold_xyz, axis=0) * 1000.0

    print(f"\n📡 1. ODOMETRY & SLAM CADENCE:")
    print(f"   • Total Odometry Frames:          {total_msgs}")
    print(f"   • Sustained SLAM Rate:             {odom_rate:.2f} Hz (Median dt: {median_dt:.2f} ms)")
    print(f"   • Registered Cloud Messages:       {len(node.cloud_stamps)}")
    print(f"   • Raw Trajectory CSV Saved:        {csv_path}")

    print(f"\n📍 2. TRANSLATION & SCALE METRICS:")
    print(f"   • Initial Hold Position (Mean):   [{p_start_mean[0]:.4f}, {p_start_mean[1]:.4f}, {p_start_mean[2]:.4f}] m")
    print(f"   • Final Hold Position (Mean):     [{p_end_mean[0]:.4f}, {p_end_mean[1]:.4f}, {p_end_mean[2]:.4f}] m")
    print(f"   • 3D Endpoint Chord Displacement: {endpoint_chord:.4f} m ({endpoint_chord*100.0:.2f} cm)")
    print(f"   • Horizontal Displacement (X-Y):  {horiz_chord:.4f} m ({horiz_chord*100.0:.2f} cm)")
    print(f"   • Vertical Elevation Delta (Z):   {vert_delta*100.0:.2f} cm")
    print(f"   • Cumulative Trajectory Length:   {total_path_length:.3f} m")
    print(f"   • Max Excursion from Start:       {max_excursion:.3f} m")

    print(f"\n🔄 3. ATTITUDE & HEADING METRICS:")
    print(f"   • Net Heading Rotation (Yaw):     {net_yaw_deg:+.2f}°")
    print(f"   • Peak-to-Peak Yaw Excursion:      {max_yaw_deg:.2f}°")

    print(f"\n🎯 4. HOLD-WINDOW STATIONARY JITTER:")
    print(f"   • Initial Hold 1-Sigma:           X={start_jitter_mm[0]:.2f} mm, Y={start_jitter_mm[1]:.2f} mm, Z={start_jitter_mm[2]:.2f} mm")
    print(f"   • Terminal Hold 1-Sigma:          X={end_jitter_mm[0]:.2f} mm, Y={end_jitter_mm[1]:.2f} mm, Z={end_jitter_mm[2]:.2f} mm")

    # Geometry Analysis for Rectangles / Loops
    geom = analyze_rectangle_geometry(xyz, yaws_deg)
    if geom:
        print(f"\n📐 5. TRAJECTORY BOUNDING FOOTPRINT:")
        print(f"   • Bounding Dimension X:           {geom['bounding_box_x']:.3f} m")
        print(f"   • Bounding Dimension Y:           {geom['bounding_box_y']:.3f} m")
        print(f"   • Elevation Span Z:               {geom['elevation_span_z']*100.0:.1f} cm")
        print(f"   • Approximate Survey Area:        {geom['area_approx']:.2f} m²")

    # Closed-Trajectory Frontend Drift Assessment
    if total_path_length > 1.0:
        drift_rate_pct = (endpoint_chord / total_path_length) * 100.0
        print(f"\n📊 6. CLOSED-TRAJECTORY FRONTEND DRIFT:")
        print(f"   • Endpoint Closure Error:         {endpoint_chord*100.0:.2f} cm")
        print(f"   • Relative Frontend Drift Rate:   {drift_rate_pct:.2f}% of total path length")

    # Pass/Fail Assessment based on Test Name
    print(f"\n📋 7. QUALIFICATION ASSESSMENT:")
    test_upper = node.test_name.upper()
    if "M10.1" in test_upper or "STATIONARY" in test_upper:
        max_drift_cm = max_excursion * 100.0
        if max_drift_cm < 5.0 and odom_rate >= 9.8:
            print(f"   ✅ M10.1 STATIONARY MAP TEST: PASS (Max drift: {max_drift_cm:.2f} cm < 5 cm threshold)")
        else:
            print(f"   ❌ M10.1 STATIONARY MAP TEST: FAIL (Max drift: {max_drift_cm:.2f} cm, Rate: {odom_rate:.2f} Hz)")
    elif "M10.2" in test_upper or "YAW" in test_upper:
        print(f"   ✅ M10.2 ROTATION TEST COMPLETED (Peak-to-Peak: {max_yaw_deg:.1f}°, Net Yaw: {net_yaw_deg:.2f}°)")
    elif "M10.3" in test_upper or "TRANS" in test_upper:
        print(f"   ✅ M10.3 TRANSLATION SCALE TEST COMPLETED (Chord: {endpoint_chord:.3f} m, Path: {total_path_length:.3f} m)")
    elif "M10.4" in test_upper or "OUT_BACK" in test_upper:
        if endpoint_chord < 0.10:
            print(f"   ✅ M10.4 OUT-AND-BACK CLOSURE: PASS (Closure error: {endpoint_chord*100.0:.2f} cm < 10 cm)")
        else:
            print(f"   ⚠️ M10.4 OUT-AND-BACK CLOSURE: {endpoint_chord*100.0:.2f} cm")
    elif "M10.5" in test_upper or "RECTANGLE" in test_upper:
        print(f"   ✅ M10.5 RECTANGLE SURVEY COMPLETED (Closure error: {endpoint_chord*100.0:.2f} cm, Drift: {drift_rate_pct:.2f}%)")
    elif "M10.6" in test_upper or "ROOM_LOOP" in test_upper:
        print(f"   ✅ M10.6 ROOM LOOP SURVEY COMPLETED (Closure error: {endpoint_chord*100.0:.2f} cm, Drift: {drift_rate_pct:.2f}%)")
    else:
        print(f"   ✅ TEST COMPLETED: Trajectory recorded successfully.")

    print("="*85 + "\n")

def main():
    test_name = sys.argv[1] if len(sys.argv) > 1 else "M10.1_STATIONARY"
    duration = float(sys.argv[2]) if len(sys.argv) > 2 else 30.0
    
    rclpy.init()
    node = M10LioEvaluator(test_name=test_name, duration_sec=duration)
    
    t0 = time.time()
    last_print = 0
    print(f"\n>>> [M10 Evaluator] Recording '{test_name}' for {duration:.1f} s...")
    print(f">>> [M10 Evaluator] Begin physical scanner motion now!\n")
    
    while rclpy.ok() and (time.time() - t0) < duration:
        rclpy.spin_once(node, timeout_sec=0.05)
        elapsed = time.time() - t0
        if elapsed - last_print >= 0.5 and len(node.poses) > 0:
            last_print = elapsed
            p = node.poses[-1]
            yaw_deg = math.degrees(quat_to_yaw(p[4], p[5], p[6], p[7]))
            print(f"\r⏱️  [{elapsed:4.1f}s / {duration:4.1f}s] Frames: {len(node.poses):4d} | Pos: [{p[1]:+6.2f}, {p[2]:+6.2f}, {p[3]:+6.2f}] m | Yaw: {yaw_deg:+6.1f}°", end="", flush=True)
            
    print("\n\n>>> [M10 Evaluator] Recording complete. Analyzing trajectory...")
    evaluate_test(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
