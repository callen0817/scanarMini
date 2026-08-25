#!/usr/bin/env python3
"""
Deep Telemetry Failure Window Analysis Script for Session Scan_0804_1443_20260804_144415.
Correlates LiDAR gaps, receive delays, and SLAM pose steps timestamp-by-timestamp.
"""

import sys
import os
import csv
import math
import numpy as np

session_dir = "/home/scanar/scanR/diagnostics/Scan_0804_1443_20260804_144415"
lidar_csv = os.path.join(session_dir, "lidar.csv")
imu_csv = os.path.join(session_dir, "imu.csv")
slam_csv = os.path.join(session_dir, "slam.csv")

print("======================================================================")
print(f"FAILURE WINDOW TELEMETRY CORRELATION: {os.path.basename(session_dir)}")
print("======================================================================")

# 1. Parse LiDAR Scans
lidar_scans = []
if os.path.exists(lidar_csv):
    with open(lidar_csv, "r") as f:
        reader = csv.DictReader(f)
        for row in reader:
            sys_t = float(row["sys_time"])
            hdr_t = float(row["sensor_sec"])
            pts = int(row["point_count"])
            lidar_scans.append((sys_t, hdr_t, pts))

# 2. Parse SLAM Poses
slam_poses = []
if os.path.exists(slam_csv):
    with open(slam_csv, "r") as f:
        reader = csv.DictReader(f)
        for row in reader:
            sys_t = float(row["sys_time"])
            pose_t = float(row["pose_sec"])
            x = float(row["x"])
            y = float(row["y"])
            z = float(row["z"])
            yaw = float(row["yaw"])
            slam_poses.append((sys_t, pose_t, x, y, z, yaw))

t0 = lidar_scans[0][1] if lidar_scans else (slam_poses[0][1] if slam_poses else 0)

print("\n--- CHRONOLOGICAL FAILURE SEQUENCE (t = 28.0s to 43.0s) ---")
print(f"{'Rel Time (s)':<12} | {'Event Type':<12} | {'LiDAR Gap / Step':<18} | {'X (m)':<8} | {'Y (m)':<8} | {'Z (m)':<8} | {'Details'}")
print("-" * 100)

events = []

for i in range(1, len(lidar_scans)):
    sys_t, hdr_t, pts = lidar_scans[i]
    _, prev_hdr, _ = lidar_scans[i-1]
    hdr_delta_ms = (hdr_t - prev_hdr) * 1000.0
    rel_t = hdr_t - t0
    if 28.0 <= rel_t <= 43.0 and hdr_delta_ms > 150.0:
        events.append((rel_t, "LIDAR GAP", f"Gap: {hdr_delta_ms:.1f} ms", f"{pts} pts"))

for i in range(1, len(slam_poses)):
    st, pt, x, y, z, yaw = slam_poses[i]
    _, _, px, py, pz, _ = slam_poses[i-1]
    rel_t = pt - t0
    step_dist = math.sqrt((x - px)**2 + (y - py)**2 + (z - pz)**2)
    if 28.0 <= rel_t <= 43.0:
        events.append((rel_t, "SLAM POSE", f"Step: {step_dist:.2f} m", f"X={x:.2f}, Y={y:.2f}, Z={z:.2f}"))

events.sort(key=lambda ev: ev[0])

for rel_t, ev_type, val_str, details in events:
    if ev_type == "LIDAR GAP":
        print(f"\033[91m{rel_t:<12.2f} | {ev_type:<12} | {val_str:<18} | {'--':<8} | {'--':<8} | {'--':<8} | {details}\033[0m")
    else:
        # Extract X Y Z for neat formatting
        x_str = details.split(", ")[0].split("=")[1]
        y_str = details.split(", ")[1].split("=")[1]
        z_str = details.split(", ")[2].split("=")[1]
        print(f"{rel_t:<12.2f} | {ev_type:<12} | {val_str:<18} | {x_str:<8} | {y_str:<8} | {z_str:<8} | {details}")

print("\n======================================================================")
