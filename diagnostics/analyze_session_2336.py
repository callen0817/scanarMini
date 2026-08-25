#!/usr/bin/env python3
"""
Session Scan_0804_2336_20260804_233828 Analysis Script.
Evaluates pose stability, max translation drift, step sizes, and LiDAR gap correlation.
"""

import sys
import os
import csv
import math

session_dir = "/home/scanar/scanR/diagnostics/Scan_0804_2336_20260804_233828"
slam_csv = os.path.join(session_dir, "slam.csv")
lidar_csv = os.path.join(session_dir, "lidar.csv")

print("======================================================================")
print(f"SESSION ANALYSIS: {os.path.basename(session_dir)}")
print("======================================================================")

# 1. Parse SLAM Poses
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
            roll = float(row["roll"])
            pitch = float(row["pitch"])
            yaw = float(row["yaw"])
            slam_poses.append((sys_t, pose_t, x, y, z, roll, pitch, yaw))

print(f"Total SLAM Poses Recorded: {len(slam_poses)}")

if slam_poses:
    t0 = slam_poses[0][1]
    tf = slam_poses[-1][1]
    dur = tf - t0
    
    x0, y0, z0 = slam_poses[0][2], slam_poses[0][3], slam_poses[0][4]
    xf, yf, zf = slam_poses[-1][2], slam_poses[-1][3], slam_poses[-1][4]
    
    tot_drift = math.sqrt((xf - x0)**2 + (yf - y0)**2 + (zf - z0)**2)
    
    max_dist_from_origin = 0.0
    max_step_dist = 0.0
    large_step_count = 0
    
    for i in range(1, len(slam_poses)):
        _, pt, x, y, z, _, _, _ = slam_poses[i]
        _, _, px, py, pz, _, _, _ = slam_poses[i-1]
        
        dist_orig = math.sqrt((x - x0)**2 + (y - y0)**2 + (z - z0)**2)
        if dist_orig > max_dist_from_origin:
            max_dist_from_origin = dist_orig
            
        step_dist = math.sqrt((x - px)**2 + (y - py)**2 + (z - pz)**2)
        if step_dist > max_step_dist:
            max_step_dist = step_dist
            
        if step_dist > 1.0: # Step size > 1.0m per frame
            large_step_count += 1

    print(f"\n1. SLAM POSE TRAJECTORY METRICS:")
    print(f"   * Total Session Duration:    {dur:.2f} seconds")
    print(f"   * Start Pose:                X={x0:.3f}m, Y={y0:.3f}m, Z={z0:.3f}m")
    print(f"   * Final Pose:                X={xf:.3f}m, Y={yf:.3f}m, Z={zf:.3f}m")
    print(f"   * Net Translation Drift:     {tot_drift:.3f} meters")
    print(f"   * Max Distance from Origin:  {max_dist_from_origin:.3f} meters")
    print(f"   * Max Step Size Per Frame:   {max_step_dist:.3f} meters")
    print(f"   * Large Step Count (>1.0m):  {large_step_count}")

# 2. Parse LiDAR Scans
lidar_scans = []
if os.path.exists(lidar_csv):
    with open(lidar_csv, "r") as f:
        reader = csv.DictReader(f)
        for row in reader:
            sys_t = float(row["sys_time"])
            hdr_t = float(row["sensor_sec"])
            pts = int(row["point_count"])
            lidar_scans.append((sys_t, hdr_t, pts))

print(f"\n2. LIDAR SCAN METRICS:")
print(f"   * Total Scans Received:      {len(lidar_scans)}")

if len(lidar_scans) > 1:
    l_dur = lidar_scans[-1][1] - lidar_scans[0][1]
    gaps_200ms = 0
    gaps_300ms = 0
    gaps_400ms = 0
    gaps_gt400ms = 0
    
    for i in range(1, len(lidar_scans)):
        delta_ms = (lidar_scans[i][1] - lidar_scans[i-1][1]) * 1000.0
        if 150.0 <= delta_ms < 250.0:
            gaps_200ms += 1
        elif 250.0 <= delta_ms < 350.0:
            gaps_300ms += 1
        elif 350.0 <= delta_ms < 450.0:
            gaps_400ms += 1
        elif delta_ms >= 450.0:
            gaps_gt400ms += 1

    print(f"   * Effective Scan Rate:       {len(lidar_scans)/l_dur:.2f} Hz")
    print(f"   * Gap Breakdown:")
    print(f"       - 200ms Gaps (1 scan dropped):  {gaps_200ms}")
    print(f"       - 300ms Gaps (2 scans dropped): {gaps_300ms}")
    print(f"       - 400ms Gaps (3 scans dropped): {gaps_400ms}")
    print(f"       - >450ms Blackout Gaps:         {gaps_gt400ms}")

print("\n======================================================================")
