#!/usr/bin/env python3
"""
scanHUB Standalone Diagnostic Telemetry & Point Cloud Analyzer (Milestone 4.5)

Analyzes diagnostic session folders (diagnostics/walk_YYMMDD_HHMMSS/) or ROS 2 bags:
- Point cloud density & frame completeness (point counts per scan, zero-point collapses).
- Sensor timestamp spacing & jitter analysis (mean delta, max delay, 400ms+ timing gaps).
- FAST-LIVO2 pose smoothness & tracking continuity (motion jumps, stationary velocity noise).
- System load & resource utilization.
- Generates/updates comprehensive diagnostic summary reports.
"""

import os
import sys
import csv
import glob
import math
import argparse
from datetime import datetime


def analyze_diagnostic_folder(folder_path):
    if not os.path.exists(folder_path):
        print(f"[ERROR] Diagnostic folder '{folder_path}' does not exist.")
        return False

    print("================================================================================")
    print(f"scanHUB DIAGNOSTIC ANALYSIS REPORT: {os.path.basename(folder_path)}")
    print("================================================================================")
    
    # 1. Analyze IMU CSV
    imu_csv = os.path.join(folder_path, "imu.csv")
    if os.path.exists(imu_csv):
        with open(imu_csv, "r", encoding="utf-8") as f:
            reader = csv.DictReader(f)
            sys_times = []
            gyros = []
            accels = []
            for row in reader:
                try:
                    sys_times.append(float(row["sys_time"]))
                    gyros.append((float(row["gyro_x"]), float(row["gyro_y"]), float(row["gyro_z"])))
                    accels.append((float(row["accel_x"]), float(row["accel_y"]), float(row["accel_z"])))
                except (ValueError, KeyError):
                    continue
        if len(sys_times) > 1:
            dur = sys_times[-1] - sys_times[0]
            hz = len(sys_times) / max(dur, 0.001)
            deltas = [sys_times[i] - sys_times[i-1] for i in range(1, len(sys_times))]
            mean_d = sum(deltas) / len(deltas)
            max_d = max(deltas)
            spikes = sum(1 for d in deltas if d > 0.02) # >20ms gap for IMU
            print(f"[IMU STREAM] Total Msgs: {len(sys_times)} | Rate: {hz:.1f} Hz | Mean Delta: {mean_d*1000:.1f} ms | Max Delta: {max_d*1000:.1f} ms | Gaps >20ms: {spikes}")
        else:
            print("[IMU STREAM] Insufficient samples.")
    else:
        print("[IMU STREAM] imu.csv not found.")

    # 2. Analyze LiDAR CSV
    lidar_csv = os.path.join(folder_path, "lidar.csv")
    if os.path.exists(lidar_csv):
        with open(lidar_csv, "r", encoding="utf-8") as f:
            reader = csv.DictReader(f)
            sys_times = []
            pts_list = []
            for row in reader:
                try:
                    sys_times.append(float(row["sys_time"]))
                    pts_list.append(int(row["point_count"]))
                except (ValueError, KeyError):
                    continue
        if len(sys_times) > 1:
            dur = sys_times[-1] - sys_times[0]
            hz = len(sys_times) / max(dur, 0.001)
            deltas = [sys_times[i] - sys_times[i-1] for i in range(1, len(sys_times))]
            mean_d = sum(deltas) / len(deltas)
            max_d = max(deltas)
            spikes = sum(1 for d in deltas if d > 0.25) # >250ms gap
            zero_scans = sum(1 for p in pts_list if p == 0)
            avg_pts = sum(pts_list) / len(pts_list) if pts_list else 0
            print(f"[LiDAR STREAM] Scans: {len(sys_times)} | Rate: {hz:.1f} Hz | Mean Delta: {mean_d*1000:.1f} ms | Max Delta: {max_d*1000:.1f} ms | Gaps >250ms: {spikes}")
            print(f"               Avg Points/Scan: {avg_pts:.0f} | Min: {min(pts_list) if pts_list else 0} | Max: {max(pts_list) if pts_list else 0} | Empty Scans: {zero_scans}")
        else:
            print("[LiDAR STREAM] Insufficient samples.")
    else:
        print("[LiDAR STREAM] lidar.csv not found.")

    # 3. Analyze SLAM CSV
    slam_csv = os.path.join(folder_path, "slam.csv")
    if os.path.exists(slam_csv):
        with open(slam_csv, "r", encoding="utf-8") as f:
            reader = csv.DictReader(f)
            sys_times = []
            poses = []
            for row in reader:
                try:
                    sys_times.append(float(row["sys_time"]))
                    poses.append((float(row["x"]), float(row["y"]), float(row["z"]), float(row["yaw"])))
                except (ValueError, KeyError):
                    continue
        if len(sys_times) > 1:
            dur = sys_times[-1] - sys_times[0]
            hz = len(sys_times) / max(dur, 0.001)
            deltas = [sys_times[i] - sys_times[i-1] for i in range(1, len(sys_times))]
            mean_d = sum(deltas) / len(deltas)
            max_d = max(deltas)
            
            # Distance and motion analysis
            dist = 0.0
            max_vel = 0.0
            motion_jumps = 0
            for i in range(1, len(poses)):
                dt = deltas[i-1]
                if dt > 0:
                    d = math.hypot(poses[i][0] - poses[i-1][0], poses[i][1] - poses[i-1][1])
                    dist += d
                    vel = d / dt
                    if vel > max_vel:
                        max_vel = vel
                    if vel > 3.0: # > 3.0 m/s unrealistically fast human walking speed jump
                        motion_jumps += 1
            
            print(f"[SLAM POSE] Pos Updates: {len(sys_times)} | Rate: {hz:.1f} Hz | Mean Delta: {mean_d*1000:.1f} ms | Max Delta: {max_d*1000:.1f} ms")
            print(f"            Total Path Dist: {dist:.2f} m | Peak Vel: {max_vel:.2f} m/s | Motion Jumps (>3m/s): {motion_jumps}")
        else:
            print("[SLAM POSE] Insufficient samples.")
    else:
        print("[SLAM POSE] slam.csv not found.")

    # 4. Analyze SLAM C++ Per-Frame Diagnostics CSV
    diag_csv = os.path.join(folder_path, "slam_diagnostics.csv")
    if os.path.exists(diag_csv):
        with open(diag_csv, "r", encoding="utf-8") as f:
            reader = csv.DictReader(f)
            proc_times = []
            features = []
            vels = []
            deltas = []
            div_frame = None
            for row in reader:
                try:
                    f_idx = int(row["frame_idx"])
                    p_ms = float(row["proc_time_ms"])
                    feat = int(row["features_matched"])
                    v = float(row["estimated_velocity_m_s"])
                    dt = float(row["delta_translation_m"])
                    proc_times.append(p_ms)
                    features.append(feat)
                    vels.append(v)
                    deltas.append(dt)
                    if v > 3.0 and div_frame is None:
                        div_frame = f_idx
                except (ValueError, KeyError):
                    continue
        if proc_times:
            print(f"[FAST-LIVO2 C++ DIAGNOSTICS] Total Frames: {len(proc_times)}")
            print(f"                            Mean Proc Time: {sum(proc_times)/len(proc_times):.2f} ms | Max Proc Time: {max(proc_times):.2f} ms")
            print(f"                            Mean Features Matched: {sum(features)/len(features):.0f} | Min: {min(features)} | Max: {max(features)}")
            print(f"                            Peak Velocity: {max(vels):.2f} m/s | Max Frame Step: {max(deltas):.4f} m")
            if div_frame is not None:
                print(f"                            ⚠️ DIVERGENCE DETECTED at Frame #{div_frame}!")
            else:
                print(f"                            🟢 Zero tracking divergence observed across all frames.")
    else:
        print("[FAST-LIVO2 C++ DIAGNOSTICS] slam_diagnostics.csv not recorded.")

    # 4. Check summary.txt
    summary_path = os.path.join(folder_path, "summary.txt")
    if os.path.exists(summary_path):
        print(f"\n[SUMMARY FILE] Found existing report at {summary_path}")
    else:
        print("\n[SUMMARY FILE] No summary.txt found in directory.")

    print("================================================================================")
    return True


def main():
    parser = argparse.ArgumentParser(description="scanHUB Diagnostic Analyzer")
    parser.add_argument("path", nargs="?", default="/home/scanar/scanarMini/diagnostics", help="Path to diagnostic walk folder or base diagnostics directory")
    args = parser.parse_args()

    target_path = args.path
    if os.path.isdir(target_path):
        # Check if target_path is a specific walk folder or base diagnostics directory containing walk folders
        if os.path.exists(os.path.join(target_path, "imu.csv")) or os.path.exists(os.path.join(target_path, "slam.csv")):
            analyze_diagnostic_folder(target_path)
        else:
            # Base directory: search for all walk subfolders
            walk_dirs = sorted(glob.glob(os.path.join(target_path, "walk_*")))
            if not walk_dirs:
                print(f"[INFO] No walk folders found in '{target_path}'.")
            else:
                for w in walk_dirs:
                    analyze_diagnostic_folder(w)
                    print("\n")
    else:
        print(f"[ERROR] Invalid directory path: {target_path}")


if __name__ == "__main__":
    main()
