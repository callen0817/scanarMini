#!/usr/bin/env python3
"""
Analyzes stationary SLAM drift, high-frequency pose jitter, and coordinate variance
from the stationary baseline session.
"""

import os
import sys
import math
import numpy as np
import json

def analyze_drift(csv_path):
    if not os.path.exists(csv_path):
        print(f"Error: {csv_path} not found.")
        return

    data = np.genfromtxt(csv_path, delimiter=',', skip_header=1)
    # columns: index,timestamp,x_m,y_m,z_m,qx,qy,qz,qw,yaw_deg
    t = data[:, 1]
    xyz = data[:, 2:5]
    yaw = data[:, 9]
    duration = t[-1] - t[0]

    # Consecutive step lengths
    diffs = np.diff(xyz, axis=0)
    step_lens = np.linalg.norm(diffs, axis=1)
    accum_path_m = np.sum(step_lens)

    # Excursion from start pose
    p0 = xyz[0]
    excursions_from_origin = np.linalg.norm(xyz - p0, axis=1)
    max_excursion_m = np.max(excursions_from_origin)
    net_displacement_m = np.linalg.norm(xyz[-1] - p0)

    # Coordinate variance / std dev
    sigma_xyz = np.std(xyz, axis=0) * 1000.0 # mm
    yaw_drift_deg = abs(yaw[-1] - yaw[0])
    yaw_std_deg = np.std(yaw)

    results = {
        "duration_seconds": float(round(duration, 2)),
        "total_poses": int(len(xyz)),
        "pose_rate_hz": float(round(len(xyz) / duration, 2)),
        "accumulated_path_jitter_mm": float(round(accum_path_m * 1000.0, 2)),
        "jitter_rate_mm_per_sec": float(round((accum_path_m * 1000.0) / duration, 2)),
        "mean_step_jitter_mm": float(round(np.mean(step_lens) * 1000.0, 3)),
        "net_start_to_end_displacement_mm": float(round(net_displacement_m * 1000.0, 2)),
        "max_excursion_from_start_mm": float(round(max_excursion_m * 1000.0, 2)),
        "coordinate_std_dev_mm": {
            "sigma_x_mm": float(round(sigma_xyz[0], 2)),
            "sigma_y_mm": float(round(sigma_xyz[1], 2)),
            "sigma_z_mm": float(round(sigma_xyz[2], 2))
        },
        "yaw_start_to_end_drift_deg": float(round(yaw_drift_deg, 3)),
        "yaw_std_dev_deg": float(round(yaw_std_deg, 3))
    }

    print("============================================================")
    print("   STATIONARY SLAM DRIFT & HIGH-FREQUENCY JITTER TELEMETRY  ")
    print("============================================================")
    print(json.dumps(results, indent=2))
    return results

if __name__ == '__main__':
    csv_file = "/home/scanar/scanarMini/data/M12_7_Static_Drift_Observation/trajectory.csv"
    analyze_drift(csv_file)
