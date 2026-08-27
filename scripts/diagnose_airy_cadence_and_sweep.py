#!/usr/bin/env python3
"""
Comprehensive 60-Second Airy LiDAR Frame Cadence and 360-Degree Sweep Validation Script
Computes:
1. Header Timestamp vs Arrival Timestamp interval distributions:
   P1, P5, P25, P50, P75, P95, P99, min, max, <75ms, 75-125ms, >125ms, >200ms
2. Complete 360-degree sweep geometric validation across 100+ consecutive clouds:
   - Point count
   - Start azimuth, End azimuth, Angular span (deg)
   - Sector coverage across 36 bins (10 deg each) to check for gaps/duplicates
3. FAST-LIVO2 registered cloud (/cloud_registered) cadence distribution
"""

import os
import sys
import time
import math
import json
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2

class AiryCadenceValidator(Node):
    def __init__(self):
        super().__init__('airy_cadence_validator')
        self.raw_clouds = []      # (arrival_t, header_t, pts_xyz)
        self.reg_clouds = []      # (arrival_t, header_t)
        
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=200)
        self.sub_raw = self.create_subscription(PointCloud2, '/rslidar_points', self.raw_cb, qos)
        self.sub_reg = self.create_subscription(PointCloud2, '/cloud_registered', self.reg_cb, qos)

    def raw_cb(self, msg: PointCloud2):
        arr_t = time.perf_counter()
        hdr_t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        # Extract points for sweep geometry check
        if msg.width * msg.height == 0 or msg.point_step < 12:
            return
        raw_bytes = bytes(msg.data)
        dt = np.dtype({
            'names': ['x', 'y', 'z'],
            'formats': ['<f4', '<f4', '<f4'],
            'offsets': [0, 4, 8],
            'itemsize': msg.point_step
        })
        pts = np.frombuffer(raw_bytes, dtype=dt)
        valid = np.isfinite(pts['x']) & np.isfinite(pts['y']) & np.isfinite(pts['z'])
        pts_v = pts[valid]
        xyz = np.column_stack((pts_v['x'], pts_v['y'], pts_v['z'])).astype(np.float32) if len(pts_v) > 0 else np.empty((0, 3))
        self.raw_clouds.append((arr_t, hdr_t, xyz))

    def reg_cb(self, msg: PointCloud2):
        arr_t = time.perf_counter()
        hdr_t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.reg_clouds.append((arr_t, hdr_t))

def compute_interval_distribution(timestamps_sec):
    if len(timestamps_sec) < 2:
        return {}
    diffs_ms = np.diff(timestamps_sec) * 1000.0
    return {
        "count": int(len(timestamps_sec)),
        "mean_ms": float(round(np.mean(diffs_ms), 2)),
        "std_ms": float(round(np.std(diffs_ms), 2)),
        "min_ms": float(round(np.min(diffs_ms), 2)),
        "p1_ms": float(round(np.percentile(diffs_ms, 1), 2)),
        "p5_ms": float(round(np.percentile(diffs_ms, 5), 2)),
        "p25_ms": float(round(np.percentile(diffs_ms, 25), 2)),
        "p50_ms": float(round(np.percentile(diffs_ms, 50), 2)),
        "p75_ms": float(round(np.percentile(diffs_ms, 75), 2)),
        "p95_ms": float(round(np.percentile(diffs_ms, 95), 2)),
        "p99_ms": float(round(np.percentile(diffs_ms, 99), 2)),
        "max_ms": float(round(np.max(diffs_ms), 2)),
        "bins": {
            "under_75ms": int(np.sum(diffs_ms < 75.0)),
            "between_75_125ms": int(np.sum((diffs_ms >= 75.0) & (diffs_ms <= 125.0))),
            "between_125_200ms": int(np.sum((diffs_ms > 125.0) & (diffs_ms <= 200.0))),
            "over_200ms": int(np.sum(diffs_ms > 200.0))
        }
    }

def run_60s_validation(duration=60.0):
    rclpy.init()
    node = AiryCadenceValidator()
    print(f"\n[AIRY VALIDATION] Collecting LiDAR streams for {duration:.1f} seconds...")
    t0 = time.time()
    while (time.time() - t0) < duration:
        rclpy.spin_once(node, timeout_sec=0.01)

    print(f"[AIRY VALIDATION] Collection complete. Processing {len(node.raw_clouds)} raw clouds, {len(node.reg_clouds)} registered clouds...")
    node.destroy_node()
    rclpy.shutdown()

    # 1. Raw LiDAR Cadence (Arrival vs Header)
    raw_arr = [c[0] for c in node.raw_clouds]
    raw_hdr = [c[1] for c in node.raw_clouds]
    raw_arr_dist = compute_interval_distribution(raw_arr)
    raw_hdr_dist = compute_interval_distribution(raw_hdr)

    # 2. Registered Cloud Cadence (Arrival vs Header)
    reg_arr = [c[0] for c in node.reg_clouds]
    reg_hdr = [c[1] for c in node.reg_clouds]
    reg_arr_dist = compute_interval_distribution(reg_arr)
    reg_hdr_dist = compute_interval_distribution(reg_hdr)

    # 3. Geometric 360-Degree Sweep Validation across consecutive clouds
    sweep_records = []
    clouds_to_check = node.raw_clouds[-120:] if len(node.raw_clouds) >= 120 else node.raw_clouds
    for i in range(1, len(clouds_to_check)):
        prev_hdr = clouds_to_check[i-1][1]
        curr_hdr = clouds_to_check[i][1]
        dt_hdr_ms = (curr_hdr - prev_hdr) * 1000.0
        xyz = clouds_to_check[i][2]
        n_pts = len(xyz)
        if n_pts > 0:
            azimuths_rad = np.arctan2(xyz[:, 1], xyz[:, 0]) # -pi to +pi
            azimuths_deg = (np.degrees(azimuths_rad) + 360.0) % 360.0 # 0 to 360
            az_min = float(np.min(azimuths_deg))
            az_max = float(np.max(azimuths_deg))
            az_span = float(az_max - az_min)
            # Bin into 36 bins of 10 deg
            hist, _ = np.histogram(azimuths_deg, bins=36, range=(0.0, 360.0))
            empty_sectors = int(np.sum(hist == 0))
            first_az = float(azimuths_deg[0])
            last_az = float(azimuths_deg[-1])
        else:
            az_min, az_max, az_span, empty_sectors, first_az, last_az = 0, 0, 0, 36, 0, 0

        sweep_records.append({
            "cloud_idx": i,
            "dt_hdr_ms": round(dt_hdr_ms, 2),
            "point_count": int(n_pts),
            "first_az_deg": round(first_az, 1),
            "last_az_deg": round(last_az, 1),
            "az_min_deg": round(az_min, 1),
            "az_max_deg": round(az_max, 1),
            "angular_span_deg": round(az_span, 1),
            "empty_10deg_sectors": empty_sectors
        })

    # Summary of Sweep Geometry
    pts_arr = [s["point_count"] for s in sweep_records]
    spans = [s["angular_span_deg"] for s in sweep_records]
    empty_secs = [s["empty_10deg_sectors"] for s in sweep_records]

    report = {
        "duration_seconds": duration,
        "raw_lidar_points_cadence": {
            "total_messages": len(node.raw_clouds),
            "average_rate_hz": round(len(node.raw_clouds) / duration, 2),
            "header_timestamp_distribution": raw_hdr_dist,
            "arrival_timestamp_distribution": raw_arr_dist
        },
        "cloud_registered_cadence": {
            "total_messages": len(node.reg_clouds),
            "average_rate_hz": round(len(node.reg_clouds) / duration, 2),
            "header_timestamp_distribution": reg_hdr_dist,
            "arrival_timestamp_distribution": reg_arr_dist
        },
        "sweep_geometry_validation": {
            "consecutive_clouds_analyzed": len(sweep_records),
            "mean_point_count": float(round(np.mean(pts_arr), 1)) if len(pts_arr) else 0,
            "p50_point_count": int(np.percentile(pts_arr, 50)) if len(pts_arr) else 0,
            "mean_angular_span_deg": float(round(np.mean(spans), 2)) if len(spans) else 0,
            "p50_angular_span_deg": float(round(np.percentile(spans, 50), 2)) if len(spans) else 0,
            "max_empty_10deg_sectors": int(np.max(empty_secs)) if len(empty_secs) else 36,
            "complete_360_sweep_status": "PASS" if len(spans) and np.percentile(spans, 50) > 350.0 and np.max(empty_secs) <= 1 else "FAIL",
            "sample_records_first_10": sweep_records[:10]
        }
    }

    print("\n" + "="*85)
    print("           60-SECOND AIRY LIDAR CADENCE & SWEEP GEOMETRY REPORT          ")
    print("="*85)
    print(json.dumps(report, indent=2))
    print("="*85)

    with open("/home/scanar/scanarMini/data/airy_cadence_report.json", 'w') as f:
        json.dump(report, f, indent=2)

    return report

if __name__ == '__main__':
    dur = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
    run_60s_validation(dur)
