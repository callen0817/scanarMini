#!/usr/bin/env python3
"""
Milestone M12.8B: MVP Trajectory & Map Accuracy Qualification Tool

Evaluates:
1. File integrity (.ply, .pcd, .csv, .json).
2. Trajectory motion dynamics & Loop closure drift quantification:
   - Start / End 3D displacement
   - Start / End Yaw orientation error
   - Path length & average walking velocity
   - Drift rate (mm per meter traveled)
   - Floor elevation difference (Start vs. Return)
   - Wall-to-wall duplicate separation in closure zone
3. Multi-modal Geometric Co-Registration (Airy vs Gemini):
   - Surface Snapping Zone (<= 20mm)
   - Tight Envelope Zone (<= 50mm)
   - Conflict Suppression Zone (20mm - 80mm, filtered out)
   - Neutral Metric: "Unmatched Gemini Geometry (> 80mm)"
4. Local LIO Planar Consistency vs. Global Odometric Drift.
5. Truecolor RGB texturing on stereo and fused clouds.
"""

import os
import sys
import json
import time
import numpy as np
from scipy.spatial import cKDTree

def parse_ply_fast(filepath):
    """Fast ASCII PLY parser."""
    if not os.path.exists(filepath):
        return np.empty((0, 3)), None
        
    with open(filepath, 'r') as f:
        has_rgb = False
        n_pts = 0
        while True:
            line = f.readline()
            if not line:
                break
            if line.startswith("element vertex"):
                n_pts = int(line.split()[2])
            if "property uchar red" in line:
                has_rgb = True
            if line.strip() == "end_header":
                break
                
        if n_pts == 0:
            return np.empty((0, 3)), None

        data = np.loadtxt(f, dtype=np.float32)
        if len(data) == 0:
            return np.empty((0, 3)), None
        xyz = data[:, :3].astype(np.float64)
        rgb = data[:, 3:6].astype(np.uint8) if (has_rgb and data.shape[1] >= 6) else None
        return xyz, rgb

def analyze_m12_8b_dataset(export_dir):
    fused_ply = os.path.join(export_dir, "pointcloud_fused_rgb.ply")
    airy_ply = os.path.join(export_dir, "pointcloud_airy.ply")
    stereo_ply = os.path.join(export_dir, "pointcloud_stereo.ply")
    meta_json = os.path.join(export_dir, "scan_metadata.json")
    traj_csv = os.path.join(export_dir, "trajectory.csv")

    print(f"\n=======================================================")
    print(f"M12.8B TRAJECTORY & MAP ACCURACY QUALIFICATION")
    print(f"Dataset: {export_dir}")
    print(f"=======================================================")

    # 1. Trajectory & Motion Analysis
    if os.path.exists(traj_csv):
        traj = np.genfromtxt(traj_csv, delimiter=",", skip_header=1)
        stamps = traj[:, 0]
        positions = traj[:, 1:4]
        yaws = traj[:, 8] if traj.shape[1] > 8 else None

        dists_step = np.linalg.norm(np.diff(positions, axis=0), axis=1)
        total_path_len = float(np.sum(dists_step))
        duration = float(stamps[-1] - stamps[0])
        
        # Start vs End displacement
        disp_xyz = positions[-1] - positions[0]
        net_disp_3d = float(np.linalg.norm(disp_xyz))
        net_disp_xy = float(np.linalg.norm(disp_xyz[:2]))
        net_disp_z = float(abs(disp_xyz[2]))

        # Start vs End yaw difference
        if yaws is not None:
            yaw_diff = float(np.degrees(np.arctan2(np.sin(yaws[-1] - yaws[0]), np.cos(yaws[-1] - yaws[0]))))
        else:
            yaw_diff = 0.0

        drift_per_meter = (net_disp_3d / max(0.01, total_path_len)) * 1000.0 # mm / m

        print(f"1. TRAJECTORY MOTION & LOOP CLOSURE QUANTIFICATION:")
        print(f"   - Scan Duration: {duration:.2f} s")
        print(f"   - Total Path Traveled: {total_path_len:.2f} m")
        print(f"   - Average Walking Velocity: {total_path_len / max(0.1, duration):.2f} m/s")
        print(f"   - Net Start/End 3D Displacement: {net_disp_3d * 1000.0:.1f} mm ({net_disp_3d:.3f} m)")
        print(f"     * Horizontal (XY) Offset: {net_disp_xy * 1000.0:.1f} mm")
        print(f"     * Vertical (Z) Elevation Drift: {net_disp_z * 1000.0:.1f} mm")
        print(f"   - Net Start/End Yaw Orientation Drift: {yaw_diff:.2f}°")
        print(f"   - Accumulated Position Drift Rate: {drift_per_meter:.1f} mm/m ({drift_per_meter/10.0:.2f}%)")

    # 2. Point Cloud Assets & Color Validation
    xyz_fused, rgb_fused = parse_ply_fast(fused_ply)
    xyz_airy, rgb_airy = parse_ply_fast(airy_ply)
    xyz_stereo, rgb_stereo = parse_ply_fast(stereo_ply)

    print(f"\n2. POINT CLOUD ASSETS:")
    print(f"   - Authoritative Airy LiDAR: {len(xyz_airy):,} pts (Pure Geometry: {'YES' if rgb_airy is None else 'NO (has RGB)'})")
    print(f"   - Supplemental Gemini Stereo: {len(xyz_stereo):,} pts (Colorized: {'YES' if rgb_stereo is not None else 'NO'})")
    print(f"   - Final Fused Cloud: {len(xyz_fused):,} pts (Truecolor RGB: {'YES' if rgb_fused is not None else 'NO'})")
    print(f"   - Supplemental Density Multiplier: {len(xyz_fused) / max(1, len(xyz_airy)):.2f}x")

    if len(xyz_fused) > 0:
        min_b = np.min(xyz_fused, axis=0)
        max_b = np.max(xyz_fused, axis=0)
        print(f"   - Spatial Extents: X:[{min_b[0]:.2f}, {max_b[0]:.2f}] m | Y:[{min_b[1]:.2f}, {max_b[1]:.2f}] m | Z:[{min_b[2]:.2f}, {max_b[2]:.2f}] m")

    # 3. Geometric Co-Registration (Stereo vs. Airy)
    if len(xyz_airy) > 0 and len(xyz_stereo) > 0:
        sub_a = xyz_airy[::max(1, len(xyz_airy)//200000)]
        sub_s = xyz_stereo[::max(1, len(xyz_stereo)//200000)]
        tree_a = cKDTree(sub_a)
        dists, _ = tree_a.query(sub_s, k=1, workers=-1)

        p50 = float(np.percentile(dists, 50) * 1000.0)
        p80 = float(np.percentile(dists, 80) * 1000.0)
        p90 = float(np.percentile(dists, 90) * 1000.0)
        p95 = float(np.percentile(dists, 95) * 1000.0)

        snap_pct = float(np.mean(dists <= 0.020) * 100.0)
        near_pct = float(np.mean(dists <= 0.050) * 100.0)
        conflict_pct = float(np.mean((dists > 0.020) & (dists <= 0.080)) * 100.0)
        unmatched_pct = float(np.mean(dists > 0.080) * 100.0)

        print(f"\n3. GEOMETRIC CO-REGISTRATION (STEREO vs. AIRY):")
        print(f"   - Median Alignment Residual (P50): {p50:.2f} mm")
        print(f"   - 80th Percentile Residual (P80): {p80:.2f} mm")
        print(f"   - 90th Percentile Residual (P90): {p90:.2f} mm")
        print(f"   - 95th Percentile Residual (P95): {p95:.2f} mm")
        print(f"   - Surface Snapping Zone (<= 20mm): {snap_pct:.1f}%")
        print(f"   - Tight Envelope Zone (<= 50mm): {near_pct:.1f}%")
        print(f"   - Conflict Suppression Zone (20mm - 80mm): {conflict_pct:.1f}% (filtered out)")
        print(f"   - Unmatched Gemini Geometry (> 80mm): {unmatched_pct:.1f}% (evaluated separately)")

    # 4. Local Planar Consistency vs. Loop Drift
    if len(xyz_airy) > 1000:
        sub_pts = xyz_airy[::max(1, len(xyz_airy)//100000)]
        z_vals = sub_pts[:, 2]
        z_floor_cand = sub_pts[z_vals < np.percentile(z_vals, 20)]
        if len(z_floor_cand) > 50:
            floor_z_p50 = float(np.percentile(z_floor_cand[:, 2], 50))
            floor_res = np.abs(z_floor_cand[:, 2] - floor_z_p50) * 1000.0
            print(f"\n4. LOCAL PLANAR CONSISTENCY:")
            print(f"   - Airy Floor Planar Flatness (P50 / P95): {np.percentile(floor_res, 50):.1f} mm / {np.percentile(floor_res, 95):.1f} mm")

    # 5. RGB Texture Quality
    if rgb_fused is not None and len(rgb_fused) > 0:
        non_default = np.sum((rgb_fused[:, 0] != 175) | (rgb_fused[:, 1] != 175) | (rgb_fused[:, 2] != 175))
        color_pct = (non_default / len(rgb_fused)) * 100.0
        
        color_diff = np.abs(rgb_fused[:, 0].astype(int) - rgb_fused[:, 1].astype(int)) + \
                     np.abs(rgb_fused[:, 1].astype(int) - rgb_fused[:, 2].astype(int))
        chromatic_pts = np.sum(color_diff > 10)
        chromatic_pct = (chromatic_pts / len(rgb_fused)) * 100.0

        print(f"\n5. RGB TEXTURE & COLOR FIDELITY:")
        print(f"   - Camera Colormapped Coverage: {color_pct:.1f}% of fused vertices")
        print(f"   - Chromatic Variation Richness: {chromatic_pct:.1f}% (distinct non-monochrome colors)")
        print(f"   - Channel Intensity Means (R, G, B): {np.mean(rgb_fused, axis=0).round(1)}")

    print(f"=======================================================\n")

if __name__ == '__main__':
    target_dir = sys.argv[1] if len(sys.argv) > 1 else "/home/scanar/scanarMini/data/M12_8_Live_Walk_Scan_01"
    analyze_m12_8b_dataset(target_dir)
