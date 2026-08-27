#!/usr/bin/env python3
"""
Tests and Validates M12.7 Three-Class Classifier on M12.4 Ground Truth Data
"""

import os
import sys
import numpy as np
from m12_7_dynamic_fusion_engine import execute_three_class_asps_fusion
from m12_5_forensic_validation import read_binary_ply, fit_plane_svd

def run_test():
    airy_file = "/home/scanar/scanarMini/data/M12_4_Characterization/m12_4_airy_cloud.ply"
    gemini_file = "/home/scanar/scanarMini/data/M12_4_Characterization/m12_4_gemini_cloud_rgb.ply"

    airy_xyz, _ = read_binary_ply(airy_file)
    gemini_xyz, gemini_rgb = read_binary_ply(gemini_file)

    print(f"Loaded: Airy={len(airy_xyz):,}, Gemini={len(gemini_xyz):,}")

    fused_xyz, fused_rgb, meta = execute_three_class_asps_fusion(
        airy_xyz, gemini_xyz, gemini_rgb,
        tau_snap_m=0.020,
        tau_conflict_m=0.080,
        r_search=0.15
    )

    print("\n--- Three-Class Classification Results ---")
    for k, v in meta.items():
        print(f"  {k}: {v}")

    # Wall Bounding Box Analysis
    wall_box = {'x': (-0.6, 0.6), 'y': (-2.5, -2.1), 'z': (-1.0, -0.2)}
    mask_a = (airy_xyz[:, 0] >= wall_box['x'][0]) & (airy_xyz[:, 0] <= wall_box['x'][1]) & \
             (airy_xyz[:, 1] >= wall_box['y'][0]) & (airy_xyz[:, 1] <= wall_box['y'][1]) & \
             (airy_xyz[:, 2] >= wall_box['z'][0]) & (airy_xyz[:, 2] <= wall_box['z'][1])
    mask_f = (fused_xyz[:, 0] >= wall_box['x'][0]) & (fused_xyz[:, 0] <= wall_box['x'][1]) & \
             (fused_xyz[:, 1] >= wall_box['y'][0]) & (fused_xyz[:, 1] <= wall_box['y'][1]) & \
             (fused_xyz[:, 2] >= wall_box['z'][0]) & (fused_xyz[:, 2] <= wall_box['z'][1])

    a_wall = airy_xyz[mask_a]
    f_wall = fused_xyz[mask_f]

    norm_a, d_a, cent_a, sigma_a, res_a = fit_plane_svd(a_wall)
    norm_f, d_f, cent_f, sigma_f, res_f = fit_plane_svd(f_wall)

    abs_res_f = np.abs(res_f) * 1000.0
    p50_thick = np.percentile(abs_res_f, 50) * 2.0
    p90_thick = np.percentile(abs_res_f, 90) * 2.0
    p95_thick = np.percentile(abs_res_f, 95) * 2.0

    print("\n--- Planar Ground Truth Comparison on Forward Wall ---")
    print(f"Airy Alone:      {len(a_wall):,} pts | Sigma = {sigma_a*1000:.2f} mm | P95 Thickness = {np.percentile(np.abs(res_a)*1000, 95)*2:.2f} mm")
    print(f"M12.7 Fused Wall: {len(f_wall):,} pts | Sigma = {sigma_f*1000:.2f} mm | P95 Thickness = {p95_thick:.2f} mm")

if __name__ == '__main__':
    run_test()
