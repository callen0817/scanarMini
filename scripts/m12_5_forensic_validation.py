#!/usr/bin/env python3
"""
Milestone M12.5: Forensic Validation of Geometric Fusion Metrics
Performs exhaustive mathematical reconciliation of surface statistics, plane fits,
point-to-plane vs Euclidean residuals, and voxel occupancy across fusion policies.
"""

import os
import sys
import math
import json
import numpy as np
from scipy.spatial import cKDTree

def read_binary_ply(ply_path):
    with open(ply_path, 'rb') as f:
        header = b""
        while True:
            line = f.readline()
            header += line
            if line.strip() == b"end_header":
                break
        raw_data = f.read()

    header_str = header.decode('ascii', errors='ignore')
    has_rgb = "uchar red" in header_str or "uchar r" in header_str
    
    if has_rgb:
        dt = np.dtype([
            ('x', '<f4'), ('y', '<f4'), ('z', '<f4'),
            ('r', 'u1'), ('g', 'u1'), ('b', 'u1')
        ])
        arr = np.frombuffer(raw_data, dtype=dt)
        xyz = np.column_stack((arr['x'], arr['y'], arr['z'])).astype(np.float64)
        rgb = np.column_stack((arr['r'], arr['g'], arr['b'])).astype(np.uint8)
        return xyz, rgb
    else:
        dt = np.dtype([
            ('x', '<f4'), ('y', '<f4'), ('z', '<f4')
        ])
        arr = np.frombuffer(raw_data, dtype=dt)
        xyz = np.column_stack((arr['x'], arr['y'], arr['z'])).astype(np.float64)
        return xyz, None

def fit_plane_svd(pts):
    """
    Fits least-squares plane via SVD / PCA covariance decomposition.
    Returns: normal (normalized, y>0), intercept d, centroid, sigma (std dev of residuals), residuals.
    """
    centroid = np.mean(pts, axis=0)
    cov = np.cov((pts - centroid).T)
    evals, evecs = np.linalg.eigh(cov)
    normal = evecs[:, 0]
    if normal[1] < 0:
        normal = -normal
    d = -np.dot(normal, centroid)
    residuals = np.dot(pts, normal) + d
    sigma = np.std(residuals)
    return normal, d, centroid, sigma, residuals

def precompute_airy_tangents(airy_xyz, gemini_xyz, r_search=0.15, min_pts=6):
    """
    Precomputes local tangent planes for unique matched Airy anchor points using radius search.
    """
    tree = cKDTree(airy_xyz)
    dists_euclid, idxs_a = tree.query(gemini_xyz, k=1, workers=4)
    unique_uids = np.unique(idxs_a)
    ball_idxs = tree.query_ball_point(airy_xyz[unique_uids], r=r_search, workers=4)
    tangents = {}
    for i, uid in enumerate(unique_uids):
        idxs = ball_idxs[i]
        if len(idxs) >= min_pts:
            sub = airy_xyz[idxs]
            cent = np.mean(sub, axis=0)
            cov = np.cov((sub - cent).T)
            evals, evecs = np.linalg.eigh(cov)
            norm = evecs[:, 0]
            if evals[0] < 0.008: # Clean planar surface
                if np.dot(norm, cent) > 0:
                    norm = -norm
                tangents[uid] = (norm, cent, True)
            else:
                tangents[uid] = (norm, cent, False)
        else:
            tangents[uid] = (np.array([0., 1., 0.]), airy_xyz[uid], False)
    return tree, tangents, dists_euclid, idxs_a

def run_forensic_analysis():
    airy_file = "/home/scanar/scanarMini/data/M12_4_Characterization/m12_4_airy_cloud.ply"
    gemini_file = "/home/scanar/scanarMini/data/M12_4_Characterization/m12_4_gemini_cloud_rgb.ply"

    print("================================================================================")
    print("M12.5 FORENSIC METRIC VALIDATION & MATHEMATICAL RECONCILIATION")
    print("================================================================================")
    airy_xyz, _ = read_binary_ply(airy_file)
    gemini_xyz, gemini_rgb = read_binary_ply(gemini_file)
    n_airy = len(airy_xyz)
    n_gemini = len(gemini_xyz)

    print(f"Loaded Raw Clouds: Airy = {n_airy:,} points, Gemini = {n_gemini:,} points.")

    # Build Airy KDTree and precompute local tangent planes on unique anchors
    airy_tree, tangents, dists_euclid, idxs_a = precompute_airy_tangents(airy_xyz, gemini_xyz, r_search=0.15)

    # Global Wall Bounding Box for planar evaluation
    wall_box = {'x': (-0.6, 0.6), 'y': (-2.5, -2.1), 'z': (-1.0, -0.2)}
    mask_a_wall = (airy_xyz[:, 0] >= wall_box['x'][0]) & (airy_xyz[:, 0] <= wall_box['x'][1]) & \
                  (airy_xyz[:, 1] >= wall_box['y'][0]) & (airy_xyz[:, 1] <= wall_box['y'][1]) & \
                  (airy_xyz[:, 2] >= wall_box['z'][0]) & (airy_xyz[:, 2] <= wall_box['z'][1])
    mask_g_wall = (gemini_xyz[:, 0] >= wall_box['x'][0]) & (gemini_xyz[:, 0] <= wall_box['x'][1]) & \
                  (gemini_xyz[:, 1] >= wall_box['y'][0]) & (gemini_xyz[:, 1] <= wall_box['y'][1]) & \
                  (gemini_xyz[:, 2] >= wall_box['z'][0]) & (gemini_xyz[:, 2] <= wall_box['z'][1])

    a_wall_pts = airy_xyz[mask_a_wall]
    g_wall_pts = gemini_xyz[mask_g_wall]

    # Baseline Plane Fits
    norm_a, d_a, cent_a, sigma_a, res_a = fit_plane_svd(a_wall_pts)
    norm_g, d_g, cent_g, sigma_g, res_g = fit_plane_svd(g_wall_pts)

    signed_plane_offset = (d_g - d_a) * 1000.0 # mm along normal
    normal_angle_deg = math.degrees(math.acos(np.clip(np.dot(norm_a, norm_g), -1.0, 1.0)))

    print(f"\n[1] Wall Baseline Planar Ground Truth:")
    print(f"  Airy Wall Reference Plane: Normal={norm_a.round(4)}, d={d_a:.4f}m, Sigma={sigma_a*1000:.2f}mm ({len(a_wall_pts):,} pts)")
    print(f"  Gemini Wall Raw Plane:      Normal={norm_g.round(4)}, d={d_g:.4f}m, Sigma={sigma_g*1000:.2f}mm ({len(g_wall_pts):,} pts)")
    print(f"  Signed Offset (Gemini - Airy): {signed_plane_offset:+.2f} mm | Normal Angular Difference: {normal_angle_deg:.2f}°")

    # Global Queries for all Gemini points against Airy
    dists_euclid, idxs_a = airy_tree.query(gemini_xyz, k=1, workers=4)
    r_gemini = np.linalg.norm(gemini_xyz, axis=1)

    # Compute point-to-plane residual for all Gemini points to their matched Airy tangent plane
    pt2plane_signed = np.zeros(n_gemini, dtype=np.float64)
    is_planar_anchor = np.zeros(n_gemini, dtype=bool)

    for i in range(n_gemini):
        uid = idxs_a[i]
        norm_t, cent_t, is_pl = tangents[uid]
        is_planar_anchor[i] = is_pl
        if is_pl:
            pt2plane_signed[i] = np.dot(gemini_xyz[i] - cent_t, norm_t)
        else:
            pt2plane_signed[i] = np.dot(gemini_xyz[i] - airy_xyz[uid], np.array([0., 1., 0.]))

    pt2plane_abs = np.abs(pt2plane_signed)

    # Define Candidate Policies
    policies = [
        {"name": "Airy-Only Baseline", "strat": "AIRY", "tau": 0.0},
        {"name": "Naive Concatenation", "strat": "NAIVE", "tau": 0.0},
        {"name": "ASPS (τ = 20 mm)", "strat": "ASPS", "tau": 0.020},
        {"name": "ASPS (τ = 30 mm)", "strat": "ASPS", "tau": 0.030},
        {"name": "ASPS (τ = 50 mm)", "strat": "ASPS", "tau": 0.050},
        {"name": "SSD  (τ = 20 mm)", "strat": "SSD",  "tau": 0.020},
    ]

    results = []

    print("\n[2] Executing Forensic Evaluation Across Candidate Policies...")

    for pol in policies:
        strat = pol["strat"]
        tau_m = pol["tau"]

        # Classification masks
        # Tier 3 (4m+) gate: reject far-field Gemini points with dist > tau
        far_mask = (r_gemini >= 4.0)
        rejected_far = far_mask & (dists_euclid > max(tau_m, 0.05))

        if strat == "AIRY":
            fused_xyz = airy_xyz.copy()
            fused_rgb = np.full((n_airy, 3), 160, dtype=np.uint8)
            snapped_pct = 0.0
            rejected_pct = 0.0
            gemini_only_pct = 0.0
            gemini_contrib = 0
            airy_contrib = n_airy

        elif strat == "NAIVE":
            fused_xyz = np.vstack((airy_xyz, gemini_xyz))
            fused_rgb = np.vstack((np.full((n_airy, 3), 160, dtype=np.uint8), gemini_rgb))
            snapped_pct = 0.0
            rejected_pct = 0.0
            gemini_only_pct = 100.0
            gemini_contrib = n_gemini
            airy_contrib = n_airy

        elif strat == "ASPS":
            # Snapping rule: p_G' = p_G - [n^T (p_G - cent_A)] n
            # Triggered when point is within tau_m of Airy tangent plane and in-plane search <= (tau_m + 0.10)
            overlap_mask = (pt2plane_abs <= tau_m) & is_planar_anchor & (dists_euclid <= (tau_m + 0.10)) & (~rejected_far)
            
            gemini_processed = gemini_xyz.copy()
            overlap_idxs = np.where(overlap_mask)[0]
            for idx in overlap_idxs:
                uid = idxs_a[idx]
                norm_t, cent_t, _ = tangents[uid]
                s = pt2plane_signed[idx]
                gemini_processed[idx] = gemini_xyz[idx] - (s * norm_t)

            admitted_g = ~rejected_far
            fused_xyz = np.vstack((airy_xyz, gemini_processed[admitted_g]))
            fused_rgb = np.vstack((np.full((n_airy, 3), 160, dtype=np.uint8), gemini_rgb[admitted_g]))

            snapped_cnt = np.sum(overlap_mask)
            rejected_cnt = np.sum(rejected_far)
            gemini_only_cnt = np.sum(admitted_g & (~overlap_mask))

            snapped_pct = (snapped_cnt / n_gemini) * 100.0
            rejected_pct = (rejected_cnt / n_gemini) * 100.0
            gemini_only_pct = (gemini_only_cnt / n_gemini) * 100.0
            gemini_contrib = int(np.sum(admitted_g))
            airy_contrib = n_airy

        elif strat == "SSD":
            overlap_mask = (dists_euclid <= tau_m) & (~rejected_far)
            keep_g = (~overlap_mask) & (~rejected_far)
            fused_xyz = np.vstack((airy_xyz, gemini_xyz[keep_g]))
            fused_rgb = np.vstack((np.full((n_airy, 3), 160, dtype=np.uint8), gemini_rgb[keep_g]))

            snapped_pct = 0.0
            rejected_pct = (np.sum(rejected_far | overlap_mask) / n_gemini) * 100.0
            gemini_only_pct = (np.sum(keep_g) / n_gemini) * 100.0
            gemini_contrib = int(np.sum(keep_g))
            airy_contrib = n_airy

        # Wall Planar Analysis on Fused Cloud
        mask_fused_wall = (fused_xyz[:, 0] >= wall_box['x'][0]) & (fused_xyz[:, 0] <= wall_box['x'][1]) & \
                          (fused_xyz[:, 1] >= wall_box['y'][0]) & (fused_xyz[:, 1] <= wall_box['y'][1]) & \
                          (fused_xyz[:, 2] >= wall_box['z'][0]) & (fused_xyz[:, 2] <= wall_box['z'][1])
        fused_wall_pts = fused_xyz[mask_fused_wall]

        # 1. Fitted Fused Plane Sigma
        norm_f, d_f, cent_f, sigma_fused, res_fused = fit_plane_svd(fused_wall_pts)

        # 2. Complete Wall Thickness (span of normal residuals)
        abs_res_fused = np.abs(res_fused) * 1000.0 # in mm
        thick_p50 = np.percentile(abs_res_fused, 50) * 2.0
        thick_p90 = np.percentile(abs_res_fused, 90) * 2.0
        thick_p95 = np.percentile(abs_res_fused, 95) * 2.0

        # 3. Absolute Point-to-Plane Residuals to Authoritative Airy Wall Plane
        res_to_airy_plane = (np.dot(fused_wall_pts, norm_a) + d_a) * 1000.0 # mm
        abs_res_to_airy = np.abs(res_to_airy_plane)
        p2p_p50 = np.percentile(abs_res_to_airy, 50)
        p2p_p90 = np.percentile(abs_res_to_airy, 90)
        p2p_p95 = np.percentile(abs_res_to_airy, 95)

        # 4. Nearest-Neighbor Euclidean Residuals (Fused to Airy Ground Truth)
        tree_fused = cKDTree(fused_xyz)
        dists_euclid_res, _ = tree_fused.query(airy_xyz, k=1, workers=4)
        valid_euclid = dists_euclid_res[dists_euclid_res < 0.10] * 1000.0 # mm
        euc_p50 = np.percentile(valid_euclid, 50) if len(valid_euclid) > 0 else 0.0
        euc_p90 = np.percentile(valid_euclid, 90) if len(valid_euclid) > 0 else 0.0
        euc_p95 = np.percentile(valid_euclid, 95) if len(valid_euclid) > 0 else 0.0

        # 5. Voxel Occupancy Counts (15mm and 30mm)
        vox_15mm = np.unique(np.floor(fused_xyz / 0.015).astype(int), axis=0)
        vox_30mm = np.unique(np.floor(fused_xyz / 0.030).astype(int), axis=0)
        n_vox_15mm = len(vox_15mm)
        n_vox_30mm = len(vox_30mm)

        # 6. Density Gain relative to Airy
        density_gain = len(fused_xyz) / n_airy

        results.append({
            "name": pol["name"],
            "strat": strat,
            "tau_mm": tau_m * 1000.0,
            "total_points": len(fused_xyz),
            "airy_points": airy_contrib,
            "gemini_points": gemini_contrib,
            "wall_points": len(fused_wall_pts),
            "fused_plane_sigma_mm": round(sigma_fused * 1000.0, 2),
            "signed_offset_mm": round((d_f - d_a) * 1000.0, 2),
            "thick_p50_mm": round(thick_p50, 2),
            "thick_p90_mm": round(thick_p90, 2),
            "thick_p95_mm": round(thick_p95, 2),
            "p2p_p50_mm": round(p2p_p50, 2),
            "p2p_p90_mm": round(p2p_p90, 2),
            "p2p_p95_mm": round(p2p_p95, 2),
            "euc_p50_mm": round(euc_p50, 2),
            "euc_p90_mm": round(euc_p90, 2),
            "euc_p95_mm": round(euc_p95, 2),
            "snapped_pct": round(snapped_pct, 2),
            "rejected_pct": round(rejected_pct, 2),
            "gemini_only_pct": round(gemini_only_pct, 2),
            "vox_15mm_count": n_vox_15mm,
            "vox_30mm_count": n_vox_30mm,
            "density_gain": round(density_gain, 2)
        })

    # Print Formatted Comprehensive Table
    print("\n" + "="*160)
    print("M12.5 FORENSIC COMPARISON TABLE (UNAMBIGUOUS DEFINITIONS)")
    print("="*160)
    header = (
        f"{'Policy Candidate':<22} | {'Total Pts':<10} | {'Plane σ':<8} | {'P2Plane P95':<11} | {'Wall Thk P95':<12} | "
        f"{'Euclid P95':<10} | {'Snapped %':<9} | {'GemOnly %':<9} | {'15mm Voxels':<11} | {'Density Gain'}"
    )
    print(header)
    print("-" * 160)
    for r in results:
        row = (
            f"{r['name']:<22} | {r['total_points']:<10,d} | {r['fused_plane_sigma_mm']:<8.2f} | {r['p2p_p95_mm']:<11.2f} | "
            f"{r['thick_p95_mm']:<12.2f} | {r['euc_p95_mm']:<10.2f} | {r['snapped_pct']:<8.1f}% | {r['gemini_only_pct']:<8.1f}% | "
            f"{r['vox_15mm_count']:<11,d} | {r['density_gain']:<5.2f}x"
        )
        print(row)
    print("="*160)

    # Save forensic JSON
    forensic_path = "/home/scanar/scanarMini/data/M12_5_Fusion/m12_5_forensic_metrics.json"
    with open(forensic_path, "w") as f_out:
        json.dump(results, f_out, indent=2)
    print(f"\n[Forensic] Detailed JSON saved to {forensic_path}")

if __name__ == '__main__':
    run_forensic_analysis()
