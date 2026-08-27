#!/usr/bin/env python3
"""
Milestone M12.7: Optimized Production Three-Class Geometric Classifier & Dynamic Fusion Engine
Classification:
1. SUPPORTED OVERLAP (|s| <= 20mm, Airy planar):
   - Snaps normal component to Airy tangent plane: p_G' = p_G - [n^T (p_G - p_A)] n
   - Preserves in-plane tangential detail (I - n n^T) p_G and RGB color.
2. CONFLICTING OVERLAP (20mm < |s| <= 80mm, Airy planar, d_euclid <= 0.20m):
   - Corrects stereo disparity bias onto Airy tangent plane: p_G' = p_G - [n^T (p_G - p_A)] n
   - Completely collapses double-wall artifacts while preserving dense color samples.
3. TRUE SUPPLEMENTAL DETAIL (no local Airy support, structural gap, non-planar edge, or |s| > 80mm):
   - Preserved untouched as genuine supplemental geometry.
"""

import os
import sys
import time
import math
import json
import numpy as np
from scipy.spatial import cKDTree

def precompute_airy_tangent_patches(airy_xyz, r_search=0.15, min_pts=6):
    """Precomputes authoritative tangent planes for Airy anchors."""
    tree = cKDTree(airy_xyz)
    ball_idxs = tree.query_ball_point(airy_xyz, r=r_search, workers=4)
    tangents = {}
    for i, idxs in enumerate(ball_idxs):
        if len(idxs) >= min_pts:
            sub = airy_xyz[idxs]
            cent = np.mean(sub, axis=0)
            cov = np.cov((sub - cent).T)
            evals, evecs = np.linalg.eigh(cov)
            norm = evecs[:, 0]
            if evals[0] < 0.008: # Clean planar surface
                if np.dot(norm, cent) > 0:
                    norm = -norm
                tangents[i] = (norm, cent, True)
            else:
                tangents[i] = (norm, cent, False)
        else:
            tangents[i] = (np.array([0., 1., 0.]), airy_xyz[i], False)
    return tree, tangents

def execute_three_class_asps_fusion(
    airy_xyz, 
    gemini_xyz, 
    gemini_rgb, 
    tau_snap_m=0.020, 
    tau_conflict_m=0.080, 
    r_search=0.15
):
    """
    Executes Three-Class ASPS Fusion.
    Returns:
      fused_xyz, fused_rgb, classification_dict
    """
    n_airy = len(airy_xyz)
    n_gemini = len(gemini_xyz)
    if n_gemini == 0:
        return airy_xyz, np.full((n_airy, 3), 160, dtype=np.uint8), {}

    t0 = time.perf_counter()

    # 1. Authoritative Airy KDTree & Tangent Planes on Unique Matched Anchors
    airy_tree = cKDTree(airy_xyz)
    dists_euclid, idxs_a = airy_tree.query(gemini_xyz, k=1, workers=4)

    unique_uids = np.unique(idxs_a)
    ball_idxs = airy_tree.query_ball_point(airy_xyz[unique_uids], r=r_search, workers=4)
    
    tangents = {}
    for i, uid in enumerate(unique_uids):
        idxs = ball_idxs[i]
        if len(idxs) >= 6:
            sub = airy_xyz[idxs]
            cent = np.mean(sub, axis=0)
            cov = np.cov((sub - cent).T)
            evals, evecs = np.linalg.eigh(cov)
            norm = evecs[:, 0]
            if evals[0] < 0.008: # Clean planar patch
                if np.dot(norm, cent) > 0:
                    norm = -norm
                tangents[uid] = (norm, cent, True)
            else:
                tangents[uid] = (norm, cent, False)
        else:
            tangents[uid] = (np.array([0., 1., 0.]), airy_xyz[uid], False)

    # 2. Vectorized Signed Point-to-Plane Distance
    norms_arr = np.zeros((len(unique_uids), 3), dtype=np.float64)
    cents_arr = np.zeros((len(unique_uids), 3), dtype=np.float64)
    is_pl_arr = np.zeros(len(unique_uids), dtype=bool)
    for i, u in enumerate(unique_uids):
        norms_arr[i] = tangents[u][0]
        cents_arr[i] = tangents[u][1]
        is_pl_arr[i] = tangents[u][2]

    # Map idxs_a to unique_uids indices
    uid_to_idx = {u: i for i, u in enumerate(unique_uids)}
    gem_u_idx = np.array([uid_to_idx[u] for u in idxs_a])

    gem_norms = norms_arr[gem_u_idx]
    gem_cents = cents_arr[gem_u_idx]
    airy_is_planar = is_pl_arr[gem_u_idx]

    pt2plane_signed = np.sum((gemini_xyz - gem_cents) * gem_norms, axis=1)
    pt2plane_signed[~airy_is_planar] = dists_euclid[~airy_is_planar]
    pt2plane_abs = np.abs(pt2plane_signed)

    # Far-field Gating (>4.5m requires tighter spatial proximity)
    r_gemini = np.linalg.norm(gemini_xyz, axis=1)
    rejected_far = (r_gemini >= 4.5) & (dists_euclid > 0.08)

    # Three-Class Masks
    # Class 1: SUPPORTED OVERLAP (|s| <= tau_snap_m)
    mask_supported = (pt2plane_abs <= tau_snap_m) & airy_is_planar & (dists_euclid <= (tau_snap_m + 0.10)) & (~rejected_far)

    # Class 2: CONFLICTING OVERLAP (tau_snap_m < |s| <= tau_conflict_m)
    # Lies within disparity bias band on same planar structure
    mask_conflicting = (pt2plane_abs > tau_snap_m) & (pt2plane_abs <= tau_conflict_m) & \
                       airy_is_planar & (dists_euclid <= (tau_conflict_m + 0.15)) & (~rejected_far)

    # Class 3: TRUE SUPPLEMENTAL DETAIL (Fine features, LiDAR beam gaps, objects outside plane)
    mask_supplemental = (~mask_supported) & (~mask_conflicting) & (~rejected_far)

    # Point Processing - Vectorized Snapping
    gemini_processed = gemini_xyz.copy()
    snap_mask = (mask_supported | mask_conflicting)
    gemini_processed[snap_mask] = gemini_xyz[snap_mask] - (pt2plane_signed[snap_mask, None] * gem_norms[snap_mask])

    admitted_mask = ~rejected_far
    fused_xyz = np.vstack((airy_xyz, gemini_processed[admitted_mask]))
    airy_rgb = np.full((n_airy, 3), 160, dtype=np.uint8)
    fused_rgb = np.vstack((airy_rgb, gemini_rgb[admitted_mask]))

    t_elapsed_ms = (time.perf_counter() - t0) * 1000.0

    n_supp = int(np.sum(mask_supported))
    n_conf = int(np.sum(mask_conflicting))
    n_suppl = int(np.sum(mask_supplemental))
    n_rej = int(np.sum(rejected_far))

    classification = {
        "total_gemini_input": int(n_gemini),
        "class_1_supported_overlap_count": n_supp,
        "class_1_supported_overlap_percent": float(round((n_supp / n_gemini) * 100.0, 2)),
        "class_2_conflicting_overlap_count": n_conf,
        "class_2_conflicting_overlap_percent": float(round((n_conf / n_gemini) * 100.0, 2)),
        "class_3_true_supplemental_count": n_suppl,
        "class_3_true_supplemental_percent": float(round((n_suppl / n_gemini) * 100.0, 2)),
        "rejected_far_field_count": n_rej,
        "rejected_far_field_percent": float(round((n_rej / n_gemini) * 100.0, 2)),
        "density_gain_over_airy": float(round(len(fused_xyz) / n_airy, 2)),
        "runtime_ms": float(round(t_elapsed_ms, 2))
    }

    return fused_xyz, fused_rgb, classification

if __name__ == '__main__':
    print("M12.7 Three-Class Dynamic Fusion Engine Ready.")
