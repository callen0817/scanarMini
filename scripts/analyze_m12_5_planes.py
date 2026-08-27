#!/usr/bin/env python3
import os
import sys
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

def fit_plane_pca(pts):
    centroid = np.mean(pts, axis=0)
    cov = np.cov((pts - centroid).T)
    eigenvalues, eigenvectors = np.linalg.eigh(cov)
    # The normal is the eigenvector corresponding to the smallest eigenvalue
    normal = eigenvectors[:, 0]
    if normal[1] < 0: # orient normal towards positive Y
        normal = -normal
    d = -np.dot(normal, centroid)
    residuals = np.dot(pts, normal) + d
    std_dev = np.std(residuals)
    return normal, d, centroid, std_dev, residuals

def main():
    airy_file = "/home/scanar/scanarMini/data/M12_4_Characterization/m12_4_airy_cloud.ply"
    gemini_file = "/home/scanar/scanarMini/data/M12_4_Characterization/m12_4_gemini_cloud_rgb.ply"

    a_pts, _ = read_binary_ply(airy_file)
    g_pts, g_rgb = read_binary_ply(gemini_file)

    tree = cKDTree(a_pts)
    dists, idxs = tree.query(g_pts, k=1, workers=4)

    # Let's define multiple spatial sub-regions within the overlap zone:
    # 1. Main Forward Wall (around Y = -2.3m, Z in [-1.5, 0.0], X in [-0.8, 0.8])
    # 2. Lower Ground/Floor Plane (Z in [-1.8, -1.4], Y in [-3.5, -2.0], X in [-1.0, 1.0])
    # 3. Left Planar Obstacle/Wall (X in [-1.8, -1.0], Y in [-3.0, -2.0], Z in [-1.0, 0.0])
    # 4. Right Planar Obstacle/Wall (X in [0.8, 1.8], Y in [-3.0, -2.0], Z in [-1.0, 0.0])
    # 5. Mid-Range Wall (Y in [-3.8, -3.2], X in [-1.0, 1.0], Z in [-1.2, -0.2])
    # 6. Deep Far Wall (Y in [-5.2, -4.4], X in [-1.2, 1.2], Z in [-1.5, -0.5])

    patches = [
        ("Patch 1: Forward Wall Center (Y ~ -2.3m)", {'x': (-0.6, 0.6), 'y': (-2.5, -2.1), 'z': (-1.0, -0.2)}),
        ("Patch 2: Forward Wall Left (Y ~ -2.4m)",   {'x': (-1.5, -0.6), 'y': (-2.6, -2.2), 'z': (-1.0, -0.2)}),
        ("Patch 3: Forward Wall Right (Y ~ -2.4m)",  {'x': (0.6, 1.5), 'y': (-2.6, -2.2), 'z': (-1.0, -0.2)}),
        ("Patch 4: Ground/Floor Plane (Z ~ -1.5m)",  {'x': (-0.8, 0.8), 'y': (-3.0, -2.0), 'z': (-1.7, -1.4)}),
        ("Patch 5: Mid-Distance Surface (Y ~ -3.5m)",{'x': (-0.8, 0.8), 'y': (-3.7, -3.2), 'z': (-1.2, -0.4)}),
        ("Patch 6: Far Surface (Y ~ -4.8m)",         {'x': (-1.0, 1.0), 'y': (-5.2, -4.4), 'z': (-1.5, -0.6)}),
    ]

    print("=" * 100)
    print(f"{'Patch Description':<35} | {'Airy Pts':<9} | {'Gemini Pts':<10} | {'Airy Std':<9} | {'Gem Std':<9} | {'Plane Normal':<22} | {'Bias (Mean)':<12} | {'Bias (Med)'}")
    print("=" * 100)

    for name, box in patches:
        a_in = (a_pts[:, 0] >= box['x'][0]) & (a_pts[:, 0] <= box['x'][1]) & \
               (a_pts[:, 1] >= box['y'][0]) & (a_pts[:, 1] <= box['y'][1]) & \
               (a_pts[:, 2] >= box['z'][0]) & (a_pts[:, 2] <= box['z'][1])
        
        g_in = (g_pts[:, 0] >= box['x'][0]) & (g_pts[:, 0] <= box['x'][1]) & \
               (g_pts[:, 1] >= box['y'][0]) & (g_pts[:, 1] <= box['y'][1]) & \
               (g_pts[:, 2] >= box['z'][0]) & (g_pts[:, 2] <= box['z'][1])
        
        a_sub = a_pts[a_in]
        g_sub = g_pts[g_in]

        if len(a_sub) < 15 or len(g_sub) < 50:
            print(f"{name:<35} | {len(a_sub):<9} | {len(g_sub):<10} | INSUFFICIENT SAMPLES")
            continue

        # Fit Airy plane
        a_norm, a_d, a_cent, a_std, a_res = fit_plane_pca(a_sub)
        
        # Fit Gemini plane
        g_norm, g_d, g_cent, g_std, g_res = fit_plane_pca(g_sub)

        # Signed distance of Gemini points to Airy plane
        g_signed = np.dot(g_sub, a_norm) + a_d
        
        # Trim outliers beyond 10 cm
        inlier_mask = np.abs(g_signed) < 0.10
        if np.sum(inlier_mask) > 10:
            g_inliers = g_signed[inlier_mask]
            mean_b = np.mean(g_inliers) * 1000.0
            med_b = np.median(g_inliers) * 1000.0
        else:
            mean_b = np.mean(g_signed) * 1000.0
            med_b = np.median(g_signed) * 1000.0

        norm_str = f"[{a_norm[0]:.2f}, {a_norm[1]:.2f}, {a_norm[2]:.2f}]"
        print(f"{name:<35} | {len(a_sub):<9} | {len(g_sub):<10} | {a_std*1000:6.1f} mm | {g_std*1000:6.1f} mm | {norm_str:<22} | {mean_b:+9.2f} mm | {med_b:+9.2f} mm")

if __name__ == '__main__':
    main()
