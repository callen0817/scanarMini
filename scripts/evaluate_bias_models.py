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

def evaluate_scale_and_extrinsics():
    airy_file = "/home/scanar/scanarMini/data/M12_4_Characterization/m12_4_airy_cloud.ply"
    gemini_file = "/home/scanar/scanarMini/data/M12_4_Characterization/m12_4_gemini_cloud_rgb.ply"

    a_pts, _ = read_binary_ply(airy_file)
    g_pts, _ = read_binary_ply(gemini_file)

    tree = cKDTree(a_pts)

    print("Evaluating depth scale factor sweep on nearest-neighbor residuals...")
    # Assume origin is (0, 0, 0) in body/sensor frame
    # In world frame, sensor is near origin (0, 0, 0)
    for scale in [0.97, 0.98, 0.99, 1.00, 1.01, 1.02, 1.03]:
        scaled_g = g_pts * scale
        dists, _ = tree.query(scaled_g, k=1, workers=4)
        ov_mask = dists < 0.10
        ov_dists = dists[ov_mask]
        p50 = np.percentile(ov_dists, 50) * 1000.0 if len(ov_dists) > 0 else 0
        p90 = np.percentile(ov_dists, 90) * 1000.0 if len(ov_dists) > 0 else 0
        mean = np.mean(ov_dists) * 1000.0 if len(ov_dists) > 0 else 0
        print(f"Scale {scale:.3f}: Overlap (<10cm) = {len(ov_dists):6d} | P50 = {p50:5.2f} mm | P90 = {p90:5.2f} mm | Mean = {mean:5.2f} mm")

if __name__ == '__main__':
    evaluate_scale_and_extrinsics()
