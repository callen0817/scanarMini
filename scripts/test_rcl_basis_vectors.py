#!/usr/bin/env python3
"""
Unit Test: FAST-LIVO2 Camera <- LiDAR Rotation Matrix (Rcl) Basis Vector Mapping
Validates that Rcl correctly maps LiDAR coordinate axes (Forward=+X, Left=+Y, Up=+Z)
into Camera Optical coordinate axes (Right=+X, Down=+Y, Optical Axis/Forward=+Z).
"""

import sys
import numpy as np

def main():
    print("="*75)
    print("   FAST-LIVO2 Rcl (Camera <- LiDAR) BASIS VECTOR VERIFICATION")
    print("="*75)

    # Rcl from config/robosense.yaml
    Rcl = np.array([
        [ 0.0, -1.0,  0.0],
        [ 0.0,  0.0, -1.0],
        [ 1.0,  0.0,  0.0]
    ])
    
    print("\nConfigured Rcl matrix (Camera <- LiDAR):")
    print(Rcl)

    # Basis vectors in LiDAR frame
    e_x_lidar = np.array([1.0, 0.0, 0.0]) # Forward
    e_y_lidar = np.array([0.0, 1.0, 0.0]) # Left
    e_z_lidar = np.array([0.0, 0.0, 1.0]) # Up

    # Expected basis vectors in Camera Optical frame
    # Camera Optical: +X = Right, +Y = Down, +Z = Forward (Optical axis)
    # Forward (+X_lidar) -> +Z_cam [0, 0, 1]
    # Left (+Y_lidar)    -> -X_cam [-1, 0, 0] (since +X_cam is Right)
    # Up (+Z_lidar)      -> -Y_cam [0, -1, 0] (since +Y_cam is Down)

    v_x_cam = Rcl @ e_x_lidar
    v_y_cam = Rcl @ e_y_lidar
    v_z_cam = Rcl @ e_z_lidar

    exp_x = np.array([0.0, 0.0, 1.0])
    exp_y = np.array([-1.0, 0.0, 0.0])
    exp_z = np.array([0.0, -1.0, 0.0])

    pass_x = np.allclose(v_x_cam, exp_x)
    pass_y = np.allclose(v_y_cam, exp_y)
    pass_z = np.allclose(v_z_cam, exp_z)

    print("\n--- Basis Vector Test Results ---")
    print(f"1. LiDAR +X (Forward) [1,0,0] -> Cam: {v_x_cam.tolist()} | Expected: {exp_x.tolist()} -> {'✅ PASS' if pass_x else '❌ FAIL'}")
    print(f"2. LiDAR +Y (Left)    [0,1,0] -> Cam: {v_y_cam.tolist()} | Expected: {exp_y.tolist()} -> {'✅ PASS' if pass_y else '❌ FAIL'}")
    print(f"3. LiDAR +Z (Up)      [0,0,1] -> Cam: {v_z_cam.tolist()} | Expected: {exp_z.tolist()} -> {'✅ PASS' if pass_z else '❌ FAIL'}")

    det = np.linalg.det(Rcl)
    is_so3 = np.isclose(det, 1.0) and np.allclose(Rcl.T @ Rcl, np.eye(3))
    print(f"\nOrthogonality & Determinant Check: det(Rcl) = {det:.4f} -> {'✅ SO(3) VALID' if is_so3 else '❌ INVALID'}")
    print("="*75)

    if not (pass_x and pass_y and pass_z and is_so3):
        sys.exit(1)

if __name__ == '__main__':
    main()
