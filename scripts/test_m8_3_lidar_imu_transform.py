#!/usr/bin/env python3
"""
M8.3: Factory LiDAR <-> Internal IMU Transform Verification
Audits the factory RoboSense DIFOP quaternion and tests gravity vector rotation
from native IMU frame into the body-aligned LiDAR frame (rslidar).
"""

import math
import numpy as np
from scipy.spatial.transform import Rotation as R

def main():
    print("="*80)
    print("   M8.3: ROBOSENSE AIRY FACTORY LIDAR <-> IMU RIGID TRANSFORM AUDIT")
    print("="*80)

    # Factory DIFOP parameters from RoboSense:
    # static_transform_publisher args: x y z qx qy qz qw parent child
    # parent: rslidar, child: airy_imu
    t_lidar_imu = np.array([0.004250, 0.004180, -0.004460]) # translation in meters
    q_xyzw = [0.71145376, -0.70271848, 0.0018789, 0.00409338]
    
    rot_lidar_from_imu = R.from_quat(q_xyzw)
    R_lidar_imu = rot_lidar_from_imu.as_matrix()
    
    print("\n--- [Factory Calibration Parameters] ---")
    print(f"Translation t_lidar_from_imu (m): {t_lidar_imu}")
    print(f"Quaternion (qx, qy, qz, qw):       {q_xyzw}")
    print("\nRotation Matrix R_lidar_from_imu:")
    for row in R_lidar_imu:
        print(f"  [{row[0]:+9.6f}, {row[1]:+9.6f}, {row[2]:+9.6f}]")

    euler_xyz_deg = rot_lidar_from_imu.as_euler('xyz', degrees=True)
    print(f"\nEuler Angles (XYZ extrinsic, deg): Roll={euler_xyz_deg[0]:.2f}°, Pitch={euler_xyz_deg[1]:.2f}°, Yaw={euler_xyz_deg[2]:.2f}°")

    # -------------------------------------------------------------
    # State 1: Normal Base-Down / Level Pose
    # -------------------------------------------------------------
    print("\n" + "-"*80)
    print("STATE 1: Stationary Level / Base-Down Pose (Dome-Up)")
    print("-"*80)
    g_imu_s1 = np.array([+0.883, +1.073, -9.814]) # measured from M8.2
    g_lidar_s1 = R_lidar_imu @ g_imu_s1
    print(f"Native IMU Vector:      [{g_imu_s1[0]:+7.3f}, {g_imu_s1[1]:+7.3f}, {g_imu_s1[2]:+7.3f}] m/s²")
    print(f"Transformed to LiDAR:   [{g_lidar_s1[0]:+7.3f}, {g_lidar_s1[1]:+7.3f}, {g_lidar_s1[2]:+7.3f}] m/s²")
    print(f"LiDAR Body Expectation: Gravity / vertical specific force aligned with body vertical (+Z / -Z).")

    # -------------------------------------------------------------
    # State 2: Nose Down (Camera/Front Down, Pitch Down 90°)
    # -------------------------------------------------------------
    print("\n" + "-"*80)
    print("STATE 2: Nose Down (Camera/Front Down 90°)")
    print("-"*80)
    # Pitch down moves gravity from -Z to +Y in native IMU
    g_imu_s2 = np.array([0.0, +9.81, 0.0])
    g_lidar_s2 = R_lidar_imu @ g_imu_s2
    print(f"Native IMU Vector:      [{g_imu_s2[0]:+7.3f}, {g_imu_s2[1]:+7.3f}, {g_imu_s2[2]:+7.3f}] m/s²")
    print(f"Transformed to LiDAR:   [{g_lidar_s2[0]:+7.3f}, {g_lidar_s2[1]:+7.3f}, {g_lidar_s2[2]:+7.3f}] m/s²")
    print(f"LiDAR Body Expectation: Gravity projects along forward axis (-X in LiDAR body).")

    # -------------------------------------------------------------
    # State 3: Left Side Down (Roll Left 90°)
    # -------------------------------------------------------------
    print("\n" + "-"*80)
    print("STATE 3: Left Side Down (Roll Left 90°)")
    print("-"*80)
    # Roll left moves gravity from -Z to +X in native IMU
    g_imu_s3 = np.array([+9.81, 0.0, 0.0])
    g_lidar_s3 = R_lidar_imu @ g_imu_s3
    print(f"Native IMU Vector:      [{g_imu_s3[0]:+7.3f}, {g_imu_s3[1]:+7.3f}, {g_imu_s3[2]:+7.3f}] m/s²")
    print(f"Transformed to LiDAR:   [{g_lidar_s3[0]:+7.3f}, {g_lidar_s3[1]:+7.3f}, {g_lidar_s3[2]:+7.3f}] m/s²")
    print(f"LiDAR Body Expectation: Gravity projects along left lateral axis (-Y in LiDAR body).")

    print("\n" + "="*80)

if __name__ == '__main__':
    main()
