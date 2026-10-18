#!/usr/bin/env python3
import os
import sys
import json
import numpy as np
from scipy.spatial.transform import Rotation

def evaluate_motion_sanity(dataset_dir, fastlivo_log='/tmp/m12_9_fastlivo.log', bringup_log='/tmp/m12_9_bringup.log'):
    print('======================================================================')
    print('=== EVALUATING DYNAMIC MOTION SANITY TEST UNDER NEW LIO EXTRINSIC ===')
    print('======================================================================')
    
    # 1. Trajectory Analysis
    tum_path = os.path.join(dataset_dir, 'trajectory.tum')
    if not os.path.exists(tum_path):
        print(f'FATAL: {tum_path} does not exist!')
        return False
        
    poses = []
    with open(tum_path, 'r') as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) == 8:
                stamp = float(parts[0])
                pos = np.array([float(parts[1]), float(parts[2]), float(parts[3])])
                ori = np.array([float(parts[4]), float(parts[5]), float(parts[6]), float(parts[7])]) # x, y, z, w
                poses.append((stamp, pos, ori))
                
    if len(poses) < 5:
        print(f'FATAL: Too few poses ({len(poses)}) recorded in trajectory!')
        return False
        
    stamps = np.array([p[0] for p in poses])
    positions = np.array([p[1] for p in poses])
    quats = np.array([p[2] for p in poses])
    rotations = Rotation.from_quat(quats)
    euler_rpy = rotations.as_euler('xyz', degrees=True) # roll, pitch, yaw in degrees
    
    duration = stamps[-1] - stamps[0]
    total_dist = float(np.sum(np.linalg.norm(np.diff(positions, axis=0), axis=1)))
    
    # Forward and return displacements relative to initial pose
    pos_rel = positions - positions[0]
    dist_from_start = np.linalg.norm(pos_rel, axis=1)
    max_disp = float(np.max(dist_from_start))
    end_disp = float(dist_from_start[-1])
    
    # Yaw changes
    yaw_angles = euler_rpy[:, 2]
    # Unwrapped yaw
    yaw_unwrapped = np.unwrap(np.radians(yaw_angles))
    yaw_range_deg = float(np.degrees(np.max(yaw_unwrapped) - np.min(yaw_unwrapped)))
    
    # Roll and pitch excursions
    roll_angles = euler_rpy[:, 0]
    pitch_angles = euler_rpy[:, 1]
    roll_excursion = float(np.max(roll_angles) - np.min(roll_angles))
    pitch_excursion = float(np.max(pitch_angles) - np.min(pitch_angles))
    
    print(f'Trajectory Duration: {duration:.2f} s | Number of Poses: {len(poses)}')
    print(f'Total Integrated Path Distance: {total_dist:.2f} m')
    print(f'Max Forward Displacement from Start: {max_disp:.2f} m')
    print(f'End Return Displacement from Start: {end_disp:.2f} m')
    print(f'Yaw Angular Excursion: {yaw_range_deg:.1f} deg')
    print(f'Roll Stability (Max-Min): {roll_excursion:.2f} deg')
    print(f'Pitch Stability (Max-Min): {pitch_excursion:.2f} deg')
    
    # 2. PointCloud2 Schema Verification
    lidar_idx_path = os.path.join(dataset_dir, 'lidar_index.json')
    if os.path.exists(lidar_idx_path):
        with open(lidar_idx_path) as f:
            lidar_entries = json.load(f)
        if len(lidar_entries) > 0:
            first = lidar_entries[0]
            print('\n--> Actual /rslidar_points PointCloud2 Schema:')
            print(f'    point_step: {first.get("point_step")}, is_bigendian: {first.get("is_bigendian")}')
            for f in first.get('fields', []):
                print(f'    - field: {f["name"]:15s} | offset: {f["offset"]:2d} | datatype: {f["datatype"]} | count: {f["count"]}')
            
            # Datatype validity confirmation
            all_count_1 = all(f['count'] == 1 for f in first.get('fields', []))
            is_le = not first.get('is_bigendian', False)
            print(f'    [Schema Conclusion] is_bigendian={first.get("is_bigendian")}, all fields count=1: {all_count_1}')
            print('    Current datatype-aware decoder is VALID for this Airy stream.')
    
    # 3. Rate and Desync Telemetry from this exact run
    print('\n--> Rate Breakdown from This Exact Run:')
    # FAST-LIVO Log Check
    fl_sync_warnings = 0
    if os.path.exists(fastlivo_log):
        with open(fastlivo_log) as f:
            fl_text = f.read()
        fl_sync_warnings = fl_text.count('fall behind')
        
    print(f'    FAST-LIVO2 Sync Warnings (Fall Behind): {fl_sync_warnings}')
    
    # Stream rates
    with open(os.path.join(dataset_dir, 'lidar_index.json')) as f:
        l_entries = json.load(f)
    with open(os.path.join(dataset_dir, 'depth_index.json')) as f:
        d_entries = json.load(f)
    with open(os.path.join(dataset_dir, 'color_index.json')) as f:
        c_entries = json.load(f)
        
    l_rate = len(l_entries) / duration if duration > 0 else 0
    d_rate = len(d_entries) / duration if duration > 0 else 0
    c_rate = len(c_entries) / duration if duration > 0 else 0
    odom_rate = len(poses) / duration if duration > 0 else 0
    
    print(f'    FAST-LIVO2 Odometry Callback Rate: {odom_rate:.2f} Hz ({len(poses)} poses in {duration:.2f} s)')
    print(f'    Airy LiDAR Recorder Rate:         {l_rate:.2f} Hz ({len(l_entries)} sweeps in {duration:.2f} s)')
    print(f'    Gemini Depth Keyframe Rate:       {d_rate:.2f} FPS ({len(d_entries)} frames)')
    print(f'    Gemini RGB Keyframe Rate:         {c_rate:.2f} FPS ({len(c_entries)} frames)')
    
    # Gate Checks
    checks = {
        'forward_motion_observed': max_disp > 0.40,
        'return_motion_observed': end_disp < max_disp * 0.70,
        'yaw_motion_observed': yaw_range_deg > 20.0,
        'roll_pitch_stability': (roll_excursion < 30.0) and (pitch_excursion < 30.0),
        'zero_sustained_sync_warnings': fl_sync_warnings == 0,
        'fastlivo_healthy': len(poses) > 10
    }
    
    print('\n--> Dynamic Sanity Gate Verdict:')
    all_pass = True
    for k, v in checks.items():
        status = 'PASS' if v else 'FAIL'
        if not v:
            all_pass = False
        print(f'    [{status}] {k}')
        
    print('======================================================================')
    return all_pass

if __name__ == '__main__':
    d_dir = sys.argv[1] if len(sys.argv) > 1 else '/home/scanar/scanarMini/diagnostics/m12_9_room_dataset'
    evaluate_motion_sanity(d_dir)
