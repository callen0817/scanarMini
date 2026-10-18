#!/usr/bin/env python3
import os
import sys
import json
import yaml
import shutil
import cv2
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation, Slerp

def decode_field(raw_bytes, offset, datatype, count, point_step, is_bigendian=False):
    # ROS2 PointField datatypes:
    # 1: INT8, 2: UINT8, 3: INT16, 4: UINT16, 5: INT32, 6: UINT32, 7: FLOAT32, 8: FLOAT64
    num_pts = len(raw_bytes) // point_step
    if num_pts == 0:
        return np.zeros(0, dtype=np.float32)
        
    raw_arr = np.frombuffer(raw_bytes, dtype=np.uint8).reshape(num_pts, point_step)
    endian_prefix = '>' if is_bigendian else '<'
    
    if datatype == 7: # FLOAT32
        dtype_str = f'{endian_prefix}f4'
        return raw_arr[:, offset:offset+4].copy().view(dtype_str).flatten().astype(np.float32)
    elif datatype == 8: # FLOAT64
        dtype_str = f'{endian_prefix}f8'
        return raw_arr[:, offset:offset+8].copy().view(dtype_str).flatten().astype(np.float32)
    elif datatype == 2: # UINT8
        return raw_arr[:, offset].astype(np.float32)
    elif datatype == 1: # INT8
        return raw_arr[:, offset].astype(np.int8).astype(np.float32)
    elif datatype == 4: # UINT16
        dtype_str = f'{endian_prefix}u2'
        return raw_arr[:, offset:offset+2].copy().view(dtype_str).flatten().astype(np.float32)
    elif datatype == 3: # INT16
        dtype_str = f'{endian_prefix}i2'
        return raw_arr[:, offset:offset+2].copy().view(dtype_str).flatten().astype(np.float32)
    elif datatype == 6: # UINT32
        dtype_str = f'{endian_prefix}u4'
        return raw_arr[:, offset:offset+4].copy().view(dtype_str).flatten().astype(np.float32)
    elif datatype == 5: # INT32
        dtype_str = f'{endian_prefix}i4'
        return raw_arr[:, offset:offset+4].copy().view(dtype_str).flatten().astype(np.float32)
    else:
        dtype_str = f'{endian_prefix}f4'
        return raw_arr[:, offset:offset+4].copy().view(dtype_str).flatten().astype(np.float32)

def process_room_scan(dataset_dir, out_dir):
    # Always clean output directory to prevent stale artifacts
    if os.path.exists(out_dir):
        shutil.rmtree(out_dir)
    os.makedirs(out_dir, exist_ok=True)
    
    print(f'=== PROCESSING ROOM SCAN FROM {dataset_dir} ===')
    
    # 1. Load Trajectory
    tum_path = os.path.join(dataset_dir, 'trajectory.tum')
    if not os.path.exists(tum_path):
        print(f'Error: {tum_path} does not exist')
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
                
    # Ensure strictly monotonically increasing timestamps for SciPy Slerp
    clean_poses = []
    last_t = -1.0
    for p in poses:
        if p[0] > last_t:
            clean_poses.append(p)
            last_t = p[0]
    poses = clean_poses
    
    if len(poses) < 2:
        print('Error: Trajectory has fewer than 2 poses')
        return False
        
    stamps = np.array([p[0] for p in poses])
    positions = np.array([p[1] for p in poses])
    rotations = Rotation.from_quat([p[2] for p in poses])
    slerp = Slerp(stamps, rotations)

    # Compute instantaneous angular velocity (rad/s) and net displacement over 1.5s window
    dt_traj = np.diff(stamps)
    dpos_traj = np.linalg.norm(np.diff(positions, axis=0), axis=1)
    lin_speed = np.concatenate([[0.0], dpos_traj / (dt_traj + 1e-6)])
    r_matrices = rotations.as_matrix()
    rel_rots = Rotation.from_matrix(np.matmul(np.transpose(r_matrices[:-1], (0, 2, 1)), r_matrices[1:]))
    ang_speed = np.concatenate([[0.0], np.linalg.norm(rel_rots.as_rotvec(), axis=1) / (dt_traj + 1e-6)])

    # Detect end of active walking phase using 15-pose (~1.5s) rolling net displacement > 8cm
    win = min(15, len(positions) - 1)
    if win > 1:
        net_disp = np.linalg.norm(positions[win:] - positions[:-win], axis=1)
        moving_indices = np.where(net_disp > 0.08)[0] + win
    else:
        moving_indices = np.where(lin_speed > 0.05)[0]

    if len(moving_indices) > 0:
        t_active_end = min(float(stamps[-1]), float(stamps[moving_indices[-1]]) + 2.0)
    else:
        t_active_end = float(stamps[-1])

    traj_diffs = np.diff(positions, axis=0)
    traj_dist = float(np.sum(np.linalg.norm(traj_diffs, axis=1)))
    duration = float(stamps[-1] - stamps[0])
    active_duration = float(t_active_end - stamps[0])
    print(f'Trajectory: {len(poses)} poses | Distance: {traj_dist:.2f} m | Total Duration: {duration:.2f} s (Active Walk: {active_duration:.2f} s)')

    def get_pose(t):
        t_clamped = np.clip(t, stamps[0], stamps[-1])
        idx = np.searchsorted(stamps, t_clamped)
        if idx == 0:
            pos = positions[0]
        elif idx >= len(stamps):
            pos = positions[-1]
        else:
            t0, t1 = stamps[idx-1], stamps[idx]
            a = (t_clamped - t0) / (t1 - t0 + 1e-9)
            pos = (1.0 - a) * positions[idx-1] + a * positions[idx]
        rot = slerp([t_clamped])[0].as_matrix()
        return pos, rot

    def get_ang_speed(t):
        idx = np.clip(np.searchsorted(stamps, t), 0, len(ang_speed) - 1)
        return float(ang_speed[idx])

    # Load Active FAST-LIVO2 IMU <- LiDAR Extrinsics
    fastlivo_yaml_path = '/home/scanar/scanarMini/src/FAST-LIVO2/config/robosense.yaml'
    with open(fastlivo_yaml_path, 'r') as f:
        fl_cfg = yaml.safe_load(f)
    fl_params = fl_cfg['/**']['ros__parameters']['extrin_calib']
    R_imu_lidar = np.array(fl_params['extrinsic_R'], dtype=np.float64).reshape(3, 3)
    t_imu_lidar = np.array(fl_params['extrinsic_T'], dtype=np.float64)
    T_imu_lidar = np.eye(4)
    T_imu_lidar[:3, :3] = R_imu_lidar
    T_imu_lidar[:3, 3] = t_imu_lidar
    T_lidar_imu = np.linalg.inv(T_imu_lidar)
    print(f'   [FAST-LIVO Extrinsics Loaded] R_imu_lidar trace = {np.trace(R_imu_lidar):.4f}')

    # Camera (cam0/depth optical) <- LiDAR (Device 2 / 45-deg experimental baseline)
    extrinsics_yaml_path = '/home/scanar/scanarMini/config/production_extrinsics.yaml'
    with open(extrinsics_yaml_path, 'r') as f:
        prod_ext = yaml.safe_load(f)
    T_cam0_lidar = np.array(prod_ext['active_transforms']['T_cam0_airy_lidar']['matrix'], dtype=np.float64)
    T_lidar_cam0 = np.linalg.inv(T_cam0_lidar)
    R_lidar_cam = T_lidar_cam0[:3, :3]
    t_lidar_cam = T_lidar_cam0[:3, 3]

    # Factory EEPROM Depth -> RGB Optical Transform
    T_rgb_depth = np.array(prod_ext['factory_transforms']['T_rgb_depth']['matrix'], dtype=np.float64)
    R_color_depth = T_rgb_depth[:3, :3]
    t_color_depth = T_rgb_depth[:3, 3]

    # Load Camera Intrinsics & Plumb-Bob Distortion
    with open(os.path.join(dataset_dir, 'depth_camera_info.json')) as f:
        d_info = json.load(f)
    fx_d, fy_d = d_info['k'][0], d_info['k'][4]
    cx_d, cy_d = d_info['k'][2], d_info['k'][5]
    W_d, H_d = d_info['width'], d_info['height']

    with open(os.path.join(dataset_dir, 'color_camera_info.json')) as f:
        c_info = json.load(f)
    K_c = np.array(c_info['k'], dtype=np.float64).reshape(3, 3)
    D_c = np.array(c_info['d'], dtype=np.float64).flatten()
    W_c, H_c = c_info['width'], c_info['height']
    print(f'   [Camera Intrinsics Loaded] RGB: {W_c}x{H_c}, D={D_c[:5]} | Depth: {W_d}x{H_d}')

    # 3. Process Airy LiDAR Data with Strict Schema Validation
    print('--> 2. Reconstructing Airy LiDAR Point Cloud with Strict Schema Validation...')
    with open(os.path.join(dataset_dir, 'lidar_index.json')) as f:
        lidar_entries = json.load(f)
        
    if len(lidar_entries) == 0:
        raise RuntimeError('FATAL: No LiDAR entries found in dataset!')

    def normalize_fields(raw_fields):
        if isinstance(raw_fields, dict):
            return {
                k: {'name': k, 'offset': int(v), 'datatype': 8 if k == 'timestamp' else 7, 'count': 1}
                for k, v in raw_fields.items()
            }
        return {f['name']: f for f in raw_fields}

    # Schema check on first entry
    first_fields = normalize_fields(lidar_entries[0]['fields'])
    for req_field in ['x', 'y', 'z']:
        if req_field not in first_fields:
            raise RuntimeError(f'FATAL: Mandatory PointCloud2 field "{req_field}" missing from metadata!')
            
    is_bigendian = lidar_entries[0].get('is_bigendian', False)
    print(f'   [Schema Verified] is_bigendian={is_bigendian}, Fields={list(first_fields.keys())}')

    # 3a. Automatic Initial Floor / Gravity Frame Alignment Check (handles recordings made with gravity_align_en=false)
    init_pts_list = []
    for entry in lidar_entries[:25]:
        fp = os.path.join(dataset_dir, 'lidar', entry['file'])
        if not os.path.exists(fp):
            continue
        with open(fp, 'rb') as f:
            raw_b = f.read()
        pt_s = entry['point_step']
        if len(raw_b) // pt_s == 0:
            continue
        f_d = normalize_fields(entry['fields'])
        is_be = entry.get('is_bigendian', False)
        xs = decode_field(raw_b, f_d['x']['offset'], f_d['x']['datatype'], f_d['x']['count'], pt_s, is_be)
        ys = decode_field(raw_b, f_d['y']['offset'], f_d['y']['datatype'], f_d['y']['count'], pt_s, is_be)
        zs = decode_field(raw_b, f_d['z']['offset'], f_d['z']['datatype'], f_d['z']['count'], pt_s, is_be)
        val = np.isfinite(xs) & np.isfinite(ys) & np.isfinite(zs) & (np.hypot(xs, ys) > 0.4) & (np.hypot(xs, ys) < 8.0)
        if np.sum(val) == 0:
            continue
        p_l = np.column_stack([xs[val], ys[val], zs[val]])
        p_i = (R_imu_lidar @ p_l.T).T + t_imu_lidar
        pos_0, rot_0 = get_pose(entry['stamp'])
        init_pts_list.append((rot_0 @ p_i.T).T + pos_0)

    G_R_init = np.eye(3, dtype=np.float64)
    gravity_tilt_deg = 0.0
    if len(init_pts_list) > 0:
        init_pts = np.vstack(init_pts_list)
        _, u_init = np.unique(np.floor(init_pts / 0.05).astype(np.int32), axis=0, return_index=True)
        init_ds = init_pts[u_init]
        if len(init_ds) > 200:
            tree_init = cKDTree(init_ds)
            _, idx_init = tree_init.query(init_ds, k=20)
            centered = init_ds[idx_init] - np.mean(init_ds[idx_init], axis=1, keepdims=True)
            _, s_i, vh_i = np.linalg.svd(centered, full_matrices=False)
            norms_i = vh_i[:, 2, :]
            flip_i = norms_i[:, 2] < 0
            norms_i[flip_i] = -norms_i[flip_i]
            plan_i = 1.0 - (s_i[:, 2] / (s_i[:, 1] + 1e-6))
            planar_mask = (plan_i > 0.65) & (norms_i[:, 2] > 0.5)
            if np.sum(planar_mask) > 50:
                cand_n = np.median(norms_i[planar_mask], axis=0)
                cand_n /= np.linalg.norm(cand_n)
                proj_h = init_ds @ cand_n
                floor_z0 = np.percentile(proj_h[planar_mask], 8)
                floor_inliers = planar_mask & (np.abs(proj_h - floor_z0) < 0.06)
                if np.sum(floor_inliers) > 30:
                    _, _, vh_f = np.linalg.svd(init_ds[floor_inliers] - np.mean(init_ds[floor_inliers], axis=0), full_matrices=False)
                    n_floor = vh_f[2, :]
                    if n_floor[2] < 0:
                        n_floor = -n_floor
                    gravity_tilt_deg = float(np.degrees(np.arccos(np.clip(n_floor[2], -1.0, 1.0))))
                    if gravity_tilt_deg > 3.0:
                        z_up = np.array([0.0, 0.0, 1.0])
                        r_axis = np.cross(n_floor, z_up)
                        r_axis_norm = np.linalg.norm(r_axis)
                        if r_axis_norm > 1e-6:
                            G_R_init = Rotation.from_rotvec((r_axis / r_axis_norm) * np.radians(gravity_tilt_deg)).as_matrix()
                            positions = (G_R_init @ positions.T).T
                            rotations = Rotation.from_matrix(G_R_init) * rotations
                            slerp = Slerp(stamps, rotations)
                            print(f'   [Gravity Alignment Applied] Corrected initial camera_init tilt of {gravity_tilt_deg:.2f} deg (floor normal={np.round(n_floor, 4).tolist()})')

    # Save trajectory files to out_dir (preserving raw and saving gravity-aligned if corrected)
    shutil.copyfile(tum_path, os.path.join(out_dir, 'trajectory_raw.tum'))
    with open(os.path.join(out_dir, 'trajectory.tum'), 'w') as f_tum_out:
        for s_t, p_t, r_t in zip(stamps, positions, rotations.as_quat()):
            f_tum_out.write(f'{s_t:.6f} {p_t[0]:.6f} {p_t[1]:.6f} {p_t[2]:.6f} {r_t[0]:.6f} {r_t[1]:.6f} {r_t[2]:.6f} {r_t[3]:.6f}\n')
    csv_path = os.path.join(dataset_dir, 'trajectory.csv')
    if os.path.exists(csv_path):
        shutil.copyfile(csv_path, os.path.join(out_dir, 'trajectory.csv'))

    # 3b. Reconstruct Airy LiDAR with Sub-Sweep Motion Deskewing & Vectorized Multi-Sweep Voxel Centroid Filtering
    voxel_size = 0.03
    sweep_pts_list = []
    sweep_int_list = []
    sweep_key_list = []
    raw_sweeps_used = 0

    for entry in lidar_entries:
        stamp = entry['stamp']
        if stamp > t_active_end:
            continue
        if get_ang_speed(stamp) > 0.85:
            continue

        fn = entry['file']
        pt_step = entry['point_step']
        f_dict = normalize_fields(entry['fields'])
        is_be = entry.get('is_bigendian', False)
        
        fp = os.path.join(dataset_dir, 'lidar', fn)
        if not os.path.exists(fp):
            continue
        with open(fp, 'rb') as f:
            raw_bytes = f.read()
            
        num_pts = len(raw_bytes) // pt_step
        if num_pts == 0:
            continue
            
        f_x = f_dict['x']
        f_y = f_dict['y']
        f_z = f_dict['z']
        
        xs = decode_field(raw_bytes, f_x['offset'], f_x['datatype'], f_x['count'], pt_step, is_be)
        ys = decode_field(raw_bytes, f_y['offset'], f_y['datatype'], f_y['count'], pt_step, is_be)
        zs = decode_field(raw_bytes, f_z['offset'], f_z['datatype'], f_z['count'], pt_step, is_be)
        
        if 'intensity' in f_dict:
            f_i = f_dict['intensity']
            intens = decode_field(raw_bytes, f_i['offset'], f_i['datatype'], f_i['count'], pt_step, is_be)
        elif 'reflectivity' in f_dict:
            f_i = f_dict['reflectivity']
            intens = decode_field(raw_bytes, f_i['offset'], f_i['datatype'], f_i['count'], pt_step, is_be)
        else:
            intens = np.zeros(num_pts, dtype=np.float32)

        has_ts = 'timestamp' in f_dict
        if has_ts:
            f_t = f_dict['timestamp']
            if f_t['datatype'] == 8:
                raw_arr = np.frombuffer(raw_bytes, dtype=np.uint8).reshape(num_pts, pt_step)
                endian_prefix = '>' if is_be else '<'
                ts_arr = raw_arr[:, f_t['offset']:f_t['offset']+8].copy().view(f'{endian_prefix}f8').flatten()
            else:
                ts_arr = decode_field(raw_bytes, f_t['offset'], f_t['datatype'], f_t['count'], pt_step, is_be).astype(np.float64)
        else:
            ts_arr = None
            
        valid = np.isfinite(xs) & np.isfinite(ys) & np.isfinite(zs)
        r_dist = np.hypot(xs, ys)
        valid &= (r_dist > 0.40) & (r_dist < 15.0)
        if ts_arr is not None:
            valid &= np.isfinite(ts_arr)
        
        pts_lidar = np.column_stack([xs[valid], ys[valid], zs[valid]]).astype(np.float64)
        intens_valid = np.nan_to_num(intens[valid], nan=0.0, posinf=255.0, neginf=0.0)
        
        if len(pts_lidar) == 0:
            continue

        pts_imu = (R_imu_lidar @ pts_lidar.T).T + t_imu_lidar
        pos_w, rot_w = get_pose(stamp)
        pts_w = (rot_w @ pts_imu.T).T + pos_w

        # Sub-sweep Slerp motion deskewing (10 sub-intervals per 100ms sweep)
        if ts_arr is not None:
            ts_v = ts_arr[valid]
            t_med = float(np.median(ts_v))
            in_sweep_t = np.abs(ts_v - t_med) <= 0.12
            if np.sum(in_sweep_t) > 100:
                t_min, t_max = float(np.min(ts_v[in_sweep_t])), float(np.max(ts_v[in_sweep_t]))
                if 0.005 < (t_max - t_min) <= 0.25:
                    bins = np.linspace(t_min, t_max, 11)
                    b_idx = np.clip(np.digitize(ts_v, bins) - 1, 0, 9)
                    for b in range(10):
                        m_b = (b_idx == b)
                        if np.any(m_b):
                            t_mid = 0.5 * (bins[b] + bins[b + 1])
                            p_b, r_b = get_pose(t_mid)
                            pts_w[m_b] = (r_b @ pts_imu[m_b].T).T + p_b

        # Ensure all transformed coordinates are strictly finite and within room bounds
        finite_w = np.all(np.isfinite(pts_w), axis=1) & (np.abs(pts_w[:, 0]) < 100.0) & (np.abs(pts_w[:, 1]) < 100.0) & (np.abs(pts_w[:, 2]) < 100.0)
        pts_w = pts_w[finite_w]
        intens_valid = intens_valid[finite_w]
        if len(pts_w) == 0:
            continue

        # Per-sweep 3cm voxel reduction
        v_sweep = np.floor(pts_w / voxel_size).astype(np.int64)
        packed_sw = ((v_sweep[:, 0] + 50000) << 40) | ((v_sweep[:, 1] + 50000) << 20) | (v_sweep[:, 2] + 50000)
        u_packed, u_sw = np.unique(packed_sw, return_index=True)
        sweep_pts_list.append(pts_w[u_sw])
        sweep_int_list.append(intens_valid[u_sw])
        sweep_key_list.append(u_packed)
        raw_sweeps_used += 1

    if len(sweep_pts_list) > 0:
        all_sw_pts = np.vstack(sweep_pts_list)
        all_sw_int = np.concatenate(sweep_int_list)
        all_sw_keys = np.concatenate(sweep_key_list)
        u_keys, inv_idx, counts = np.unique(all_sw_keys, return_inverse=True, return_counts=True)
        print(f'   Processed {raw_sweeps_used} active LiDAR sweeps ({len(all_sw_pts)} sweep-unique points, {len(u_keys)} candidate voxels)')

        min_sweep_hits = 8 if raw_sweeps_used > 100 else max(1, raw_sweeps_used // 10)
        keep_mask = counts >= min_sweep_hits
        sum_x = np.bincount(inv_idx, weights=all_sw_pts[:, 0])[keep_mask]
        sum_y = np.bincount(inv_idx, weights=all_sw_pts[:, 1])[keep_mask]
        sum_z = np.bincount(inv_idx, weights=all_sw_pts[:, 2])[keep_mask]
        sum_i = np.bincount(inv_idx, weights=all_sw_int)[keep_mask]
        kept_counts = counts[keep_mask].astype(np.float64)

        airy_downsampled = np.column_stack([sum_x / kept_counts, sum_y / kept_counts, sum_z / kept_counts])
        airy_intens_down = (sum_i / kept_counts).astype(np.float32)
    else:
        airy_downsampled = np.zeros((0, 3), dtype=np.float64)
        airy_intens_down = np.zeros(0, dtype=np.float32)
        min_sweep_hits = 1
    print(f'   Multi-Sweep Consistent Airy Surface Centroids (3cm voxel, >={min_sweep_hits} hits): {len(airy_downsampled)}')

    # Export airy_only.ply (XYZ + Genuine Datatype-Decoded Intensity) via vectorized structured buffer
    n_airy_out = len(airy_downsampled)
    airy_ply_hdr = (
        f'ply\nformat ascii 1.0\nelement vertex {n_airy_out}\n'
        f'property float x\nproperty float y\nproperty float z\nproperty float intensity\nend_header\n'
    )
    airy_table = np.zeros(n_airy_out, dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4'), ('i', 'f4')])
    if n_airy_out > 0:
        airy_table['x'] = airy_downsampled[:, 0]
        airy_table['y'] = airy_downsampled[:, 1]
        airy_table['z'] = airy_downsampled[:, 2]
        airy_table['i'] = airy_intens_down
    with open(os.path.join(out_dir, 'airy_only.ply'), 'wb') as f:
        f.write(airy_ply_hdr.encode('ascii'))
        if n_airy_out > 0:
            np.savetxt(f, airy_table, fmt='%.4f %.4f %.4f %.1f')

    # 4. M12.9B Heuristic Surface Orientation & Height Telemetry
    print('--> 3. Computing M12.9B Heuristic Surface Orientation Distribution...')
    tree_airy = cKDTree(airy_downsampled)

    # Compute architectural surface normals on an 8cm macro-voxel grid (k=25, ~25-30cm patch radius)
    # so multi-pass trajectory thickness (~4-6cm) does not masquerade as 3D isotropic scatter at 3cm scale.
    macro_vs = 0.08
    v_macro = np.floor(airy_downsampled / macro_vs).astype(np.int64)
    packed_m = ((v_macro[:, 0] + 50000) << 40) | ((v_macro[:, 1] + 50000) << 20) | (v_macro[:, 2] + 50000)
    _, inv_m, counts_m = np.unique(packed_m, return_inverse=True, return_counts=True)
    macro_pts = np.column_stack([
        np.bincount(inv_m, weights=airy_downsampled[:, 0]) / counts_m,
        np.bincount(inv_m, weights=airy_downsampled[:, 1]) / counts_m,
        np.bincount(inv_m, weights=airy_downsampled[:, 2]) / counts_m,
    ])
    tree_macro = cKDTree(macro_pts)
    k_macro = min(25, len(macro_pts))
    _, idx_m = tree_macro.query(macro_pts, k=k_macro)
    neigh_m = macro_pts[idx_m]
    cent_m = np.mean(neigh_m, axis=1, keepdims=True)
    _, s_m, vh_m = np.linalg.svd(neigh_m - cent_m, full_matrices=False)
    normals_m = vh_m[:, 2, :]
    plan_m = 1.0 - (s_m[:, 2] / (s_m[:, 1] + 1e-6))

    normals = normals_m[inv_m]
    planarities = plan_m[inv_m]

    z_coords = airy_downsampled[:, 2]
    horizontal_mask = (np.abs(normals[:, 2]) > 0.78) & (planarities > 0.45)

    if np.sum(horizontal_mask) > 100:
        z_horiz = z_coords[horizontal_mask]
        z_floor_cand = np.percentile(z_horiz, 8)
        z_ceil_cand = np.percentile(z_horiz, 92)
        floor_mask = horizontal_mask & (np.abs(z_coords - z_floor_cand) < 0.25)
        ceiling_mask = horizontal_mask & (np.abs(z_coords - z_ceil_cand) < 0.25)
    else:
        floor_mask = np.zeros(len(airy_downsampled), dtype=bool)
        ceiling_mask = np.zeros(len(airy_downsampled), dtype=bool)

    wall_mask = (np.abs(normals[:, 2]) < 0.35) & (planarities > 0.40) & (~floor_mask) & (~ceiling_mask)
    other_mask = (~floor_mask) & (~ceiling_mask) & (~wall_mask)
    
    floor_cnt = int(np.sum(floor_mask))
    ceiling_cnt = int(np.sum(ceiling_mask))
    wall_cnt = int(np.sum(wall_mask))
    other_cnt = int(np.sum(other_mask))
    total_surf = len(airy_downsampled)
    
    pct_floor = round(floor_cnt / total_surf * 100.0, 2)
    pct_ceiling = round(ceiling_cnt / total_surf * 100.0, 2)
    pct_wall = round(wall_cnt / total_surf * 100.0, 2)
    pct_other = round(other_cnt / total_surf * 100.0, 2)

    # 5. Process Gemini Depth + Real RGB with Plumb-Bob Lens Distortion
    print('--> 4. Reconstructing Gemini Point Cloud with Lens Distortion Corrected RGB Projection...')
    with open(os.path.join(dataset_dir, 'depth_index.json')) as f:
        depth_entries = json.load(f)
    with open(os.path.join(dataset_dir, 'color_index.json')) as f:
        color_entries = json.load(f)
        
    color_dict = {c['stamp']: c['file'] for c in color_entries}
    color_stamps = np.array(list(color_dict.keys()))

    gemini_w_points = []
    gemini_colors = []
    gemini_ranges = []
    gemini_has_rgb = []
    gemini_poses = []
    
    stride = 6
    v_idx, u_idx = np.indices((H_d, W_d))
    v_s = v_idx[::stride, ::stride].flatten()
    u_s = u_idx[::stride, ::stride].flatten()

    for d_entry in depth_entries[::3]:
        d_stamp = d_entry['stamp']
        if d_stamp > t_active_end or get_ang_speed(d_stamp) > 0.85:
            continue
        d_fp = os.path.join(dataset_dir, 'depth', d_entry['file'])
        d_img = cv2.imread(d_fp, cv2.IMREAD_UNCHANGED)
        if d_img is None:
            continue
            
        z_s = d_img[::stride, ::stride].flatten() / 1000.0
        valid = (z_s > 0.3) & (z_s < 5.0)
        if np.sum(valid) < 20:
            continue
            
        u_v = u_s[valid]
        v_v = v_s[valid]
        z_v = z_s[valid]
        
        x_v = (u_v - cx_d) * z_v / fx_d
        y_v = (v_v - cy_d) * z_v / fy_d
        pts_depth = np.column_stack([x_v, y_v, z_v])
        
        # Matching RGB image
        rgb_img = None
        if len(color_stamps) > 0:
            c_idx = np.argmin(abs(color_stamps - d_stamp))
            if abs(color_stamps[c_idx] - d_stamp) < 0.10:
                c_fp = os.path.join(dataset_dir, 'color', color_dict[color_stamps[c_idx]])
                c_bgr = cv2.imread(c_fp)
                if c_bgr is not None:
                    rgb_img = cv2.cvtColor(c_bgr, cv2.COLOR_BGR2RGB)
                    
        # Transform 3D points from depth optical frame to color optical frame
        pts_color = (R_color_depth @ pts_depth.T).T + t_color_depth
        
        # Plumb-bob lens distortion projection using cv2.projectPoints
        pts_colors_out = np.zeros((len(pts_depth), 3), dtype=np.uint8)
        pts_rgb_valid = np.zeros(len(pts_depth), dtype=bool)
        
        if rgb_img is not None:
            pts_proj_in = pts_color.reshape(-1, 1, 3).astype(np.float64)
            img_pts, _ = cv2.projectPoints(pts_proj_in, np.zeros(3), np.zeros(3), K_c, D_c)
            img_pts = img_pts.reshape(-1, 2)
            
            u_c = np.round(img_pts[:, 0]).astype(np.int32)
            v_c = np.round(img_pts[:, 1]).astype(np.int32)
            
            in_bounds = (pts_color[:, 2] > 0.1) & (u_c >= 0) & (u_c < W_c) & (v_c >= 0) & (v_c < H_c)
            pts_colors_out[in_bounds] = rgb_img[v_c[in_bounds], u_c[in_bounds]]
            pts_rgb_valid[in_bounds] = True
            pts_colors_out[~in_bounds] = [180, 180, 180]
        else:
            pts_colors_out[:] = [180, 180, 180]

        # Transform to world via authoritative transform chain:
        pts_lidar = (R_lidar_cam @ pts_depth.T).T + t_lidar_cam
        pts_imu = (R_imu_lidar @ pts_lidar.T).T + t_imu_lidar
        
        pos_w, rot_w = get_pose(d_stamp)
        pts_w = (rot_w @ pts_imu.T).T + pos_w
        
        gemini_w_points.append(pts_w)
        gemini_colors.append(pts_colors_out)
        gemini_ranges.append(z_v)
        gemini_has_rgb.append(pts_rgb_valid)
        gemini_poses.append((pos_w, rot_w))

    all_gemini_pts = np.vstack(gemini_w_points) if gemini_w_points else np.zeros((0, 3))
    all_gemini_cols = np.vstack(gemini_colors) if gemini_colors else np.zeros((0, 3), dtype=np.uint8)
    all_gemini_ranges = np.concatenate(gemini_ranges) if gemini_ranges else np.zeros(0)
    all_gemini_has_rgb = np.concatenate(gemini_has_rgb) if gemini_has_rgb else np.zeros(0, dtype=bool)

    if len(all_gemini_pts) > 0:
        g_voxel_keys = np.floor(all_gemini_pts / voxel_size).astype(np.int64)
        _, g_u_idx = np.unique(g_voxel_keys, axis=0, return_index=True)
        gemini_downsampled = all_gemini_pts[g_u_idx]
        gemini_cols_down = all_gemini_cols[g_u_idx]
        gemini_ranges_down = all_gemini_ranges[g_u_idx]
        gemini_has_rgb_down = all_gemini_has_rgb[g_u_idx]
    else:
        gemini_downsampled = np.zeros((0, 3))
        gemini_cols_down = np.zeros((0, 3), dtype=np.uint8)
        gemini_ranges_down = np.zeros(0)
        gemini_has_rgb_down = np.zeros(0, dtype=bool)

    print(f'   Downsampled Gemini Points (with Real RGB): {len(gemini_downsampled)}')

    def _write_rgb_ply(filepath, xyz, rgb):
        n = len(xyz)
        hdr = (
            f'ply\nformat ascii 1.0\nelement vertex {n}\n'
            f'property float x\nproperty float y\nproperty float z\n'
            f'property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n'
        )
        tbl = np.zeros(n, dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4'), ('r', 'u1'), ('g', 'u1'), ('b', 'u1')])
        if n > 0:
            tbl['x'], tbl['y'], tbl['z'] = xyz[:, 0], xyz[:, 1], xyz[:, 2]
            tbl['r'], tbl['g'], tbl['b'] = rgb[:, 0], rgb[:, 1], rgb[:, 2]
        with open(filepath, 'wb') as f:
            f.write(hdr.encode('ascii'))
            if n > 0:
                np.savetxt(f, tbl, fmt='%.4f %.4f %.4f %d %d %d')

    # Export gemini_only.ply (Genuine Real RGB)
    _write_rgb_ply(os.path.join(out_dir, 'gemini_only.ply'), gemini_downsampled, gemini_cols_down)

    # 6. Export Raw Registered Overlay (Explicitly labeled)
    print('--> 5. Exporting Raw Registered Overlay...')
    airy_colors = np.full((len(airy_downsampled), 3), 200, dtype=np.uint8) # neutral grey
    overlay_pts = np.vstack([airy_downsampled, gemini_downsampled]) if len(gemini_downsampled) > 0 else airy_downsampled
    overlay_cols = np.vstack([airy_colors, gemini_cols_down]) if len(gemini_downsampled) > 0 else airy_colors

    _write_rgb_ply(os.path.join(out_dir, 'raw_registered_overlay.ply'), overlay_pts, overlay_cols)

    # 7. Mutually Exclusive Diagnostic Gemini Admission Partition
    print('--> 6. Evaluating Mutually Exclusive Gemini Admission Partition...')
    # Categories (strictly mutually exclusive):
    # 1. accepted_matched: planar agreement <= 80mm
    # 2. accepted_unmatched_novel: near-field (<2.0m) valid point with no nearby Airy surface
    # 3. rejected_conflict: planar disagreement > 80mm
    # 4. rejected_range_or_unconstrained: r < 0.3m, r > 4.0m, or unconstrained far unmatched
    
    accepted_matched_mask = np.zeros(len(gemini_downsampled), dtype=bool)
    accepted_novel_mask = np.zeros(len(gemini_downsampled), dtype=bool)
    rejected_conflict_mask = np.zeros(len(gemini_downsampled), dtype=bool)
    rejected_range_mask = np.zeros(len(gemini_downsampled), dtype=bool)
    
    g_residuals = []
    
    if len(gemini_downsampled) > 0 and len(airy_downsampled) > 0:
        g_dists, g_idx = tree_airy.query(gemini_downsampled, k=8, distance_upper_bound=0.15)
        for i in range(len(gemini_downsampled)):
            r_val = gemini_ranges_down[i]

            if r_val < 0.30 or r_val > 4.00:
                rejected_range_mask[i] = True
                continue

            v_idx = g_idx[i][g_dists[i] < 0.15]

            if len(v_idx) >= 5:
                nearest_idx = v_idx[0]
                if planarities[nearest_idx] >= 0.40:  # Planar Airy macro-neighborhood
                    cent = np.mean(airy_downsampled[v_idx], axis=0)
                    norm = normals[nearest_idx]
                    res_mm = abs(np.dot(gemini_downsampled[i] - cent, norm)) * 1000.0
                    g_residuals.append(res_mm)

                    if res_mm <= 80.0:
                        accepted_matched_mask[i] = True
                    else:
                        rejected_conflict_mask[i] = True
                else:  # Non-planar Airy neighborhood
                    if r_val <= 2.0:
                        accepted_novel_mask[i] = True
                    else:
                        rejected_range_mask[i] = True
            else:  # Lacked >= 5 Airy neighbors within 150mm
                if r_val <= 2.0:
                    accepted_novel_mask[i] = True
                else:
                    rejected_range_mask[i] = True
                    
    total_g = len(gemini_downsampled)
    n_acc_match = int(np.sum(accepted_matched_mask))
    n_acc_novel = int(np.sum(accepted_novel_mask))
    n_rej_conf = int(np.sum(rejected_conflict_mask))
    n_rej_range = int(np.sum(rejected_range_mask))
    
    # Strict partition assertion
    assert (n_acc_match + n_acc_novel + n_rej_conf + n_rej_range) == total_g, 'Partition error!'
    
    total_accepted = n_acc_match + n_acc_novel
    total_rejected = n_rej_conf + n_rej_range
    total_unmatched = n_acc_novel + n_rej_range
    
    pct_acc = round(total_accepted / (total_g + 1e-6) * 100.0, 2)
    pct_rej = round(total_rejected / (total_g + 1e-6) * 100.0, 2)
    pct_unm = round(total_unmatched / (total_g + 1e-6) * 100.0, 2)

    # Export experimental_gated_fused.ply
    all_accepted_mask = accepted_matched_mask | accepted_novel_mask
    gated_gemini_pts = gemini_downsampled[all_accepted_mask]
    gated_gemini_cols = gemini_cols_down[all_accepted_mask]
    gated_fused_pts = np.vstack([airy_downsampled, gated_gemini_pts]) if len(gated_gemini_pts) > 0 else airy_downsampled
    gated_fused_cols = np.vstack([airy_colors, gated_gemini_cols]) if len(gated_gemini_pts) > 0 else airy_colors

    _write_rgb_ply(os.path.join(out_dir, 'experimental_gated_fused.ply'), gated_fused_pts, gated_fused_cols)

    # 8. Z-buffered RGB-observable Airy Surface Coverage from Actual Recorded RGB Poses
    print('--> 7. Computing Z-Buffered RGB-Observable Surface Coverage & Depth Coverage...')
    num_voxels = len(airy_downsampled)
    airy_surf_rgb_observed = np.zeros(num_voxels, dtype=bool)
    
    T_depth_opt_cam0 = np.eye(4)
    T_color_opt_lidar = T_rgb_depth @ T_depth_opt_cam0 @ T_cam0_lidar
    T_color_opt_imu = T_color_opt_lidar @ T_lidar_imu
    R_col_imu = T_color_opt_imu[:3, :3]
    t_col_imu = T_color_opt_imu[:3, 3]

    # Process each recorded RGB keyframe from color_index.json
    z_buf_h, z_buf_w = H_c // 4, W_c // 4
    scale_x, scale_y = z_buf_w / W_c, z_buf_h / H_c

    for c_entry in color_entries[::2]: # Check every 2nd actual recorded RGB keyframe
        c_stamp = c_entry['stamp']
        pos_w, rot_w = get_pose(c_stamp)
        
        pts_rel_w = airy_downsampled - pos_w
        pts_imu_k = (rot_w.T @ pts_rel_w.T).T
        pts_col_k = (R_col_imu @ pts_imu_k.T).T + t_col_imu
        
        z_col = pts_col_k[:, 2]
        in_range = (z_col > 0.40) & (z_col < 6.00)
        if np.sum(in_range) == 0:
            continue
            
        pts_cand = pts_col_k[in_range]
        cand_indices = np.where(in_range)[0]
        
        img_pts, _ = cv2.projectPoints(pts_cand.reshape(-1, 1, 3).astype(np.float64), np.zeros(3), np.zeros(3), K_c, D_c)
        img_pts = img_pts.reshape(-1, 2)
        
        u_proj = img_pts[:, 0]
        v_proj = img_pts[:, 1]
        
        in_img = (u_proj >= 0) & (u_proj < W_c) & (v_proj >= 0) & (v_proj < H_c)
        if np.sum(in_img) == 0:
            continue
            
        u_valid = np.clip((u_proj[in_img] * scale_x).astype(np.int32), 0, z_buf_w - 1)
        v_valid = np.clip((v_proj[in_img] * scale_y).astype(np.int32), 0, z_buf_h - 1)
        z_valid = pts_cand[in_img, 2]
        orig_idx = cand_indices[in_img]
        
        # Build Z-buffer with vectorized min
        z_buf = np.full((z_buf_h, z_buf_w), np.inf, dtype=np.float32)
        np.minimum.at(z_buf, (v_valid, u_valid), z_valid)
        
        # Mark voxels within 150mm visibility tolerance of closest surface
        obs_mask = z_valid <= (z_buf[v_valid, u_valid] + 0.15)
        airy_surf_rgb_observed[orig_idx[obs_mask]] = True

    # Check Depth Observation (Airy voxel has reconstructed Gemini point within 100mm)
    airy_surf_depth_observed = np.zeros(num_voxels, dtype=bool)
    if len(gemini_downsampled) > 0:
        tree_gem = cKDTree(gemini_downsampled)
        g_near_dists, _ = tree_gem.query(airy_downsampled, k=1, distance_upper_bound=0.10)
        airy_surf_depth_observed = (g_near_dists < 0.10)

    airy_surf_dual_observed = airy_surf_rgb_observed & airy_surf_depth_observed

    rgb_cov_pct = round(np.sum(airy_surf_rgb_observed) / (num_voxels + 1e-6) * 100.0, 2)
    depth_cov_pct = round(np.sum(airy_surf_depth_observed) / (num_voxels + 1e-6) * 100.0, 2)
    dual_cov_pct = round(np.sum(airy_surf_dual_observed) / (num_voxels + 1e-6) * 100.0, 2)

    # 9. Cross-sensor Residual Distribution
    g_res_arr = np.array(g_residuals)
    res_p50 = float(np.percentile(g_res_arr, 50)) if len(g_res_arr) > 0 else 0.0
    res_p80 = float(np.percentile(g_res_arr, 80)) if len(g_res_arr) > 0 else 0.0
    res_p95 = float(np.percentile(g_res_arr, 95)) if len(g_res_arr) > 0 else 0.0
    res_mean = float(np.mean(g_res_arr)) if len(g_res_arr) > 0 else 0.0

    # Room Bounding Box
    bbox_min = np.min(airy_downsampled, axis=0).tolist() if len(airy_downsampled) > 0 else [0,0,0]
    bbox_max = np.max(airy_downsampled, axis=0).tolist() if len(airy_downsampled) > 0 else [0,0,0]
    bbox_extents = [round(bbox_max[i] - bbox_min[i], 2) for i in range(3)]

    # Final Telemetry JSON
    telemetry = {
        'test_id': 'M12.9B_M12.9C',
        'duration_s': round(duration, 2),
        'active_walk_duration_s': round(active_duration, 2),
        'gravity_alignment_correction_deg': round(gravity_tilt_deg, 2),
        'trajectory_distance_m': round(traj_dist, 2),
        'room_bounding_box_extents_m': bbox_extents,
        'room_bounding_box_min_m': [round(x, 2) for x in bbox_min],
        'room_bounding_box_max_m': [round(x, 2) for x in bbox_max],
        'm12_9b_heuristic_airy_surface_distribution': {
            'note': 'Heuristic normal and height-percentile surface orientation distribution',
            'floor_pct': pct_floor,
            'wall_pct': pct_wall,
            'ceiling_pct': pct_ceiling,
            'other_pct': pct_other,
            'floor_voxels': floor_cnt,
            'wall_voxels': wall_cnt,
            'ceiling_voxels': ceiling_cnt,
            'other_voxels': other_cnt,
            'total_voxels': total_surf
        },
        'm12_9c_point_counts': {
            'airy_surface_points': len(airy_downsampled),
            'gemini_surface_points': len(gemini_downsampled),
            'raw_registered_overlay_points': len(overlay_pts),
            'experimental_gated_fused_points': len(gated_fused_pts)
        },
        'm12_9c_mutually_exclusive_admission_partition': {
            'accepted_matched_pct': round(n_acc_match / (total_g + 1e-6) * 100.0, 2),
            'accepted_unmatched_novel_pct': round(n_acc_novel / (total_g + 1e-6) * 100.0, 2),
            'rejected_conflict_pct': round(n_rej_conf / (total_g + 1e-6) * 100.0, 2),
            'rejected_range_or_unconstrained_pct': round(n_rej_range / (total_g + 1e-6) * 100.0, 2),
            'total_accepted_pct': pct_acc,
            'total_rejected_pct': pct_rej,
            'total_unmatched_geometry_pct': pct_unm,
            'counts': {
                'accepted_matched': n_acc_match,
                'accepted_unmatched_novel': n_acc_novel,
                'rejected_conflict': n_rej_conf,
                'rejected_range_or_unconstrained': n_rej_range,
                'total_gemini_evaluated': total_g
            }
        },
        'm12_9c_surface_observation_coverage': {
            'z_buffered_rgb_observable_airy_surface_coverage_from_actual_recorded_rgb_poses_pct': rgb_cov_pct,
            'gemini_reconstructed_depth_surface_coverage_100mm_pct': depth_cov_pct,
            'rgb_plus_depth_dual_observation_pct': dual_cov_pct,
            'evaluator_parameters': {
                'rgb_keyframe_sampling_interval': 'every 2nd recorded keyframe',
                'z_buffer_resolution': f'{z_buf_w}x{z_buf_h}',
                'visibility_tolerance_mm': 150.0,
                'rgb_range_gate_m': [0.40, 6.00]
            }
        },
        'm12_9c_cross_sensor_residuals_mm': {
            'correspondence_gate_mm': 150.0,
            'evaluated_points': len(g_res_arr),
            'P50_mm': round(res_p50, 2),
            'P80_mm': round(res_p80, 2),
            'P95_mm': round(res_p95, 2),
            'mean_mm': round(res_mean, 2)
        }
    }

    with open(os.path.join(out_dir, 'coverage_metadata.json'), 'w') as f:
        json.dump(telemetry, f, indent=2)

    print('\n' + '='*70)
    print('=== M12.9B / M12.9C RECONSTRUCTION COMPLETE ===')
    print(json.dumps(telemetry, indent=2))
    print('='*70)
    return True

if __name__ == '__main__':
    d_dir = sys.argv[1] if len(sys.argv) > 1 else '/home/scanar/scanarMini/diagnostics/m12_9_room_dataset'
    o_dir = sys.argv[2] if len(sys.argv) > 2 else '/home/scanar/scanarMini/diagnostics/m12_9_room_export'
    process_room_scan(d_dir, o_dir)
