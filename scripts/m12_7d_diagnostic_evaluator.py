#!/usr/bin/env python3
import os
import sys
import math
import numpy as np
from scipy.spatial import cKDTree

def quat_to_rot(q):
    qx, qy, qz, qw = q
    n = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    if n < 1e-10:
        return np.eye(3)
    qx, qy, qz, qw = qx/n, qy/n, qz/n, qw/n
    return np.array([
        [1.0 - 2.0*(qy*qy + qz*qz), 2.0*(qx*qy - qz*qw),       2.0*(qx*qz + qy*qw)],
        [2.0*(qx*qy + qz*qw),       1.0 - 2.0*(qx*qx + qz*qz), 2.0*(qy*qz - qx*qw)],
        [2.0*(qx*qz - qy*qw),       2.0*(qy*qz + qx*qw),       1.0 - 2.0*(qx*qx + qy*qy)]
    ], dtype=np.float64)

def slerp_quat(q0, q1, alpha):
    q0 = np.asarray(q0, dtype=np.float64)
    q1 = np.asarray(q1, dtype=np.float64)
    n0, n1 = np.linalg.norm(q0), np.linalg.norm(q1)
    if n0 < 1e-10 or n1 < 1e-10:
        return q0
    q0 = q0 / n0
    q1 = q1 / n1
    dot = float(np.sum(q0 * q1))
    if dot < 0.0:
        q1 = -q1
        dot = -dot
    if dot > 0.9995:
        q_interp = (1.0 - alpha) * q0 + alpha * q1
        norm_interp = np.linalg.norm(q_interp)
        return q_interp / norm_interp if norm_interp > 1e-10 else q0
    theta = math.acos(max(-1.0, min(1.0, dot)))
    sin_theta = math.sin(theta)
    if abs(sin_theta) < 1e-6:
        return q0
    w0 = math.sin((1.0 - alpha) * theta) / sin_theta
    w1 = math.sin(alpha * theta) / sin_theta
    q_interp = w0 * q0 + w1 * q1
    return q_interp / np.linalg.norm(q_interp)

T_DEPTH_IMU = np.array([
    [ 0.999925, -0.003702,  0.011654,  0.004787],
    [ 0.011652, -0.000495, -0.999932, -0.050496],
    [ 0.003709,  0.999992, -0.000451, -0.019135],
    [ 0.0,       0.0,       0.0,       1.0     ]
], dtype=np.float64)

class InstantaneousCoTemporalEvaluator:
    def __init__(self, odom_records, depth_k=None):
        self.odom_times = [r[0] for r in odom_records]
        self.odom_records = odom_records # list of (t, pos, quat, rot)
        self.depth_k = depth_k if depth_k is not None else np.array([[460., 0., 424.], [0., 460., 240.], [0., 0., 1.]])
        self.airy_sweeps = [] # list of (t_sweep_end, xyz_world, tree)

    def add_airy_sweep(self, t_sweep_end, xyz_world):
        if len(xyz_world) < 10:
            return
        tree = cKDTree(xyz_world)
        self.airy_sweeps.append((t_sweep_end, xyz_world, tree))

    def get_interpolated_pose(self, t_query):
        if len(self.odom_times) == 0:
            return None, None
        if t_query <= self.odom_times[0]:
            return self.odom_records[0][1], self.odom_records[0][3]
        if t_query >= self.odom_times[-1]:
            return self.odom_records[-1][1], self.odom_records[-1][3]
        idx = np.searchsorted(self.odom_times, t_query)
        t0, p0, q0, r0 = self.odom_records[idx - 1]
        t1, p1, q1, r1 = self.odom_records[idx]
        dt = t1 - t0
        alpha = (t_query - t0) / dt if dt > 1e-6 else 0.0
        alpha = max(0.0, min(1.0, alpha))
        p_interp = (1.0 - alpha) * p0 + alpha * p1
        q_interp = slerp_quat(q0, q1, alpha)
        r_interp = quat_to_rot(q_interp)
        return p_interp, r_interp

    def backproject_depth(self, cv_depth, pos_w, rot_w, decimate=4):
        h, w = cv_depth.shape[:2]
        depth_m = cv_depth.astype(np.float32) / 1000.0
        u_grid, v_grid = np.meshgrid(np.arange(0, w, decimate), np.arange(0, h, decimate))
        z = depth_m[v_grid, u_grid]
        valid = (z > 0.4) & (z < 5.0)
        if np.sum(valid) == 0:
            return np.empty((0, 3)), np.empty((0,))
        u_v, v_v, z_v = u_grid[valid], v_grid[valid], z[valid]
        fx, fy = self.depth_k[0, 0], self.depth_k[1, 1]
        cx, cy = self.depth_k[0, 2], self.depth_k[1, 2]
        x_c = (u_v - cx) * z_v / fx
        y_c = (v_v - cy) * z_v / fy
        pts_cam = np.column_stack((x_c, y_c, z_v))
        pts_imu = (T_DEPTH_IMU[:3, :3] @ pts_cam.T).T + T_DEPTH_IMU[:3, 3]
        pts_world = (rot_w @ pts_imu.T).T + pos_w
        return pts_world, z_v

    def evaluate_co_temporal_frame_sensitivity(self, t_depth, cv_depth, offsets_ms=range(-100, 105, 10)):
        if len(self.airy_sweeps) == 0:
            return None

        # 1. Establish physical co-temporal Airy reference sweep based on ORIGINAL acquisition epoch
        best_sweep_idx = None
        min_dt = 1e9
        for idx, (t_l, _, _) in enumerate(self.airy_sweeps):
            dt = abs(t_depth - t_l)
            if dt < min_dt:
                min_dt = dt
                best_sweep_idx = idx

        if best_sweep_idx is None or min_dt > 0.10: # within 100ms
            return None

        t_airy_ref, airy_xyz_ref, airy_tree_ref = self.airy_sweeps[best_sweep_idx]

        # 2. Precompute Tangent Planes on candidate Airy points in this single sweep
        ball_radius = 0.15
        eval_results = {
            't_depth': t_depth,
            't_airy_ref': t_airy_ref,
            'dt_phys_ms': min_dt * 1000.0,
            'sweep_offsets': {}
        }

        for off_ms in offsets_ms:
            off_s = off_ms / 1000.0
            pos_w, rot_w = self.get_interpolated_pose(t_depth + off_s)
            if pos_w is None or rot_w is None:
                continue

            pts_w, z_range = self.backproject_depth(cv_depth, pos_w, rot_w, decimate=4)
            if len(pts_w) < 20:
                continue

            # Query against the FIXED reference Airy sweep
            dists, idxs = airy_tree_ref.query(pts_w, k=1, workers=2)
            valid_m = (dists <= ball_radius)
            if np.sum(valid_m) < 10:
                continue

            unique_a = np.unique(idxs[valid_m])
            balls = airy_tree_ref.query_ball_point(airy_xyz_ref[unique_a], r=ball_radius, workers=2)
            tangs = {}
            for j, u in enumerate(unique_a):
                nb_i = balls[j]
                if len(nb_i) >= 6:
                    nb = airy_xyz_ref[nb_i]
                    cent = np.mean(nb, axis=0)
                    cov = np.cov((nb - cent).T)
                    evals, evecs = np.linalg.eigh(cov)
                    norm = evecs[:, 0]
                    if evals[0] < 0.008:
                        tangs[u] = (norm, cent)

            # Evaluate signed point-to-plane distance
            pt2plane = []
            range_list = []
            for i, p_g in enumerate(pts_w):
                if not valid_m[i]:
                    continue
                u = idxs[i]
                if u in tangs:
                    norm, cent = tangs[u]
                    signed_d = float(np.dot(p_g - cent, norm))
                    pt2plane.append(signed_d)
                    range_list.append(z_range[i])

            if len(pt2plane) >= 10:
                abs_mm = np.abs(np.array(pt2plane)) * 1000.0
                sgn_mm = np.array(pt2plane) * 1000.0
                ranges = np.array(range_list)

                # Overall & Range-binned metrics
                res_dict = {
                    'offset_ms': off_ms,
                    'pts_evaluated': len(abs_mm),
                    'p50_mm': float(round(np.percentile(abs_mm, 50), 2)),
                    'p90_mm': float(round(np.percentile(abs_mm, 90), 2)),
                    'p95_mm': float(round(np.percentile(abs_mm, 95), 2)),
                    'signed_mean_mm': float(round(np.mean(sgn_mm), 2)),
                    'std_mm': float(round(np.std(sgn_mm), 2)),
                }

                # Range Bins
                near_m = (ranges >= 0.5) & (ranges < 1.5)
                mid_m = (ranges >= 1.5) & (ranges < 2.5)
                far_m = (ranges >= 2.5) & (ranges <= 4.0)

                res_dict['near_p50_mm'] = float(round(np.percentile(abs_mm[near_m], 50), 2)) if np.sum(near_m) > 5 else None
                res_dict['mid_p50_mm'] = float(round(np.percentile(abs_mm[mid_m], 50), 2)) if np.sum(mid_m) > 5 else None
                res_dict['far_p50_mm'] = float(round(np.percentile(abs_mm[far_m], 50), 2)) if np.sum(far_m) > 5 else None

                eval_results['sweep_offsets'][f"{off_ms:+d}ms"] = res_dict

        return eval_results

print("InstantaneousCoTemporalEvaluator module loaded successfully.")
