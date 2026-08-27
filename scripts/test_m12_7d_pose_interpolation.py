#!/usr/bin/env python3
import os
import sys
import math
import numpy as np
import pandas as pd

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

class PoseInterpolator:
    def __init__(self, traj_csv_path):
        df = pd.read_csv(traj_csv_path)
        self.times = df['timestamp'].values
        self.positions = np.column_stack((df['x_m'].values, df['y_m'].values, df['z_m'].values))
        self.quats = np.column_stack((df['qx'].values, df['qy'].values, df['qz'].values, df['qw'].values))
        self.rots = [quat_to_rot(q) for q in self.quats]
        print(f"Loaded {len(self.times)} trajectory poses from {traj_csv_path}")
        print(f"Time span: {self.times[0]:.3f} -> {self.times[-1]:.3f} ({self.times[-1]-self.times[0]:.2f}s)")

    def query(self, t_query):
        if len(self.times) == 0:
            return None
        if t_query <= self.times[0]:
            return {
                't_query': t_query,
                'is_clamped': True,
                'clamp_side': 'start',
                't0': self.times[0],
                't1': self.times[0],
                'alpha': 0.0,
                'pos': self.positions[0],
                'quat': self.quats[0],
                'rot': self.rots[0]
            }
        if t_query >= self.times[-1]:
            return {
                't_query': t_query,
                'is_clamped': True,
                'clamp_side': 'end',
                't0': self.times[-1],
                't1': self.times[-1],
                'alpha': 1.0,
                'pos': self.positions[-1],
                'quat': self.quats[-1],
                'rot': self.rots[-1]
            }
        idx = np.searchsorted(self.times, t_query)
        t0 = self.times[idx - 1]
        t1 = self.times[idx]
        p0, p1 = self.positions[idx - 1], self.positions[idx]
        q0, q1 = self.quats[idx - 1], self.quats[idx]
        dt = t1 - t0
        alpha = (t_query - t0) / dt if dt > 1e-6 else 0.0
        alpha = max(0.0, min(1.0, alpha))
        p_interp = (1.0 - alpha) * p0 + alpha * p1
        q_interp = slerp_quat(q0, q1, alpha)
        r_interp = quat_to_rot(q_interp)
        return {
            't_query': t_query,
            'is_clamped': False,
            'clamp_side': 'none',
            't0': t0,
            't1': t1,
            'alpha': alpha,
            'pos': p_interp,
            'quat': q_interp,
            'rot': r_interp
        }

def run_diagnostic():
    traj_path = "/home/scanar/scanarMini/data/M12_7_Controlled_Motion/trajectory.csv"
    if not os.path.exists(traj_path):
        print(f"Error: {traj_path} not found")
        return

    interp = PoseInterpolator(traj_path)

    # Pick 3 representative frames during active physical motion
    mid_idx = len(interp.times) // 2
    sample_indices = [mid_idx - 30, mid_idx, mid_idx + 30]
    sample_indices = [i for i in sample_indices if 0 <= i < len(interp.times)]

    offsets_ms = [-200, -100, -50, 0, 50, 100, 200]

    # Synthesize a standard depth test patch (1000 points on a planar wall at 2.0m distance)
    xx, yy = np.meshgrid(np.linspace(-0.5, 0.5, 30), np.linspace(-0.5, 0.5, 30))
    zz = np.full_like(xx, 2.0)
    pts_cam = np.column_stack((xx.flatten(), yy.flatten(), zz.flatten()))

    # Calibrated T_DEPTH_IMU
    T_DEPTH_IMU = np.array([
        [ 0.999925, -0.003702,  0.011654,  0.004787],
        [ 0.011652, -0.000495, -0.999932, -0.050496],
        [ 0.003709,  0.999992, -0.000451, -0.019135],
        [ 0.0,       0.0,       0.0,       1.0     ]
    ], dtype=np.float64)
    pts_imu = (T_DEPTH_IMU[:3, :3] @ pts_cam.T).T + T_DEPTH_IMU[:3, 3]

    print("\n" + "="*145)
    print(f"{'Frame / Epoch':<20} | {'Offset':<8} | {'t_query':<18} | {'Bracket [t0, t1] (dt ms)':<26} | {'Alpha':<6} | {'Clamped?':<9} | {'Delta Pos (mm)':<15} | {'Delta Rot (deg)':<15} | {'Cloud P50 Disp (mm)'}")
    print("="*145)

    for s_idx in sample_indices:
        t_base = interp.times[s_idx]
        base_res = interp.query(t_base)
        pts_w_base = (base_res['rot'] @ pts_imu.T).T + base_res['pos']

        p_prev = interp.positions[max(0, s_idx-1)]
        p_next = interp.positions[min(s_idx+1, len(interp.times)-1)]
        t_span = interp.times[min(s_idx+1, len(interp.times)-1)] - interp.times[max(0, s_idx-1)]
        v_est = np.linalg.norm(p_next - p_prev) / t_span if t_span > 1e-6 else 0.0

        print(f"\n--- Target Frame at t = {t_base:.4f} (Index {s_idx}, Estimated Velocity = {v_est:.3f} m/s) ---")

        for off_ms in offsets_ms:
            t_q = t_base + off_ms / 1000.0
            q_res = interp.query(t_q)

            # Positional and angular difference from base pose
            d_pos_mm = np.linalg.norm(q_res['pos'] - base_res['pos']) * 1000.0

            # Angular difference (geodesic angle)
            q_base_norm = base_res['quat'] / np.linalg.norm(base_res['quat'])
            q_query_norm = q_res['quat'] / np.linalg.norm(q_res['quat'])
            dot_q = abs(float(np.sum(q_base_norm * q_query_norm)))
            d_rot_deg = math.degrees(2.0 * math.acos(max(-1.0, min(1.0, dot_q))))

            # Point cloud world displacement
            pts_w_query = (q_res['rot'] @ pts_imu.T).T + q_res['pos']
            cloud_disp_mm = np.linalg.norm(pts_w_query - pts_w_base, axis=1) * 1000.0
            p50_disp = np.percentile(cloud_disp_mm, 50)
            p95_disp = np.percentile(cloud_disp_mm, 95)

            dt_ms = (q_res['t1'] - q_res['t0']) * 1000.0
            bracket_str = f"[{q_res['t0']:.3f}, {q_res['t1']:.3f}] ({dt_ms:.1f}ms)"
            clamped_str = f"YES ({q_res['clamp_side']})" if q_res['is_clamped'] else "NO"

            print(f"t={t_base:.4f} (idx {s_idx:<3}) | {off_ms:+5d}ms | {t_q:<18.4f} | {bracket_str:<26} | {q_res['alpha']:<6.3f} | {clamped_str:<9} | {d_pos_mm:<15.2f} | {d_rot_deg:<15.3f} | {p50_disp:8.2f} mm (P95: {p95_disp:8.2f} mm)")

if __name__ == '__main__':
    run_diagnostic()
