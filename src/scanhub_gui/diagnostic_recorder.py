#!/usr/bin/env python3
"""
scanHUB Diagnostic Recorder (Milestone 4.5)
Hardware & SLAM Diagnostic Data Acquisition System for scanR Platform.

Records synchronized multi-sensor telemetry, TF transforms, SLAM pose, GUI metrics,
system thermals/resources, and optionally an automated ROS 2 bag file into a structured directory:
diagnostics/walk_YYMMDD_HHMMSS/
  ├── imu.csv
  ├── lidar.csv
  ├── camera.csv
  ├── slam.csv
  ├── tf.csv
  ├── gui.csv
  ├── system.csv
  ├── diagnostic_bag/ (ROS 2 bag)
  └── summary.txt (Auto-generated post-run diagnostic report)
"""

import os
import sys
import csv
import time
import math
import subprocess
import threading
from datetime import datetime

try:
    import psutil
except ImportError:
    psutil = None


class DiagnosticRecorder:
    def __init__(self, base_dir="/home/scanar/scanarMini/diagnostics"):
        self.base_dir = base_dir
        self.is_recording = False
        self.session_dir = None
        self.bag_process = None
        
        self._lock = threading.Lock()
        self.csv_files = {}
        self.csv_writers = {}
        
        self.start_sys_time = 0.0
        self.counts = {
            "imu": 0,
            "lidar": 0,
            "camera": 0,
            "slam": 0,
            "tf": 0,
            "gui": 0,
            "system": 0
        }
        
        self.time_history = {
            "imu": [],
            "lidar": [],
            "camera": [],
            "slam": []
        }
        
        self.point_counts = []
        self.slam_poses = [] # (sys_time, x, y, z, yaw)

    def start_session(self, label="walk", record_rosbag=True):
        with self._lock:
            if self.is_recording:
                return False, "Diagnostic recording already active."
            
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            folder_name = f"{label}_{timestamp}"
            self.session_dir = os.path.join(self.base_dir, folder_name)
            os.makedirs(self.session_dir, exist_ok=True)
            
            self.start_sys_time = time.time()
            self.counts = {k: 0 for k in self.counts}
            self.time_history = {k: [] for k in self.time_history}
            self.point_counts.clear()
            self.slam_poses.clear()
            
            # Setup CSV files
            headers = {
                "imu": ["sys_time", "sensor_sec", "gyro_x", "gyro_y", "gyro_z", "accel_x", "accel_y", "accel_z"],
                "lidar": ["sys_time", "sensor_sec", "point_count"],
                "camera": ["sys_time", "sensor_sec", "frame_idx"],
                "slam": ["sys_time", "pose_sec", "x", "y", "z", "qx", "qy", "qz", "qw", "roll", "pitch", "yaw", "vx", "vy", "vz"],
                "slam_diagnostics": ["sys_time", "frame_idx", "lidar_header_time", "lidar_recv_time", "slam_publish_time", "publish_latency_ms", "proc_time_ms", "points_raw_count", "points_filtered_count", "features_matched", "vio_count", "lio_residual_m", "estimated_velocity_m_s", "estimated_accel_m_s2", "delta_translation_m", "tracking_status"],
                "tf": ["sys_time", "parent", "child", "tx", "ty", "tz", "qx", "qy", "qz", "qw"],
                "gui": ["sys_time", "pose_sec", "x", "y", "yaw", "map_points", "total_dist"],
                "system": ["sys_time", "cpu_pct", "ram_pct", "nvme_free_gb", "nvme_used_pct"]
            }
            
            for key, header in headers.items():
                filepath = os.path.join(self.session_dir, f"{key}.csv")
                f = open(filepath, "w", newline="", encoding="utf-8")
                writer = csv.writer(f)
                writer.writerow(header)
                self.csv_files[key] = f
                self.csv_writers[key] = writer
            
            self.is_recording = True
            
        # Start ROS 2 bag recording if requested
        if record_rosbag:
            bag_dir = os.path.join(self.session_dir, "diagnostic_bag")
            topics = [
                "/imu",
                "/rslidar_points",
                "/image_raw",
                "/aft_mapped_to_init",
                "/LIVO2/imu_propagate",
                "/tf",
                "/tf_static"
            ]
            cmd = f"source /opt/ros/humble/setup.bash && ros2 bag record -o {bag_dir} " + " ".join(topics)
            try:
                self.bag_process = subprocess.Popen(["bash", "-c", cmd], preexec_fn=os.setsid)
            except Exception as e:
                print(f"[DIAGNOSTIC] Failed to start rosbag process: {e}")
                self.bag_process = None

        return True, self.session_dir

    def log_imu(self, sensor_sec, gx, gy, gz, ax, ay, az):
        if not self.is_recording:
            return
        now = time.time()
        with self._lock:
            if "imu" in self.csv_writers:
                self.csv_writers["imu"].writerow([f"{now:.6f}", f"{sensor_sec:.6f}", gx, gy, gz, ax, ay, az])
                self.counts["imu"] += 1
                self.time_history["imu"].append(now)

    def log_lidar(self, sensor_sec, point_count):
        if not self.is_recording:
            return
        now = time.time()
        with self._lock:
            if "lidar" in self.csv_writers:
                self.csv_writers["lidar"].writerow([f"{now:.6f}", f"{sensor_sec:.6f}", point_count])
                self.counts["lidar"] += 1
                self.time_history["lidar"].append(now)
                self.point_counts.append(point_count)

    def log_camera(self, sensor_sec, frame_idx=0):
        if not self.is_recording:
            return
        now = time.time()
        with self._lock:
            if "camera" in self.csv_writers:
                self.csv_writers["camera"].writerow([f"{now:.6f}", f"{sensor_sec:.6f}", frame_idx])
                self.counts["camera"] += 1
                self.time_history["camera"].append(now)

    def log_slam(self, pose_sec, x, y, z, qx, qy, qz, qw, roll, pitch, yaw, vx=0.0, vy=0.0, vz=0.0):
        if not self.is_recording:
            return
        now = time.time()
        with self._lock:
            if "slam" in self.csv_writers:
                self.csv_writers["slam"].writerow([
                    f"{now:.6f}", f"{pose_sec:.6f}",
                    f"{x:.4f}", f"{y:.4f}", f"{z:.4f}",
                    f"{qx:.5f}", f"{qy:.5f}", f"{qz:.5f}", f"{qw:.5f}",
                    f"{roll:.4f}", f"{pitch:.4f}", f"{yaw:.4f}",
                    f"{vx:.4f}", f"{vy:.4f}", f"{vz:.4f}"
                ])
                self.counts["slam"] += 1
                self.time_history["slam"].append(now)

    def log_slam_diagnostics(self, frame_idx, h_time, r_time, pub_time, pub_lat_ms, proc_ms, raw_pts, filt_pts, matched_pts, vio_cnt, res_m, vel, accel, delta_trans, status):
        if not self.is_recording:
            return
        now = time.time()
        with self._lock:
            if "slam_diagnostics" in self.csv_writers:
                self.csv_writers["slam_diagnostics"].writerow([
                    f"{now:.6f}", frame_idx, f"{h_time:.6f}", f"{r_time:.6f}",
                    f"{pub_time:.6f}", f"{pub_lat_ms:.2f}", f"{proc_ms:.2f}",
                    raw_pts, filt_pts, matched_pts, vio_cnt,
                    f"{res_m:.4f}", f"{vel:.3f}", f"{accel:.3f}", f"{delta_trans:.4f}", status
                ])
                self.slam_poses.append((now, x, y, z, yaw))

    def log_tf(self, parent, child, tx, ty, tz, qx, qy, qz, qw):
        if not self.is_recording:
            return
        now = time.time()
        with self._lock:
            if "tf" in self.csv_writers:
                self.csv_writers["tf"].writerow([
                    f"{now:.6f}", parent, child,
                    f"{tx:.4f}", f"{ty:.4f}", f"{tz:.4f}",
                    f"{qx:.5f}", f"{qy:.5f}", f"{qz:.5f}", f"{qw:.5f}"
                ])
                self.counts["tf"] += 1

    def log_gui(self, pose_sec, x, y, yaw, map_points, total_dist):
        if not self.is_recording:
            return
        now = time.time()
        with self._lock:
            if "gui" in self.csv_writers:
                self.csv_writers["gui"].writerow([
                    f"{now:.6f}", f"{pose_sec:.6f}",
                    f"{x:.4f}", f"{y:.4f}", f"{yaw:.4f}",
                    map_points, f"{total_dist:.2f}"
                ])
                self.counts["gui"] += 1

    def log_system(self, nvme_free_gb=0.0, nvme_used_pct=0.0):
        if not self.is_recording:
            return
        now = time.time()
        cpu_pct = psutil.cpu_percent() if psutil else 0.0
        ram_pct = psutil.virtual_memory().percent if psutil else 0.0
        with self._lock:
            if "system" in self.csv_writers:
                self.csv_writers["system"].writerow([
                    f"{now:.6f}", f"{cpu_pct:.1f}", f"{ram_pct:.1f}",
                    f"{nvme_free_gb:.2f}", f"{nvme_used_pct:.1f}"
                ])
                self.counts["system"] += 1

    def stop_session(self):
        with self._lock:
            if not self.is_recording:
                return None
            
            self.is_recording = False
            
            # Close all CSV files
            for key, f in self.csv_files.items():
                try:
                    f.flush()
                    f.close()
                except Exception:
                    pass
            self.csv_files.clear()
            self.csv_writers.clear()

        # Terminate rosbag recording process safely
        if self.bag_process:
            try:
                import signal
                os.killpg(os.getpgid(self.bag_process.pid), signal.SIGINT)
                time.sleep(1.0)
            except Exception:
                pass
            self.bag_process = None

        # Auto-generate summary report
        report_path = self.generate_summary_report()
        return report_path

    def generate_summary_report(self):
        if not self.session_dir or not os.path.exists(self.session_dir):
            return None
        
        report_path = os.path.join(self.session_dir, "summary.txt")
        dur = max(time.time() - self.start_sys_time, 0.1)
        
        lines = []
        lines.append("================================================================================")
        lines.append(f"scanR DIAGNOSTIC CAPTURE SUMMARY REPORT — {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        lines.append("================================================================================")
        lines.append(f"Session Folder: {self.session_dir}")
        lines.append(f"Duration:       {dur:.2f} seconds")
        lines.append("")
        
        lines.append("--- SENSOR STREAM RATES & TOTAL MESSAGES ---")
        for sensor, count in self.counts.items():
            hz = count / dur if dur > 0 else 0.0
            lines.append(f"  * {sensor.upper():<8} Total Msgs: {count:<6} | Average Rate: {hz:.1f} Hz")
        lines.append("")
        
        lines.append("--- TIMING JITTER & INTER-FRAME DELAYS ---")
        for sensor in ["imu", "lidar", "camera", "slam"]:
            times = self.time_history.get(sensor, [])
            if len(times) > 1:
                deltas = [times[i] - times[i-1] for i in range(1, len(times))]
                mean_d = sum(deltas) / len(deltas)
                min_d = min(deltas)
                max_d = max(deltas)
                spikes = [d for d in deltas if d > 2.5 * mean_d]
                lines.append(f"  * {sensor.upper():<8} Mean Delta: {mean_d*1000.0:.1f} ms | Min: {min_d*1000.0:.1f} ms | Max: {max_d*1000.0:.1f} ms | Spikes (>2.5x): {len(spikes)}")
            else:
                lines.append(f"  * {sensor.upper():<8} Insufficient samples for jitter analysis.")
        lines.append("")

        lines.append("--- POINT CLOUD & DENSITY METRICS ---")
        if self.point_counts:
            avg_pts = sum(self.point_counts) / len(self.point_counts)
            min_pts = min(self.point_counts)
            max_pts = max(self.point_counts)
            empty_frames = sum(1 for p in self.point_counts if p == 0)
            lines.append(f"  * Total LiDAR Scans:  {len(self.point_counts)}")
            lines.append(f"  * Mean Points/Scan:   {avg_pts:.0f}")
            lines.append(f"  * Min Points/Scan:    {min_pts}")
            lines.append(f"  * Max Points/Scan:    {max_pts}")
            lines.append(f"  * Empty/Zero Scans:   {empty_frames}")
        else:
            lines.append("  * No LiDAR point cloud data recorded.")
        lines.append("")

        lines.append("--- FAST-LIVO2 POSE & MOTION SMOOTHNESS ---")
        if len(self.slam_poses) > 1:
            total_dist = 0.0
            max_vel = 0.0
            for i in range(1, len(self.slam_poses)):
                t1, x1, y1, z1, _ = self.slam_poses[i-1]
                t2, x2, y2, z2, _ = self.slam_poses[i]
                dt = t2 - t1
                if dt > 0:
                    d = math.sqrt((x2-x1)**2 + (y2-y1)**2 + (z2-z1)**2)
                    total_dist += d
                    v = d / dt
                    if v > max_vel:
                        max_vel = v
            p_start = self.slam_poses[0]
            p_end = self.slam_poses[-1]
            net_disp = math.sqrt((p_end[1]-p_start[1])**2 + (p_end[2]-p_start[2])**2 + (p_end[3]-p_start[3])**2)
            lines.append(f"  * Total Pose Distance Walked: {total_dist:.2f} m")
            lines.append(f"  * Net Displacement:           {net_disp:.2f} m")
            lines.append(f"  * Peak Pose Velocity:         {max_vel:.2f} m/s")
        else:
            lines.append("  * Insufficient SLAM poses for motion analysis.")
        lines.append("")
        lines.append("================================================================================")
        
        report_content = "\n".join(lines)
        with open(report_path, "w", encoding="utf-8") as rf:
            rf.write(report_content)
        
        return report_path
