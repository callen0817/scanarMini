#!/usr/bin/env python3
"""
scanR Ground-Truth ROS Bag Diagnostic Analyzer (Milestone 4.5)
Queries SQLite-based ROS 2 bag files directly to extract ground-truth telemetry:
- /rslidar_points header vs receive timestamps, inter-frame deltas, payload sizes.
- /imu frequency, accelerometer & gyro magnitudes, timestamp jitter.
- /aft_mapped_to_init pose trajectory, velocity, exact divergence timestamp.
- /tf & /tf_static frame transforms.
"""

import os
import sys
import sqlite3
import math
import numpy as np

try:
    import rclpy
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import PointCloud2, Imu, Image
    from nav_msgs.msg import Odometry
    from tf2_msgs.msg import TFMessage
except ImportError:
    print("[ERROR] rclpy serialization tools not found.")
    sys.exit(1)


def analyze_bag_truth(bag_folder):
    db_file = None
    for f in os.listdir(bag_folder):
        if f.endswith(".db3"):
            db_file = os.path.join(bag_folder, f)
            break
            
    if not db_file or not os.path.exists(db_file):
        print(f"[ERROR] No .db3 bag file found in '{bag_folder}'.")
        return False

    print("================================================================================")
    print(f"scanR ROS BAG GROUND-TRUTH ANALYSIS REPORT")
    print(f"Bag File: {db_file}")
    print("================================================================================")

    conn = sqlite3.connect(db_file)
    c = conn.cursor()

    c.execute("SELECT id, name, type FROM topics")
    topics = {row[0]: (row[1], row[2]) for row in c.fetchall()}

    topic_ids = {name: tid for tid, (name, ttype) in topics.items()}

    # 1. ANALYZE /imu (Topic ID for /imu)
    imu_tid = topic_ids.get("/imu")
    if imu_tid:
        c.execute("SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp ASC", (imu_tid,))
        imu_rows = c.fetchall()
        
        imu_recv_t = []
        imu_hdr_t = []
        accel_mags = []
        gyro_mags = []

        for r_ns, blob in imu_rows:
            msg = deserialize_message(blob, Imu)
            recv_sec = r_ns * 1e-9
            hdr_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            imu_recv_t.append(recv_sec)
            imu_hdr_t.append(hdr_sec)
            
            ax, ay, az = msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z
            gx, gy, gz = msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z
            
            accel_mags.append(math.sqrt(ax*ax + ay*ay + az*az))
            gyro_mags.append(math.sqrt(gx*gx + gy*gy + gz*gz))

        if imu_hdr_t:
            dur = imu_hdr_t[-1] - imu_hdr_t[0]
            hz = len(imu_hdr_t) / max(dur, 0.001)
            hdr_deltas = [imu_hdr_t[i] - imu_hdr_t[i-1] for i in range(1, len(imu_hdr_t))]
            recv_deltas = [imu_recv_t[i] - imu_recv_t[i-1] for i in range(1, len(imu_recv_t))]
            
            print(f"\n[IMU TOPIC TRUTH (/imu)]")
            print(f"  * Total Messages:     {len(imu_hdr_t)}")
            print(f"  * Mean Publish Rate:  {hz:.1f} Hz")
            print(f"  * Mean Header Delta:  {np.mean(hdr_deltas)*1000:.2f} ms")
            print(f"  * Max Header Delta:   {np.max(hdr_deltas)*1000:.2f} ms")
            print(f"  * Max Receive Delta:  {np.max(recv_deltas)*1000:.2f} ms")
            print(f"  * Header Gaps >20ms:  {sum(1 for d in hdr_deltas if d > 0.020)}")
            print(f"  * Recv Gaps >20ms:    {sum(1 for d in recv_deltas if d > 0.020)}")
            print(f"  * Mean Accel Mag:     {np.mean(accel_mags):.2f} m/s^2 (Std: {np.std(accel_mags):.2f})")
            print(f"  * Mean Gyro Mag:      {np.mean(gyro_mags):.4f} rad/s")

    # 2. ANALYZE /rslidar_points
    lidar_tid = topic_ids.get("/rslidar_points")
    if lidar_tid:
        c.execute("SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp ASC", (lidar_tid,))
        lidar_rows = c.fetchall()

        lidar_recv_t = []
        lidar_hdr_t = []
        point_counts = []

        for r_ns, blob in lidar_rows:
            msg = deserialize_message(blob, PointCloud2)
            recv_sec = r_ns * 1e-9
            hdr_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            lidar_recv_t.append(recv_sec)
            lidar_hdr_t.append(hdr_sec)
            point_counts.append(msg.width * msg.height)

        if lidar_hdr_t:
            dur = lidar_hdr_t[-1] - lidar_hdr_t[0]
            hz = len(lidar_hdr_t) / max(dur, 0.001)
            hdr_deltas = [lidar_hdr_t[i] - lidar_hdr_t[i-1] for i in range(1, len(lidar_hdr_t))]
            recv_deltas = [lidar_recv_t[i] - lidar_recv_t[i-1] for i in range(1, len(lidar_recv_t))]

            print(f"\n[LiDAR TOPIC TRUTH (/rslidar_points)]")
            print(f"  * Total Messages:     {len(lidar_hdr_t)}")
            print(f"  * Mean Publish Rate:  {hz:.1f} Hz")
            print(f"  * Mean Points/Scan:   {np.mean(point_counts):.0f} (Min: {np.min(point_counts)}, Max: {np.max(point_counts)})")
            print(f"  * Mean Header Delta:  {np.mean(hdr_deltas)*1000:.2f} ms")
            print(f"  * Max Header Delta:   {np.max(hdr_deltas)*1000:.2f} ms")
            print(f"  * Max Receive Delta:  {np.max(recv_deltas)*1000:.2f} ms")
            print(f"  * Header Gaps >250ms: {sum(1 for d in hdr_deltas if d > 0.250)}")
            print(f"  * Recv Gaps >250ms:   {sum(1 for d in recv_deltas if d > 0.250)}")

    # 3. ANALYZE /aft_mapped_to_init (SLAM POSE)
    slam_tid = topic_ids.get("/aft_mapped_to_init")
    if slam_tid:
        c.execute("SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp ASC", (slam_tid,))
        slam_rows = c.fetchall()

        slam_recv_t = []
        slam_hdr_t = []
        poses = [] # (x, y, z, qx, qy, qz, qw)

        for r_ns, blob in slam_rows:
            msg = deserialize_message(blob, Odometry)
            recv_sec = r_ns * 1e-9
            hdr_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            slam_recv_t.append(recv_sec)
            slam_hdr_t.append(hdr_sec)
            
            p = msg.pose.pose.position
            o = msg.pose.pose.orientation
            poses.append((p.x, p.y, p.z, o.x, o.y, o.z, o.w))

        if slam_hdr_t:
            dur = slam_hdr_t[-1] - slam_hdr_t[0]
            hz = len(slam_hdr_t) / max(dur, 0.001)
            
            # Find divergence timestamp
            divergence_idx = None
            for i, p in enumerate(poses):
                dist = math.hypot(p[0], p[1])
                if dist > 20.0:
                    divergence_idx = i
                    break

            print(f"\n[SLAM POSE TRUTH (/aft_mapped_to_init)]")
            print(f"  * Total Pose Updates: {len(slam_hdr_t)}")
            print(f"  * Mean SLAM Rate:     {hz:.1f} Hz")
            
            if divergence_idx is not None:
                div_t = slam_hdr_t[divergence_idx]
                rel_t = div_t - slam_hdr_t[0]
                p_div = poses[divergence_idx]
                print(f"  * Divergence Timestamp: {div_t:.6f} (+{rel_t:.2f}s into run)")
                print(f"  * Pose at Divergence:  X={p_div[0]:.2f}m Y={p_div[1]:.2f}m Z={p_div[2]:.2f}m")
                
                # Check pose 5 seconds BEFORE divergence
                pre_idx = max(0, divergence_idx - 100)
                p_pre = poses[pre_idx]
                pre_t = slam_hdr_t[pre_idx] - slam_hdr_t[0]
                print(f"  * Stable Walk Before:  t=+{pre_t:.2f}s | X={p_pre[0]:.2f}m Y={p_pre[1]:.2f}m Z={p_pre[2]:.2f}m")
            else:
                print(f"  * Zero divergence observed! Max distance remained < 20m.")

    # 4. ANALYZE /tf & /tf_static
    tf_static_tid = topic_ids.get("/tf_static")
    if tf_static_tid:
        c.execute("SELECT timestamp, data FROM messages WHERE topic_id=?", (tf_static_tid,))
        tf_static_rows = c.fetchall()
        print(f"\n[TF STATIC TRUTH (/tf_static)]")
        for r_ns, blob in tf_static_rows:
            msg = deserialize_message(blob, TFMessage)
            for transform in msg.transforms:
                parent = transform.header.frame_id
                child = transform.child_frame_id
                tr = transform.transform.translation
                rot = transform.transform.rotation
                print(f"  * Static TF: {parent} -> {child}")
                print(f"    Translation: [{tr.x:.4f}, {tr.y:.4f}, {tr.z:.4f}]")
                print(f"    Rotation (Quat): [{rot.x:.4f}, {rot.y:.4f}, {rot.z:.4f}, {rot.w:.4f}]")

    print("\n================================================================================")
    conn.close()
    return True


if __name__ == "__main__":
    if len(sys.argv) > 1:
        bag_dir = sys.argv[1]
    else:
        bag_dir = "/home/scanar/scanR/diagnostics/Scan_0804_0903_20260804_090555/diagnostic_bag"
    analyze_bag_truth(bag_dir)
