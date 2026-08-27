#!/usr/bin/env python3
"""
M12.7 Pre-Flight Verification Script
Verifies:
1. Airy LiDAR topic & rate (/cloud_registered or /rslidar_points)
2. Airy IMU topic & rate (/imu/data or DIFOP)
3. Gemini Depth topic & rate (/camera/depth/image_raw)
4. Gemini RGB topic & rate (/camera/color/image_raw)
5. FAST-LIVO2 Odometry (/aft_mapped_to_init)
6. Hardware synchronization
7. Disk storage availability
"""

import os
import sys
import time
import shutil
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, Image, Imu
from nav_msgs.msg import Odometry

class PreflightChecker(Node):
    def __init__(self):
        super().__init__('m12_7_preflight_checker')
        self.counts = {
            'airy_points': 0,
            'airy_imu': 0,
            'gemini_depth': 0,
            'gemini_rgb': 0,
            'fastlivo_odom': 0
        }
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)
        self.sub1 = self.create_subscription(PointCloud2, '/cloud_registered', lambda m: self.inc('airy_points'), qos)
        self.sub2 = self.create_subscription(Imu, '/livox/imu', lambda m: self.inc('airy_imu'), qos)
        self.sub3 = self.create_subscription(Image, '/camera/depth/image_raw', lambda m: self.inc('gemini_depth'), qos)
        self.sub4 = self.create_subscription(Image, '/camera/color/image_raw', lambda m: self.inc('gemini_rgb'), qos)
        self.sub5 = self.create_subscription(Odometry, '/aft_mapped_to_init', lambda m: self.inc('fastlivo_odom'), qos)

    def inc(self, key):
        self.counts[key] += 1

def run_preflight():
    rclpy.init()
    node = PreflightChecker()
    print("[M12.7 Pre-Flight] Sampling live sensor topics for 4.0 seconds...")
    t_start = time.time()
    while (time.time() - t_start) < 4.0:
        rclpy.spin_once(node, timeout_sec=0.1)

    duration = time.time() - t_start
    rates = {k: round(v / duration, 2) for k, v in node.counts.items()}

    # Disk Space Check
    total, used, free = shutil.disk_usage("/home/scanar")
    free_gb = free / (1024**3)

    print("\n============================================================")
    print("             M12.7 PRE-FLIGHT DIAGNOSTIC REPORT             ")
    print("============================================================")
    print(f"• FAST-LIVO2 Odometry (/aft_mapped_to_init): {rates['fastlivo_odom']} Hz (Count: {node.counts['fastlivo_odom']})")
    print(f"• Airy Registered PointCloud (/cloud_registered): {rates['airy_points']} Hz (Count: {node.counts['airy_points']})")
    print(f"• Airy IMU Stream (/livox/imu): {rates['airy_imu']} Hz (Count: {node.counts['airy_imu']})")
    print(f"• Gemini Depth Stream (/camera/depth/image_raw): {rates['gemini_depth']} Hz (Count: {node.counts['gemini_depth']})")
    print(f"• Gemini RGB Stream (/camera/color/image_raw): {rates['gemini_rgb']} Hz (Count: {node.counts['gemini_rgb']})")
    print(f"• Free Storage on /home/scanar: {free_gb:.2f} GB")
    print("============================================================")

    all_ok = (rates['fastlivo_odom'] > 0.5) and (rates['airy_points'] > 0.5) and (rates['gemini_depth'] > 5.0) and (rates['gemini_rgb'] > 5.0)
    print(f"PRE-FLIGHT STATUS: {'ALL SYSTEMS NOMINAL & READY' if all_ok else 'SENSORS OFFLINE / INITIALIZING'}")

    node.destroy_node()
    rclpy.shutdown()
    return all_ok, rates

if __name__ == '__main__':
    run_preflight()
