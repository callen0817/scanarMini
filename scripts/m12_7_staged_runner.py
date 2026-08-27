#!/usr/bin/env python3
"""
Milestone M12.7: Staged Human-in-the-Loop Dynamic Multi-Modal Fusion Qualification Runner
Manages individual physical stages:
- Stage 1: Stationary Baseline (8s)
- Stage 2: Dynamic Yaw (~90° Left, 8s)
- Stage 3: Forward Translation (1.5-2.5m, 8-10s)
- Stage 4: Lateral / Oblique Motion (1-2m, 8-10s)
- Stage 5: Return Loop + Final Stationary Hold (8s)
"""

import os
import sys
import time
import math
import json
import numpy as np
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, Image, CameraInfo, Imu
from nav_msgs.msg import Odometry

from m12_7_interactive_test_harness import M127InteractiveTestHarness

class M127StagedSession:
    def __init__(self, dataset_name="M12_7_Dynamic_Qualification"):
        self.dataset_name = dataset_name
        self.node = None
        self.executor = None
        self.spin_thread = None

    def start(self):
        rclpy.init()
        self.node = M127InteractiveTestHarness(dataset_name=self.dataset_name)
        self.spin_thread = threading.Thread(target=rclpy.spin, args=(self.node,), daemon=True)
        self.spin_thread.start()
        time.sleep(2.0)
        print(f"[M12.7 Session] Initialized and listening for dataset: {self.dataset_name}")

    def record_stage(self, stage_name, duration_sec):
        print(f"\n============================================================")
        print(f">>> RECORDING {stage_name} FOR {duration_sec:.1f} SECONDS <<<")
        print(f"============================================================")
        self.node.set_phase(stage_name)
        t_start = time.time()
        while time.time() - t_start < duration_sec:
            time.sleep(0.5)
            elapsed = time.time() - t_start
            print(f"  [{stage_name}] Elapsed: {elapsed:.1f}s / {duration_sec:.1f}s | Airy sweeps: {len(self.node.airy_sweeps)} | Depth frames: {len(self.node.gemini_frames)} | Odom msgs: {self.node.cnt_raw_odom_msgs}")
        print(f">>> {stage_name} COMPLETE. <<<")

    def finalize_and_export(self):
        print(f"\n============================================================")
        print(f">>> FINALIZING M12.7 SESSION & EXPORTING PRODUCTION DATASETS <<<")
        print(f"============================================================")
        metadata = self.node.stop_and_export()
        rclpy.shutdown()
        if self.spin_thread:
            self.spin_thread.join(timeout=3.0)
        return metadata

