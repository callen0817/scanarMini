#!/usr/bin/env python3
"""
Comprehensive 60-Second Hardware Sync & Depth Frame Loss Diagnostic
Evaluates:
A. Expected Airy Trigger Pulses (30.00 Hz * duration)
B. Gemini RGB Stream Frame Count & Interval Distribution
C. Gemini Depth Stream Frame Count & Interval Distribution
D. Frame-by-frame header delta distribution (detecting 33.3ms, 66.6ms, 100ms trigger multiples)
E. Hardware delivery % and dropped-frame telemetry
"""

import os
import sys
import time
import json
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, Imu

class DepthTriggerDiagnostics(Node):
    def __init__(self):
        super().__init__('depth_trigger_diagnostics')
        self.depth_frames = [] # (arrival_t, header_t)
        self.rgb_frames = []   # (arrival_t, header_t)
        self.imu_msgs = []     # (arrival_t, header_t)
        
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=300)
        self.sub_depth = self.create_subscription(Image, '/camera/depth/image_raw', self.depth_cb, qos)
        self.sub_rgb = self.create_subscription(Image, '/camera/color/image_raw', self.rgb_cb, qos)
        self.sub_imu = self.create_subscription(Imu, '/rslidar_imu_data', self.imu_cb, qos)

    def depth_cb(self, msg: Image):
        self.depth_frames.append((time.perf_counter(), msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9))

    def rgb_cb(self, msg: Image):
        self.rgb_frames.append((time.perf_counter(), msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9))

    def imu_cb(self, msg: Imu):
        self.imu_msgs.append((time.perf_counter(), msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9))

def analyze_stream(frames, nominal_hz, expected_count):
    if len(frames) < 2:
        return {}
    arr_t = np.array([f[0] for f in frames])
    hdr_t = np.array([f[1] for f in frames])
    
    dur_arr = arr_t[-1] - arr_t[0]
    dur_hdr = hdr_t[-1] - hdr_t[0]
    
    dt_arr_ms = np.diff(arr_t) * 1000.0
    dt_hdr_ms = np.diff(hdr_t) * 1000.0
    
    # Analyze trigger interval multiples (base ~33.33 ms)
    # 1x trigger: 25ms - 45ms
    # 2x trigger (1 dropped pulse): 55ms - 75ms
    # 3x trigger (2 dropped pulses): 85ms - 110ms
    # >3x trigger: >110ms
    single_trig = int(np.sum((dt_hdr_ms >= 25.0) & (dt_hdr_ms <= 45.0)))
    double_trig = int(np.sum((dt_hdr_ms > 45.0) & (dt_hdr_ms <= 75.0)))
    triple_trig = int(np.sum((dt_hdr_ms > 75.0) & (dt_hdr_ms <= 110.0)))
    multi_trig = int(np.sum(dt_hdr_ms > 110.0))
    
    # Calculate estimated dropped pulses
    # Each double_trig implies 1 skipped pulse
    # Each triple_trig implies 2 skipped pulses
    # Each multi_trig implies round(dt / 33.33) - 1 skipped pulses
    est_dropped_frames = double_trig * 1 + triple_trig * 2 + sum([max(0, int(round(dt / 33.33)) - 1) for dt in dt_hdr_ms[dt_hdr_ms > 110.0]])

    return {
        "count": len(frames),
        "expected_count": expected_count,
        "delivery_pct": round(len(frames) / expected_count * 100.0, 2) if expected_count > 0 else 0.0,
        "average_hz": round(len(frames) / dur_hdr, 2) if dur_hdr > 0 else 0.0,
        "header_dt_distribution": {
            "p1_ms": round(float(np.percentile(dt_hdr_ms, 1)), 2),
            "p5_ms": round(float(np.percentile(dt_hdr_ms, 5)), 2),
            "p25_ms": round(float(np.percentile(dt_hdr_ms, 25)), 2),
            "p50_ms": round(float(np.percentile(dt_hdr_ms, 50)), 2),
            "p75_ms": round(float(np.percentile(dt_hdr_ms, 75)), 2),
            "p95_ms": round(float(np.percentile(dt_hdr_ms, 95)), 2),
            "p99_ms": round(float(np.percentile(dt_hdr_ms, 99)), 2),
            "min_ms": round(float(np.min(dt_hdr_ms)), 2),
            "max_ms": round(float(np.max(dt_hdr_ms)), 2)
        },
        "trigger_cadence_breakdown": {
            "1x_trigger_33ms_delivered": single_trig,
            "2x_trigger_66ms_1_dropped": double_trig,
            "3x_trigger_100ms_2_dropped": triple_trig,
            "multi_skip_over_110ms": multi_trig,
            "total_estimated_dropped_triggers": est_dropped_frames
        }
    }

def run_depth_diagnostic(duration=60.0):
    rclpy.init()
    node = DepthTriggerDiagnostics()
    print(f"\n[DEPTH DIAGNOSTIC] Monitoring Hardware Sync & Depth Streams for {duration:.1f}s...")
    t0 = time.time()
    while (time.time() - t0) < duration:
        rclpy.spin_once(node, timeout_sec=0.01)

    node.destroy_node()
    rclpy.shutdown()

    expected_triggers = int(round(30.00 * duration)) # 30 Hz Airy pulses
    depth_report = analyze_stream(node.depth_frames, 30.0, expected_triggers)
    rgb_report = analyze_stream(node.rgb_frames, 30.0, expected_triggers)

    full_report = {
        "duration_seconds": duration,
        "airy_hardware_trigger": {
            "motor_rpm": 600,
            "physical_rotations_per_sec": 10.0,
            "pulses_per_revolution": 3,
            "pulse_spacing_deg": [0, 120, 240],
            "expected_trigger_rate_hz": 30.00,
            "expected_total_pulses": expected_triggers
        },
        "gemini_depth_stream": depth_report,
        "gemini_rgb_stream": rgb_report,
        "airy_imu_count": len(node.imu_msgs),
        "airy_imu_hz": round(len(node.imu_msgs) / duration, 2)
    }

    print("\n" + "="*85)
    print("         60-SECOND HARDWARE TRIGGER & GEMINI DEPTH DIAGNOSTIC REPORT         ")
    print("="*85)
    print(json.dumps(full_report, indent=2))
    print("="*85)

    with open("/home/scanar/scanarMini/data/depth_trigger_report.json", 'w') as f:
        json.dump(full_report, f, indent=2)

    return full_report

if __name__ == '__main__':
    dur = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
    run_depth_diagnostic(dur)
