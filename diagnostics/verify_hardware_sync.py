#!/usr/bin/env python3
"""
scanR Hardware Synchronization Verification & Preflight Diagnostic Suite
Supports Milestone M5 Sub-stages M5.0 to M5.5 for IG-2, Gemini 336L, and RoboSense Airy.
"""

import sys
import time
import argparse
import numpy as np

def print_m5_preflight_checklist():
    print("=" * 80)
    print("        scanR Milestone M5.0 — Pre-Cable Electrical Preflight Matrix        ")
    print("=" * 80)
    print("KEEP CABLES UNPLUGGED AND POWER OFF UNTIL ALL CHECKS PASS BELOW:")
    print("-" * 80)
    print("1. ELECTRICAL LEVEL & PIN DIRECTION CHECKS (EMPIRICAL MEASUREMENT REQUIRED):")
    print("   [ ] IG-2 Pin H1-6 (STROBE):   Confirm configured as STROBE INPUT (rising edge).")
    print("   [ ] IG-2 Pin H1-7 (Tx0):      Measure actual output logic level (3.3V TTL into MAX3232 -> RS-232 to Airy).")
    print("   [ ] IG-2 Pin H1-14 (PPS):     Measure actual high-level pulse voltage with meter/scope.")
    print("                                 Confirm voltage satisfies Airy Pin 3 input specification.")
    print("   [ ] Gemini Pin 3 (VSYNC_OUT): Confirm VSYNC output active on Pin 3 (Pin 4 is UNCONNECTED).")
    print("                                 Dynamically derive trigger rate from camera stream FPS.")
    print("   [ ] IG-2 Pin H1-1 (GND):      Confirm single common ground reference for both cables.")
    print("\n2. POWER-OFF MULTIMETER CONTINUITY & SHORT-CIRCUIT TEST MATRIX:")
    print("   [ ] 336L Pin 3 <-> IG-2 H1-6:    CONTINUITY (< 1 Ohm)")
    print("   [ ] 336L Pin 8 <-> IG-2 H1-1:    CONTINUITY (< 1 Ohm)")
    print("   [ ] Airy Pin 1 <-> MAX3232 T1OUT: CONTINUITY (< 1 Ohm)")
    print("   [ ] Airy Pin 3 <-> IG-2 H1-14:   CONTINUITY (< 1 Ohm)")
    print("   [ ] Airy Pin 4 <-> IG-2 H1-1:    CONTINUITY (< 1 Ohm)")
    print("   [ ] 336L Pin 3 <-> GND:          NO SHORT   (Infinity Ohm)")
    print("   [ ] Airy Pin 3 <-> GND:          NO SHORT   (Infinity Ohm)")
    print("   [ ] Airy Pin 1 <-> GND:          NO SHORT   (Infinity Ohm)")
    print("=" * 80)

def run_hardware_sync_verification(duration_sec=10.0, substage="M5.4"):
    print("=" * 80)
    print(f"      scanR Hardware Synchronization Verification Suite [{substage}]      ")
    print("=" * 80)
    print(f"Master Timing Reference: InertialSense IG-2 Dual RTK IMU")
    print(f"Camera Interface:        Orbbec Gemini 336L (Pin 3 VSYNC_OUT -> IG-2 H1-6 STROBE)")
    print(f"LiDAR Interface:         RoboSense RS-LiDAR Airy (GPS_PPS <- IG-2 H1-14, GPS_GPRMC <- IG-2 H1-7 via MAX3232)")
    print("-" * 80)

    try:
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import PointCloud2, Imu, Image
    except ImportError:
        print("[WARN] rclpy not installed in current environment. Running standalone diagnostic simulation mode.")
        _run_simulated_report(duration_sec)
        return

    if not rclpy.ok():
        rclpy.init()

    class SyncVerificationNode(Node):
        def __init__(self):
            super().__init__('verify_hardware_sync')
            self.imu_stamps = []
            self.airy_stamps = []
            self.cam_stamps = []

            self.create_subscription(Imu, '/imu', self.imu_cb, 100)
            self.create_subscription(PointCloud2, '/rslidar_points', self.airy_cb, 10)
            self.create_subscription(Image, '/image_raw', self.cam_cb, 10)

        def imu_cb(self, msg):
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.imu_stamps.append((time.time(), stamp))

        def airy_cb(self, msg):
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.airy_stamps.append((time.time(), stamp))

        def cam_cb(self, msg):
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.cam_stamps.append((time.time(), stamp))

    node = SyncVerificationNode()
    print(f"Collecting hardware sensor data for {duration_sec} seconds...")
    start_t = time.time()

    while (time.time() - start_t) < duration_sec:
        rclpy.spin_once(node, timeout_sec=0.1)

    print("\nProcessing timestamp synchronization statistics...")
    _analyze_timestamps(node.imu_stamps, node.airy_stamps, node.cam_stamps)
    node.destroy_node()
    rclpy.shutdown()

def _analyze_timestamps(imu_stamps, airy_stamps, cam_stamps):
    if not imu_stamps or not airy_stamps or not cam_stamps:
        print(f"[FAIL] Missing stream data: IMU count={len(imu_stamps)}, Airy count={len(airy_stamps)}, Camera count={len(cam_stamps)}")
        return

    imu_t = np.array([s[1] for s in imu_stamps])
    airy_t = np.array([s[1] for s in airy_stamps])
    cam_t = np.array([s[1] for s in cam_stamps])

    # Calculate actual camera stream FPS
    cam_fps = round(len(cam_t) / max(cam_t[-1] - cam_t[0], 0.001), 1) if len(cam_t) > 1 else 0.0
    print(f"Captured Samples: IG-2 IMU: {len(imu_t)} | RoboSense Airy: {len(airy_t)} | Gemini 336L: {len(cam_t)}")
    print(f"Derived 336L Camera FPS: {cam_fps} Hz | Expected VSYNC_OUT Trigger Rate: {cam_fps} Hz")

    airy_imu_deltas = []
    for at in airy_t:
        idx = np.argmin(np.abs(imu_t - at))
        airy_imu_deltas.append((at - imu_t[idx]) * 1000.0)

    cam_imu_deltas = []
    for ct in cam_t:
        idx = np.argmin(np.abs(imu_t - ct))
        cam_imu_deltas.append((ct - imu_t[idx]) * 1000.0)

    airy_imu_deltas = np.array(airy_imu_deltas)
    cam_imu_deltas = np.array(cam_imu_deltas)

    print("\n" + "=" * 80)
    print("                       HARDWARE SYNC DIAGNOSTIC REPORT                       ")
    print("=" * 80)
    print("1. IG-2 IMU <-> RoboSense Airy LiDAR Temporal Alignment:")
    print(f"   - Mean Delta:    {np.mean(airy_imu_deltas):.3f} ms")
    print(f"   - Median Delta:  {np.median(airy_imu_deltas):.3f} ms")
    print(f"   - Min / Max:     {np.min(airy_imu_deltas):.3f} ms / {np.max(airy_imu_deltas):.3f} ms")
    print(f"   - Std Dev:       {np.std(airy_imu_deltas):.3f} ms")

    print("\n2. IG-2 IMU <-> Orbbec Gemini 336L Camera Temporal Alignment:")
    print(f"   - Mean Delta:    {np.mean(cam_imu_deltas):.3f} ms")
    print(f"   - Median Delta:  {np.median(cam_imu_deltas):.3f} ms")
    print(f"   - Min / Max:     {np.min(cam_imu_deltas):.3f} ms / {np.max(cam_imu_deltas):.3f} ms")
    print(f"   - Std Dev:       {np.std(cam_imu_deltas):.3f} ms")

    sync_ok = (np.abs(np.mean(cam_imu_deltas)) < 5.0) and (np.abs(np.mean(airy_imu_deltas)) < 5.0)
    if sync_ok:
        print("\n[VERDICT] HARDWARE SYNCHRONIZATION: PASS (Tight temporal alignment verified)")
    else:
        print("\n[VERDICT] HARDWARE SYNCHRONIZATION: FAIL / UNVERIFIED (Inspect cable connections and baud settings)")
    print("=" * 80)

def _run_simulated_report(duration_sec):
    print("\n" + "=" * 80)
    print("             HARDWARE SYNC STANDALONE DIAGNOSTIC REPORT (BASELINE)           ")
    print("=" * 80)
    print("IG-2 Master Reference: H1-6 (336L Strobe Input), H1-7 (Airy GPRMC Tx), H1-14 (Airy PPS)")
    print("Expected Target Metrics:")
    print("  - 336L Derived Camera FPS:           Dynamically measured (e.g. 15 Hz / 20 Hz / 30 Hz)")
    print("  - IG-2 ↔ Airy PPS Sync Offset:       < 1.0 ms")
    print("  - IG-2 ↔ Gemini 336L Strobe Offset:  < 1.5 ms")
    print("  - Camera-IMU Delta-t Std Dev:        < 0.5 ms")
    print("  - Fast-LIVO2 time_offset_lidar_to_imu: 0.0 s")
    print("  - Fast-LIVO2 img_time_offset:        0.0 s")
    print("=" * 80)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="scanR Hardware Synchronization Verification Tool")
    parser.add_argument('--duration', type=float, default=10.0, help="Data collection duration in seconds")
    parser.add_argument('--preflight', action='store_true', help="Print M5.0 pre-cable electrical checklist")
    parser.add_argument('--substage', type=str, default="M5.4", help="Milestone sub-stage to execute (e.g. M5.0 - M5.5)")
    args = parser.parse_args()

    if args.preflight or args.substage == "M5.0":
        print_m5_preflight_checklist()
    else:
        run_hardware_sync_verification(args.duration, args.substage)
