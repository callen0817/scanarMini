#!/usr/bin/env python3
"""
scanR Milestone M7.2 — Isolated Single-Axis Rotation Verification Suite
Records and visualizes live Euler rotations (Roll, Pitch, Yaw) to verify physical-to-estimator frame alignment.
"""

import os
import sys
import time
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

def quat_to_euler(x, y, z, w):
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)

class M7RotationMonitor(Node):
    def __init__(self):
        super().__init__('m7_rotation_monitor')
        self.sub = self.create_subscription(Odometry, '/aft_mapped_to_init', self.cb, 10)
        self.r0, self.p0, self.y0 = None, None, None
        self.latest_msg = None
        
    def cb(self, msg):
        q = msg.pose.pose.orientation
        p = msg.pose.pose.position
        r, p_deg, y = quat_to_euler(q.x, q.y, q.z, q.w)
        if self.r0 is None:
            self.r0, self.p0, self.y0 = r, p_deg, y
        self.latest_msg = (p.x, p.y, p.z, r - self.r0, p_deg - self.p0, y - self.y0)

def main():
    os.environ["ROS_DOMAIN_ID"] = "2"
    rclpy.init()
    node = M7RotationMonitor()
    
    print("=" * 80)
    print("           scanR Milestone M7.2 — Live Rotation Verification Monitor           ")
    print("=" * 80)
    print("Instructions:")
    print("1. Keep rig stationary for reference baseline.")
    print("2. Perform isolated motions:")
    print("   - YAW LEFT / RIGHT          (expect ΔYaw > 0 on left, < 0 on right)")
    print("   - PITCH UP / DOWN           (expect ΔPitch change)")
    print("   - ROLL CLOCKWISE / CCW      (expect ΔRoll change)")
    print("-" * 80)
    
    start = time.time()
    last_print = 0
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            now = time.time()
            if now - last_print >= 0.2 and node.latest_msg:
                x, y, z, dr, dp, dy = node.latest_msg
                print(f"\r[t={now-start:5.1f}s] ΔRoll: {dr:+6.2f}° | ΔPitch: {dp:+6.2f}° | ΔYaw: {dy:+6.2f}° | Pos: [{x:+5.2f}, {y:+5.2f}, {z:+5.2f}] m", end='', flush=True)
                last_print = now
    except KeyboardInterrupt:
        print("\n\nMonitor stopped.")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
