#!/usr/bin/env python3
"""
Runs a 4.0-second trial of M127InteractiveTestHarness to verify telemetry ingestion.
"""

import os
import sys
import time
import json
import rclpy
from m12_7_interactive_test_harness import M127InteractiveTestHarness
import threading

def run_trial():
    rclpy.init()
    node = M127InteractiveTestHarness(dataset_name="M12_7_Preflight_Trial")
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    time.sleep(1.0)
    print("[TRIAL] Entering TRIAL_PHASE...")
    node.set_phase("TRIAL_PHASE")
    time.sleep(3.0)

    print("[TRIAL] Stopping and exporting...")
    meta = node.stop_and_export()
    rclpy.shutdown()
    spin_thread.join(timeout=2.0)

    print("\n============================================================")
    print("             TRIAL HARNESS INGESTION TELEMETRY              ")
    print("============================================================")
    print(json.dumps(meta["telemetry_counters"], indent=2))
    print(json.dumps(meta["point_counts"], indent=2))
    print("============================================================")

if __name__ == '__main__':
    run_trial()
