#!/usr/bin/env python3
"""
M12.7 Interactive Human-in-the-Loop Qualification Orchestrator
Provides stage-by-stage command execution for the live physical test:
1. Pre-flight check & rate confirmation
2. Stage 1: Stationary Baseline (5s)
3. Stage 2: 1m Forward Translation
4. Stage 3: Rotation (~90 deg)
5. Stage 4: Return / Multi-View Traverse
6. Stage 5: Final Stationary Hold (5s)
7. Final Three-Class ASPS Fusion & Telemetry Reporting
"""

import os
import sys
import time
import json
import rclpy
import threading
from m12_7_interactive_test_harness import M127InteractiveTestHarness

class M127Orchestrator:
    def __init__(self, dataset_name="M12_7_Human_Controlled_Motion"):
        self.dataset_name = dataset_name
        self.node = None
        self.executor = None
        self.spin_thread = None

    def initialize_node(self):
        rclpy.init()
        self.node = M127InteractiveTestHarness(dataset_name=self.dataset_name)
        self.spin_thread = threading.Thread(target=rclpy.spin, args=(self.node,), daemon=True)
        self.spin_thread.start()
        # Allow buffers to pre-populate for 2 seconds
        time.sleep(2.0)
        print(f"[M12.7 Orchestrator] Ingestion node initialized and listening.")

    def run_stage_1_stationary(self, duration=5.0):
        print(f"\n>>> RECORDING STAGE 1: Stationary Baseline ({duration:.1f}s) <<<")
        self.node.set_phase("1_STATIONARY_START")
        time.sleep(duration)
        print(f">>> STAGE 1 COMPLETE: {len(self.node.airy_sweeps)} sweeps, {len(self.node.gemini_frames)} depth frames buffered. <<<")

    def start_stage_2_translation(self):
        print(f"\n>>> RECORDING STAGE 2: 1m Forward Translation <<<")
        self.node.set_phase("2_FORWARD_TRANSLATION")

    def end_stage_2_translation(self, hold_duration=3.0):
        print(f">>> STAGE 2 TRANSLATION STOPPED. Entering Post-Translation Hold ({hold_duration:.1f}s) <<<")
        self.node.set_phase("3_POST_TRANSLATION_HOLD")
        time.sleep(hold_duration)
        print(f">>> Post-Translation Hold Complete. <<<")

    def start_stage_3_rotation(self):
        print(f"\n>>> RECORDING STAGE 3: ~90 deg Rotation <<<")
        self.node.set_phase("4_ROTATION_90DEG")

    def end_stage_3_rotation(self, hold_duration=3.0):
        print(f">>> STAGE 3 ROTATION STOPPED. Entering Post-Rotation Hold ({hold_duration:.1f}s) <<<")
        self.node.set_phase("5_POST_ROTATION_HOLD")
        time.sleep(hold_duration)
        print(f">>> Post-Rotation Hold Complete. <<<")

    def start_stage_4_return(self):
        print(f"\n>>> RECORDING STAGE 4: Return / Multi-View Traverse <<<")
        self.node.set_phase("6_RETURN_MULTIVIEW")

    def end_stage_4_return(self):
        print(f">>> STAGE 4 RETURN STOPPED. <<<")

    def run_stage_5_final_hold(self, duration=5.0):
        print(f"\n>>> RECORDING STAGE 5: Final Stationary Hold ({duration:.1f}s) <<<")
        self.node.set_phase("7_STATIONARY_END")
        time.sleep(duration)
        print(f">>> STAGE 5 COMPLETE. Finishing session and running Three-Class ASPS... <<<")

    def finalize_and_export(self):
        meta = self.node.stop_and_export()
        rclpy.shutdown()
        if self.spin_thread:
            self.spin_thread.join(timeout=3.0)
        return meta

if __name__ == '__main__':
    print("M12.7 Orchestrator Ready.")
