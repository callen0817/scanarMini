#!/usr/bin/env python3
"""
M12.7 Interactive Qualification Runner (Human-in-the-Loop)
Provides synchronous or command-driven execution of physical motion test stages:
- STAGE 1: Stationary Baseline (5s)
- STAGE 2: 1m Forward Translation
- STAGE 3: Post-Translation Hold (3s)
- STAGE 4: 90 deg Rotation
- STAGE 5: Post-Rotation Hold (3s)
- STAGE 6: Return Translation / Multi-View Traverse
- STAGE 7: Final Stationary Hold (5s)
"""

import os
import sys
import time
import threading
import rclpy
from m12_7_interactive_test_harness import M127InteractiveTestHarness

class M127SessionController:
    def __init__(self, dataset_name="M12_7_Human_Controlled_Motion"):
        self.dataset_name = dataset_name
        self.node = None
        self.spin_thread = None

    def start_stack(self):
        rclpy.init()
        self.node = M127InteractiveTestHarness(dataset_name=self.dataset_name)
        self.spin_thread = threading.Thread(target=rclpy.spin, args=(self.node,), daemon=True)
        self.spin_thread.start()
        time.sleep(1.0)
        print(f"[M12.7 Controller] Test harness running for dataset: {self.dataset_name}")

    def enter_phase(self, phase_name):
        if self.node:
            self.node.set_phase(phase_name)

    def finish(self):
        if self.node:
            meta = self.node.stop_and_export()
            rclpy.shutdown()
            if self.spin_thread:
                self.spin_thread.join(timeout=2.0)
            return meta
        return None

if __name__ == '__main__':
    print("M12.7 Controller Module Ready.")
