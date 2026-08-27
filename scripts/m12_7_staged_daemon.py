#!/usr/bin/env python3
"""
M12.7 Staged Human-in-the-Loop Daemon
Runs the M127InteractiveTestHarness continuously and executes stages on command.
"""

import os
import sys
import time
import math
import json
import numpy as np
import socket
import threading
import rclpy
from rclpy.node import Node
from m12_7_interactive_test_harness import M127InteractiveTestHarness

SOCKET_PATH = "/tmp/m12_7_daemon.sock"

from rclpy.executors import MultiThreadedExecutor

def spin_worker(node):
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except Exception:
        pass

def run_daemon(dataset_name="M12_7_Controlled_Motion"):
    if os.path.exists(SOCKET_PATH):
        os.remove(SOCKET_PATH)

    rclpy.init()
    harness = M127InteractiveTestHarness(dataset_name=dataset_name)

    spin_thread = threading.Thread(target=spin_worker, args=(harness,), daemon=True)
    spin_thread.start()

    print(f"[DAEMON] Node spinning in thread. Waiting for active topics (Odom/LiDAR/Depth)...")
    t_wait = time.time()
    while (harness.cnt_raw_odom_msgs < 5 or harness.cnt_raw_registered_msgs < 5 or harness.cnt_raw_depth_msgs < 5) and (time.time() - t_wait < 10.0):
        time.sleep(0.1)

    print(f"[DAEMON] Topics active: Odom={harness.cnt_raw_odom_msgs}, Registered={harness.cnt_raw_registered_msgs}, Depth={harness.cnt_raw_depth_msgs}")

    server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    server.bind(SOCKET_PATH)
    server.listen(5)
    print(f"[DAEMON] Listening on UNIX socket {SOCKET_PATH}")

    while True:
        try:
            conn, _ = server.accept()
            data = conn.recv(1024).decode('utf-8').strip()

            if not data:
                conn.close()
                continue

            print(f"[DAEMON] Received command: {data}")
            parts = data.split(":")
            cmd = parts[0]

            if cmd == "STAGE":
                stage_name = parts[1]
                duration = float(parts[2])

                print(f">>> [STAGE START] {stage_name} for {duration:.1f}s <<<")
                harness.set_phase(stage_name)
                
                t0 = time.time()
                last_print = t0
                while time.time() - t0 < duration:
                    time.sleep(0.1)
                    now = time.time()
                    if now - last_print >= 1.0:
                        last_print = now
                        el = now - t0
                        print(f"  [{stage_name}] {el:.1f}/{duration:.1f}s | Airy: {harness.active_phase_stats['raw_cloud_registered']} | Depth: {harness.active_phase_stats['raw_depth_image']} | Odom: {harness.active_phase_stats['raw_aft_mapped_to_init']}")

                # Close stage and collect isolated telemetry
                stage_stats = harness.close_phase(stage_name)

                cur_pose = None
                if len(harness.trajectory_b) > 0:
                    p = harness.trajectory_b[-1]
                    cur_pose = [float(x) if isinstance(x, (float, np.floating, int, np.integer)) else str(x) for x in p]

                resp = {
                    "status": "STAGE_DONE",
                    "stage_name": stage_name,
                    "stage_telemetry": stage_stats,
                    "current_pose": cur_pose
                }
                conn.sendall(json.dumps(resp).encode('utf-8'))
                conn.close()

            elif cmd == "FINALIZE":
                print(">>> [FINALIZE] Stopping harness and exporting datasets... <<<")
                meta = harness.stop_and_export()
                resp = {
                    "status": "FINALIZED",
                    "export_dir": harness.export_dir,
                    "metadata": meta
                }
                conn.sendall(json.dumps(resp).encode('utf-8'))
                conn.close()
                print(">>> [FINALIZE COMPLETE] Exiting daemon. <<<")
                break
        except Exception as e:
            import traceback
            print(f"[DAEMON EXCEPTION]: {e}")
            traceback.print_exc()
            try:
                conn.close()
            except Exception:
                pass

    try:
        rclpy.shutdown()
    except Exception:
        pass
    if spin_thread.is_alive():
        spin_thread.join(timeout=2.0)
    if os.path.exists(SOCKET_PATH):
        os.remove(SOCKET_PATH)

if __name__ == '__main__':
    ds_name = sys.argv[1] if len(sys.argv) > 1 else "M12_7_Controlled_Motion"
    run_daemon(ds_name)
