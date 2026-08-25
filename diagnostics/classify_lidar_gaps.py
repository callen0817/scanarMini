#!/usr/bin/env python3
"""
LiDAR Gap Classification Script for Session Scan_0804_1443_20260804_144415.
Distinguishes Case A (True Sensor/Driver Drop), Case B (Transport Delay), and Case C (Buffered Burst).
"""

import sys
import os
import csv

session_dir = "/home/scanar/scanR/diagnostics/Scan_0804_1443_20260804_144415"
lidar_csv = os.path.join(session_dir, "lidar.csv")

print("======================================================================")
print("LiDAR GAP CLASSIFICATION & DELAY ANALYSIS")
print("======================================================================")

if not os.path.exists(lidar_csv):
    print(f"Error: {lidar_csv} not found!")
    sys.exit(1)

scans = []
with open(lidar_csv, "r") as f:
    reader = csv.DictReader(f)
    for row in reader:
        sys_t = float(row["sys_time"])
        hdr_t = float(row["sensor_sec"])
        pts = int(row["point_count"])
        scans.append((sys_t, hdr_t, pts))

print(f"Total Scans Analyzed: {len(scans)}")
t0 = scans[0][1]

case_a_cnt = 0 # True Drop (hdr_delta ~ 200-400ms, recv_delta ~ 200-400ms)
case_b_cnt = 0 # Transport Delay (hdr_delta ~ 100ms, recv_delta ~ 200-400ms)
case_c_cnt = 0 # Buffered Burst (hdr_delta ~ 100ms, recv_delta ~ 5-10ms after gap)

print("\n--- DETAILED GAP TIMING BREAKDOWN (All Scans with Delta > 150ms) ---")
print(f"{'Index':<6} | {'Rel Time (s)':<12} | {'Hdr Delta (ms)':<16} | {'Recv Delta (ms)':<16} | {'Recv Delay (ms)':<16} | {'Classification'}")
print("-" * 95)

for i in range(1, len(scans)):
    sys_t, hdr_t, pts = scans[i]
    prev_sys, prev_hdr, _ = scans[i-1]
    
    hdr_delta_ms = (hdr_t - prev_hdr) * 1000.0
    recv_delta_ms = (sys_t - prev_sys) * 1000.0
    recv_delay_ms = (sys_t - hdr_t) * 1000.0
    rel_t = hdr_t - t0

    if hdr_delta_ms > 150.0 or recv_delta_ms > 150.0:
        if hdr_delta_ms > 150.0 and abs(hdr_delta_ms - recv_delta_ms) < 50.0:
            classification = "CASE A (True Sensor/Driver Drop)"
            case_a_cnt += 1
        elif recv_delta_ms < 20.0 and hdr_delta_ms < 120.0:
            classification = "CASE C (Buffered Burst Delivery)"
            case_c_cnt += 1
        else:
            classification = "CASE B (Transport/Network Delay)"
            case_b_cnt += 1

        print(f"{i:<6} | {rel_t:<12.2f} | {hdr_delta_ms:<16.1f} | {recv_delta_ms:<16.1f} | {recv_delay_ms:<16.1f} | {classification}")

print("\n======================================================================")
print("CLASSIFICATION SUMMARY:")
print(f"  * Case A (True Sensor/Driver Drops - Scans never created): {case_a_cnt}")
print(f"  * Case B (Transport/Network Delays - Scans delayed in OS): {case_b_cnt}")
print(f"  * Case C (Buffered Bursts - Queued delivery after gap):    {case_c_cnt}")
print("======================================================================")
