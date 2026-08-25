#!/usr/bin/env python3
"""
Upstream LiDAR Packet Loss & Missing Scan Audit Script for Session Scan_0804_1443_20260804_144415.
Calculates expected vs actual vs missing scans and inspects Linux NIC socket statistics.
"""

import sys
import os
import csv
import subprocess

session_dir = "/home/scanar/scanR/diagnostics/Scan_0804_1443_20260804_144415"
lidar_csv = os.path.join(session_dir, "lidar.csv")

print("======================================================================")
print("UPSTREAM LIDAR PACKET LOSS & MISSING SCAN AUDIT")
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

first_hdr = scans[0][1]
last_hdr = scans[-1][1]
duration_sec = last_hdr - first_hdr

actual_scans = len(scans)
expected_scans = round(duration_sec * 10.0) # Nominal 10 Hz rotation rate
missing_scans = max(0, expected_scans - actual_scans)
missing_pct = (missing_scans / expected_scans) * 100.0 if expected_scans > 0 else 0.0

print(f"\n1. CAPTURE DURATION & SCAN METRICS:")
print(f"   * Capture Duration:       {duration_sec:.2f} seconds")
print(f"   * Nominal Scan Rate:      10.0 Hz (100.0 ms per rotation)")
print(f"   * Expected Scans (10Hz):  {expected_scans} scans")
print(f"   * Actual Scans Received:  {actual_scans} scans")
print(f"   * Missing Scans:          {missing_scans} scans")
print(f"   * Missing Scan Deficit:   {missing_pct:.1f}%")

# 2. Linux Network Interface & Socket Buffer Audit
print("\n2. HARDWARE & LINUX NIC SOCKET STATISTICS:")
eth_iface = "enP8p1s0" # Jetson Ethernet port for LiDAR
sys_net_path = f"/sys/class/net/{eth_iface}/statistics"

if os.path.exists(sys_net_path):
    def read_net_stat(stat_name):
        try:
            with open(os.path.join(sys_net_path, stat_name), "r") as f:
                return int(f.read().strip())
        except Exception:
            return -1

    rx_packets = read_net_stat("rx_packets")
    rx_dropped = read_net_stat("rx_dropped")
    rx_errors = read_net_stat("rx_errors")
    rx_fifo_errors = read_net_stat("rx_fifo_errors")
    rx_missed_errors = read_net_stat("rx_missed_errors")

    print(f"   * Interface:            {eth_iface}")
    print(f"   * RX Total Packets:     {rx_packets}")
    print(f"   * RX Dropped Packets:   {rx_dropped}")
    print(f"   * RX FIFO Overflows:    {rx_fifo_errors}")
    print(f"   * RX Missed Packets:    {rx_missed_errors}")
    print(f"   * RX Frame Errors:      {rx_errors}")
else:
    print(f"   * Interface {eth_iface} statistics path not found.")

print("\n======================================================================")
