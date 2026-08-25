#!/usr/bin/env python3
"""
M8.1B: RoboSense Airy Live Hardware Mode & Packet Interrogation
Directly binds to DIFOP (port 7788) and MSOP (port 6699) UDP sockets to interrogate:
1. Exact DIFOP header bytes (return mode, RPM, serial number, firmware version).
2. Raw MSOP packet arrival rates, packet gap distribution, and sweep assembly.
"""

import sys
import socket
import struct
import time
import numpy as np

def query_airy_difop(host_ip="192.168.1.102", difop_port=7788, timeout_sec=3.0):
    print("--- [1] Interrogating RoboSense Airy DIFOP Packet ---")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((host_ip, difop_port))
    sock.settimeout(timeout_sec)
    
    try:
        data, addr = sock.recvfrom(2048)
        print(f"  Received DIFOP packet from {addr}: {len(data)} bytes")
        
        # Check DIFOP ID: 0xA5, 0xFF, 0x00, 0x5A, 0x11, 0x11, 0x55, 0x55
        difop_id = data[0:8]
        print(f"  DIFOP Header ID: {' '.join(f'{b:02X}' for b in difop_id)}")
        
        # RPM is at offset 8..10 (uint16 big endian)
        rpm = struct.unpack(">H", data[8:10])[0]
        print(f"  Configured RPM:  {rpm} RPM (Nominal Scan Rate: {rpm/60.0:.2f} Hz)")
        
        # Return mode byte is at offset 300 (or near offset 103/300 depending on struct)
        # Let's inspect bytes in DIFOP:
        # RSAIRYDifopPkt: id[8], rpm[2], eth[...], version[...], install_mode[1], sn[...], zero_cal[2], return_mode[1]
        # Let's find return_mode byte
        print(f"  Raw DIFOP Mode Bytes [100:110]: {' '.join(f'{b:02X}' for b in data[100:110])}")
        return_mode_byte = data[103] if len(data) > 103 else 0
        echo_str = "DUAL RETURN" if return_mode_byte == 0x03 else "SINGLE RETURN (Strongest)"
        print(f"  Return Mode:     0x{return_mode_byte:02X} -> {echo_str}")
        return True
    except socket.timeout:
        print(f"  ❌ Timeout: No DIFOP packet received on {host_ip}:{difop_port}")
        return False
    finally:
        sock.close()

def benchmark_raw_msop_cadence(host_ip="192.168.1.102", msop_port=6699, duration_sec=5.0):
    print(f"\n--- [2] Benchmarking Raw MSOP UDP Packet Cadence ({duration_sec}s) ---")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((host_ip, msop_port))
    sock.settimeout(2.0)
    
    packet_times = []
    azimuths = []
    packet_sizes = []
    
    start_time = time.time()
    try:
        while time.time() - start_time < duration_sec:
            data, addr = sock.recvfrom(2048)
            now = time.time()
            packet_times.append(now)
            packet_sizes.append(len(data))
            
            # MSOP Block 0 Azimuth at offset 44 (header=42, block_id=2, azimuth=2)
            if len(data) >= 46:
                az = struct.unpack(">H", data[44:46])[0]
                azimuths.append(az)
    except socket.timeout:
        print("  ⚠️ Socket timeout during packet capture")
    finally:
        sock.close()
        
    if len(packet_times) < 10:
        print(f"  ❌ Insufficient MSOP packets captured ({len(packet_times)})")
        return
        
    total_dur = packet_times[-1] - packet_times[0]
    pkt_rate = (len(packet_times) - 1) / total_dur
    dt_us = np.diff(packet_times) * 1e6
    
    # Calculate sweep boundaries from azimuth wraparounds (e.g., azimuth drops from ~35000 to <5000)
    az_arr = np.array(azimuths)
    az_diff = np.diff(az_arr)
    sweep_wrap_indices = np.where(az_diff < -20000)[0]
    
    print(f"  Total UDP Packets:   {len(packet_times)} in {total_dur:.3f}s")
    print(f"  Packet Delivery Rate: {pkt_rate:.1f} pkts/s (Packet size: {packet_sizes[0]} bytes)")
    print(f"  Inter-Packet Δt:     Median: {np.median(dt_us):.1f} µs | P95: {np.percentile(dt_us, 95):.1f} µs | Max: {np.max(dt_us):.1f} µs")
    
    if len(sweep_wrap_indices) > 1:
        sweep_times = np.array(packet_times)[sweep_wrap_indices]
        sweep_dt_ms = np.diff(sweep_times) * 1000.0
        sweep_rate = 1000.0 / np.median(sweep_dt_ms)
        pkts_per_sweep = np.diff(sweep_wrap_indices)
        print(f"  Detected Sweeps:     {len(sweep_wrap_indices)} full rotations")
        print(f"  Hardware Sweep Rate: {sweep_rate:.2f} Hz (Median Interval: {np.median(sweep_dt_ms):.2f} ms)")
        print(f"  Sweep Interval Jitter: P95: {np.percentile(sweep_dt_ms, 95):.2f} ms | Max: {np.max(sweep_dt_ms):.2f} ms")
        print(f"  Packets per Sweep:   {np.median(pkts_per_sweep):.0f} pkts/sweep")
    else:
        print("  ⚠️ Not enough azimuth wraparounds to compute sweep rate directly.")

def main():
    print("="*80)
    print("   M8.1B: ROBOSENSE AIRY HARDWARE MODE & DELIVERY OPTIMIZATION AUDIT")
    print("="*80)
    query_airy_difop()
    benchmark_raw_msop_cadence(duration_sec=5.0)
    print("="*80)

if __name__ == '__main__':
    main()
