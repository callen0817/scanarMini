#!/usr/bin/env python3
import os
import sys
import time
import signal
import subprocess
import shutil

bag_dir = '/home/scanar/scanarMini/diagnostics/m12_9a_final_eval'
if os.path.exists(bag_dir):
    shutil.rmtree(bag_dir)

print('=== 1. Starting Sensor Bringup (Airy + Gemini + Static TFs) ===')
p_bringup = subprocess.Popen(
    ['bash', '-c', 'source /opt/ros/humble/setup.bash && source /home/scanar/scanarMini/install/setup.bash && ros2 launch scan_routine sensor_bringup.launch.py'],
    stdout=open('/tmp/final_bringup.log', 'w'),
    stderr=subprocess.STDOUT,
    preexec_fn=os.setsid
)

print('=== 2. Starting FAST-LIVO2 Mapping ===')
p_fastlivo = subprocess.Popen(
    ['bash', '-c', 'source /opt/ros/humble/setup.bash && source /home/scanar/scanarMini/install/setup.bash && ros2 launch fast_livo mapping_robosense.launch.py'],
    stdout=open('/tmp/final_fastlivo.log', 'w'),
    stderr=subprocess.STDOUT,
    preexec_fn=os.setsid
)

print('Waiting 5 seconds for system stabilization...')
time.sleep(5.0)

print('=== 3. Starting Lightweight ROS 2 Bag Recording (25s) ===')
topics_to_record = [
    '/rslidar_points',
    '/rslidar_imu_data',
    '/camera/depth/image_raw',
    '/camera/depth/camera_info',
    '/camera/color/image_raw',
    '/camera/color/camera_info',
    '/aft_mapped_to_init',
    '/tf',
    '/tf_static'
]

bag_cmd = ['ros2', 'bag', 'record', '-o', bag_dir] + topics_to_record
p_bag = subprocess.Popen(
    ['bash', '-c', 'source /opt/ros/humble/setup.bash && ' + ' '.join(bag_cmd)],
    stdout=open('/tmp/final_bag.log', 'w'),
    stderr=subprocess.STDOUT,
    preexec_fn=os.setsid
)

print('Recording dataset for 25 seconds...')
for elapsed in range(1, 26):
    time.sleep(1.0)
    if elapsed % 5 == 0:
        print(f'Progress: {elapsed}/25 seconds...')

print('=== 4. Stopping Bag Recording Gracefully (SIGINT) ===')
os.killpg(os.getpgid(p_bag.pid), signal.SIGINT)
try:
    p_bag.wait(timeout=5.0)
except subprocess.TimeoutExpired:
    os.killpg(os.getpgid(p_bag.pid), signal.SIGTERM)

print('=== 5. Stopping Sensor Stack Gracefully ===')
os.killpg(os.getpgid(p_fastlivo.pid), signal.SIGINT)
os.killpg(os.getpgid(p_bringup.pid), signal.SIGINT)
time.sleep(2.0)
try:
    os.killpg(os.getpgid(p_fastlivo.pid), signal.SIGKILL)
    os.killpg(os.getpgid(p_bringup.pid), signal.SIGKILL)
except:
    pass

print('Final registration capture completed cleanly.')
