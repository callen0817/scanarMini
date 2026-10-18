#!/usr/bin/env python3
import os
import sys
import time
import signal
import subprocess
import shutil

bag_dir = '/home/scanar/scanarMini/diagnostics/m12_9a_transport_30s'
if os.path.exists(bag_dir):
    shutil.rmtree(bag_dir)

print('=== 1. Starting Sensor Bringup Stack ===')
p_bringup = subprocess.Popen(
    ['bash', '-c', 'source /opt/ros/humble/setup.bash && source /home/scanar/scanarMini/install/setup.bash && ros2 launch scan_routine sensor_bringup.launch.py'],
    stdout=open('/tmp/bringup_transport.log', 'w'),
    stderr=subprocess.STDOUT,
    preexec_fn=os.setsid
)

print('=== 2. Starting FAST-LIVO2 Mapping ===')
p_fastlivo = subprocess.Popen(
    ['bash', '-c', 'source /opt/ros/humble/setup.bash && source /home/scanar/scanarMini/install/setup.bash && ros2 launch fast_livo mapping_robosense.launch.py'],
    stdout=open('/tmp/fastlivo_transport.log', 'w'),
    stderr=subprocess.STDOUT,
    preexec_fn=os.setsid
)

print('Waiting 5 seconds for sensor initialization...')
time.sleep(5.0)

print('=== 3. Starting Lightweight ROS 2 Bag Recording (30s) ===')
# Lightweight topics ONLY
topics_to_record = [
    '/rslidar_points',
    '/rslidar_imu_data',
    '/camera/depth/image_raw',
    '/camera/color/image_raw',
    '/aft_mapped_to_init',
    '/tf',
    '/tf_static'
]

bag_cmd = ['ros2', 'bag', 'record', '-o', bag_dir] + topics_to_record
p_bag = subprocess.Popen(
    ['bash', '-c', 'source /opt/ros/humble/setup.bash && ' + ' '.join(bag_cmd)],
    stdout=open('/tmp/bag_transport.log', 'w'),
    stderr=subprocess.STDOUT,
    preexec_fn=os.setsid
)

start_time = time.time()
print('Recording active for 30 seconds...')
try:
    for elapsed in range(1, 31):
        time.sleep(1.0)
        if elapsed % 5 == 0:
            print(f'Progress: {elapsed}/30 seconds...')
except KeyboardInterrupt:
    print('Interrupted early.')

print('=== 4. Stopping Bag Recording Gracefully (SIGINT) ===')
os.killpg(os.getpgid(p_bag.pid), signal.SIGINT)
try:
    p_bag.wait(timeout=5.0)
except subprocess.TimeoutExpired:
    os.killpg(os.getpgid(p_bag.pid), signal.SIGTERM)

print('=== 5. Stopping Sensor Nodes Gracefully ===')
os.killpg(os.getpgid(p_fastlivo.pid), signal.SIGINT)
os.killpg(os.getpgid(p_bringup.pid), signal.SIGINT)
time.sleep(2.0)
try:
    os.killpg(os.getpgid(p_fastlivo.pid), signal.SIGKILL)
    os.killpg(os.getpgid(p_bringup.pid), signal.SIGKILL)
except:
    pass

print('Transport test session completed cleanly.')
