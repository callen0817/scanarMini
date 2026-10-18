#!/usr/bin/env python3
import os
import sys
import time
import signal
import subprocess

print('=== TEST 1: ISOLATED AIRY + FAST-LIVO2 (NO GEMINI, NO ROSBAG, NO GUI) ===')

# Launch rslidar_sdk node directly
rslidar_config = '/home/scanar/scanarMini/src/rslidar_sdk/config/drone_config.yaml'
p_rslidar = subprocess.Popen(
    ['bash', '-c', f'source /opt/ros/humble/setup.bash && source /home/scanar/scanarMini/install/setup.bash && ros2 run rslidar_sdk rslidar_sdk_node --ros-args -p config_path:={rslidar_config}'],
    stdout=open('/tmp/test1_rslidar.log', 'w'),
    stderr=subprocess.STDOUT,
    preexec_fn=os.setsid
)

time.sleep(2.0)

# Launch FAST-LIVO2
p_fastlivo = subprocess.Popen(
    ['bash', '-c', 'source /opt/ros/humble/setup.bash && source /home/scanar/scanarMini/install/setup.bash && ros2 launch fast_livo mapping_robosense.launch.py'],
    stdout=open('/tmp/test1_fastlivo.log', 'w'),
    stderr=subprocess.STDOUT,
    preexec_fn=os.setsid
)

print('Running isolated Airy + FAST-LIVO2 for 30 seconds...')
for i in range(1, 31):
    time.sleep(1.0)
    if i % 5 == 0:
        print(f'Progress: {i}/30s...')

print('Stopping processes gracefully...')
os.killpg(os.getpgid(p_fastlivo.pid), signal.SIGINT)
os.killpg(os.getpgid(p_rslidar.pid), signal.SIGINT)
time.sleep(2.0)
try:
    os.killpg(os.getpgid(p_fastlivo.pid), signal.SIGKILL)
    os.killpg(os.getpgid(p_rslidar.pid), signal.SIGKILL)
except:
    pass

print('Test 1 finished.')
