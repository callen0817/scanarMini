#!/usr/bin/env python3
import os
import sys
import time
import signal
import subprocess

print('=== TEST 2: AIRY + FAST-LIVO2 + GEMINI (NO ROSBAG) ===')

# Launch sensor bringup (Airy + Gemini)
p_bringup = subprocess.Popen(
    ['bash', '-c', 'source /opt/ros/humble/setup.bash && source /home/scanar/scanarMini/install/setup.bash && ros2 launch scan_routine sensor_bringup.launch.py'],
    stdout=open('/tmp/test2_bringup.log', 'w'),
    stderr=subprocess.STDOUT,
    preexec_fn=os.setsid
)

time.sleep(4.0)

# Launch FAST-LIVO2
p_fastlivo = subprocess.Popen(
    ['bash', '-c', 'source /opt/ros/humble/setup.bash && source /home/scanar/scanarMini/install/setup.bash && ros2 launch fast_livo mapping_robosense.launch.py'],
    stdout=open('/tmp/test2_fastlivo.log', 'w'),
    stderr=subprocess.STDOUT,
    preexec_fn=os.setsid
)

print('Running Airy + FAST-LIVO2 + Gemini for 30 seconds...')
for i in range(1, 31):
    time.sleep(1.0)
    if i % 5 == 0:
        print(f'Progress: {i}/30s...')

print('Stopping processes gracefully...')
os.killpg(os.getpgid(p_fastlivo.pid), signal.SIGINT)
os.killpg(os.getpgid(p_bringup.pid), signal.SIGINT)
time.sleep(2.0)
try:
    os.killpg(os.getpgid(p_fastlivo.pid), signal.SIGKILL)
    os.killpg(os.getpgid(p_bringup.pid), signal.SIGKILL)
except:
    pass

print('Test 2 finished.')
