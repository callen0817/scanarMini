#!/usr/bin/env python3
import subprocess
import time
import urllib.request
import urllib.parse
import json

def get_current_settings():
    url = "http://192.168.1.200/setting_data.json"
    req = urllib.request.Request(url)
    with urllib.request.urlopen(req, timeout=3) as resp:
        return json.loads(resp.read().decode('utf-8'))

def set_airy_pulse_switch(enable=True):
    url = "http://192.168.1.200/Parameter_Setting.html"
    cur = get_current_settings()
    post_data = {
        'SrcIp': cur.get('SrcIp', '192.168.1.200'),
        'SrcMask': cur.get('SrcMask', '255.255.255.0'),
        'SrcGateWay': cur.get('SrcGateWay', '192.168.1.1'),
        'DstIp': cur.get('DstIp', '192.168.1.102'),
        'MPort': cur.get('MPort', '6699'),
        'DPort': cur.get('DPort', '7788'),
        'ReMode': cur.get('ReMode', '0'),
        'FovS': cur.get('FovS', '0'),
        'FovE': cur.get('FovE', '360'),
        'SynSrc': cur.get('SynSrc', '3'),
        'PDomNum': cur.get('PDomNum', '0'),
        'RTPdelay': cur.get('RTPdelay', '0'),
        'NoLeap': cur.get('NoLeap', '0'),
        'SyTOut': cur.get('SyTOut', '5'),
        'UToL': cur.get('UToL', '1'),
        'LToU': cur.get('LToU', '20'),
        'RefEn': cur.get('RefEn', '0'),
        'TFilLvl': cur.get('TFilLvl', '4'),
        'BlkRnDis': cur.get('BlkRnDis', '0'),
        'Rn': cur.get('Rn', '0'),
        'Blk': cur.get('Blk', '0'),
        'OpM': cur.get('OpM', '1'),
        'FSAgl': cur.get('FSAgl', '0'),
        'DZ10cm': cur.get('DZ10cm', '0'),
        'C81858993': cur.get('C81858993', '0'),
        'GapFill': cur.get('GapFill', '0'),
        'RanM': cur.get('RanM', '0'),
        'L48': cur.get('L48', '0'),
        'RnFgSwt': cur.get('RnFgSwt', '0'),
        'PhaLockSet': cur.get('PhaLockSet', '0'),
        'GPSBRate': cur.get('GPSBRate', '3'),
        'ImuCtrl': cur.get('ImuCtrl', '1'),
        'IPort': cur.get('IPort', '6688'),
        'ImuOutR': cur.get('ImuOutR', '3'),
        'AccRg': cur.get('AccRg', '1'),
        'GyroRg': cur.get('GyroRg', '1'),
        'Lpf': cur.get('Lpf', '0'),
        'mode2': 'Mode2',
        'first_start_angle': '0.0',
        'first_pulse_step': '120.0',
        'first_pulse_width': '1000000',
        'save_param_pulse': 'Save'
    }
    if enable:
        post_data['first_group_switch'] = 'ON'
    else:
        post_data['first_group_switch'] = ''
        
    encoded_data = urllib.parse.urlencode(post_data).encode('utf-8')
    req = urllib.request.Request(url, data=encoded_data, method='POST')
    req.add_header('Content-Type', 'application/x-www-form-urlencoded')
    with urllib.request.urlopen(req, timeout=5) as resp:
        pass
    time.sleep(0.5)

print("================================================================================")
print(" MILESTONE M9.4: HARDWARE SYNC LINE DISCONNECT / NEGATIVE CONTROL VERIFICATION")
print("================================================================================")

print("\n[Step 1] Baseline: Airy SYNC_OUT1 ENABLED at 30 Hz...")
set_airy_pulse_switch(True)
cur = get_current_settings()
print(f"   Airy GroupSelect: {cur.get('GroupSelect')} | Step: {cur.get('PulseStep')}° -> SYNC_OUT1 = 30.0 Hz active")

print("\n[Step 2] Measuring Baseline Gemini Left IR frame rate (5s)...")
res1 = subprocess.check_output(
    "bash -c 'source /opt/ros/humble/setup.bash && ros2 topic hz /camera/left_ir/image_raw --window 30 & PID=$!; sleep 4; kill $PID'",
    shell=True, stderr=subprocess.STDOUT
).decode('utf-8')
print("   Baseline Output:\n" + "\n".join(["   " + l for l in res1.strip().split("\n")[-4:]]))

print("\n[Step 3] Negative Trigger Test: DISABLING Airy SYNC_OUT1 (Pin 2 pulse halted)...")
set_airy_pulse_switch(False)
cur = get_current_settings()
print(f"   Airy GroupSelect: {cur.get('GroupSelect')} (0 = SYNC_OUT1 DISABLED / 0 Hz)")

print("\n[Step 4] Checking Gemini Camera response when hardware sync line is silent...")
time.sleep(2)
res2 = subprocess.check_output(
    "bash -c 'source /opt/ros/humble/setup.bash && ros2 topic hz /camera/left_ir/image_raw --window 10 & PID=$!; sleep 4; kill $PID'",
    shell=True, stderr=subprocess.STDOUT
).decode('utf-8')
print("   Negative Response Output:\n" + "\n".join(["   " + l for l in res2.strip().split("\n")[-4:]]))

print("\n[Step 5] Restoring Airy SYNC_OUT1 (Pin 2 pulse 30 Hz resumed)...")
set_airy_pulse_switch(True)
cur = get_current_settings()
print(f"   Airy GroupSelect: {cur.get('GroupSelect')} | Step: {cur.get('PulseStep')}° -> SYNC_OUT1 = 30.0 Hz restored")

print("\n[Step 6] Measuring Gemini Left IR frame rate recovery (5s)...")
time.sleep(1)
res3 = subprocess.check_output(
    "bash -c 'source /opt/ros/humble/setup.bash && ros2 topic hz /camera/left_ir/image_raw --window 30 & PID=$!; sleep 4; kill $PID'",
    shell=True, stderr=subprocess.STDOUT
).decode('utf-8')
print("   Restored Output:\n" + "\n".join(["   " + l for l in res3.strip().split("\n")[-4:]]))
print("\n================================================================================")
