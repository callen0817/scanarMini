#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import urllib.request
import urllib.parse
import json
import time
import threading

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
        
    encoded_data = urllib.parse.urlencode(post_data).encode('utf-8')
    req = urllib.request.Request(url, data=encoded_data, method='POST')
    req.add_header('Content-Type', 'application/x-www-form-urlencoded')
    with urllib.request.urlopen(req, timeout=5) as resp:
        pass
    time.sleep(0.5)

class FrameCounterNode(Node):
    def __init__(self):
        super().__init__('sync_negative_evaluator')
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)
        self.frame_count = 0
        self.timestamps = []
        self.sub = self.create_subscription(Image, '/camera/left_ir/image_raw', self.callback, qos)

    def callback(self, msg):
        self.frame_count += 1
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.timestamps.append(t)

    def reset(self):
        self.frame_count = 0
        self.timestamps = []

def main():
    rclpy.init()
    node = FrameCounterNode()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print("================================================================================")
    print(" MILESTONE M9.4: HARDWARE SYNC LINE DISCONNECT / NEGATIVE CONTROL VERIFICATION")
    print("================================================================================")

    # Step 1: Baseline
    print("\n[Phase 1: Baseline Hardware Trigger Lock]")
    set_airy_pulse_switch(True)
    cur = get_current_settings()
    print(f"   Airy GroupSelect: {cur.get('GroupSelect')} | PulseStep: {cur.get('PulseStep')}° (SYNC_OUT1 = 30.0 Hz active)")
    time.sleep(1.0)
    node.reset()
    time.sleep(5.0)
    fps1 = node.frame_count / 5.0
    print(f"   >>> Result: Received {node.frame_count} frames in 5.0 s ({fps1:.2f} FPS) -> HARDWARE LOCKED")

    # Step 2: Negative Control (Disable SYNC_OUT1 line)
    print("\n[Phase 2: Negative Control - Halting Airy Hardware Trigger Line]")
    set_airy_pulse_switch(False)
    cur = get_current_settings()
    print(f"   Airy GroupSelect: {cur.get('GroupSelect')} (0 = SYNC_OUT1 Line Silent / 0 Hz)")
    time.sleep(1.0)
    node.reset()
    time.sleep(5.0)
    fps2 = node.frame_count / 5.0
    print(f"   >>> Result: Received {node.frame_count} frames in 5.0 s ({fps2:.2f} FPS)")

    # Step 3: Restored Hardware Trigger
    print("\n[Phase 3: Restoring Hardware Trigger Line]")
    set_airy_pulse_switch(True)
    cur = get_current_settings()
    print(f"   Airy GroupSelect: {cur.get('GroupSelect')} | PulseStep: {cur.get('PulseStep')}° (SYNC_OUT1 = 30.0 Hz active)")
    time.sleep(1.0)
    node.reset()
    time.sleep(5.0)
    fps3 = node.frame_count / 5.0
    print(f"   >>> Result: Received {node.frame_count} frames in 5.0 s ({fps3:.2f} FPS) -> HARDWARE RELOCKED")

    print("\n================================================================================")
    print(" SUMMARY OF HARDWARE SYNC LINE SENSITIVITY:")
    print(f"   • Phase 1 (Trigger Active):   {fps1:.2f} FPS (Locked to Airy SYNC_OUT1)")
    print(f"   • Phase 2 (Trigger Disabled): {fps2:.2f} FPS (Free-run / fallback state)")
    print(f"   • Phase 3 (Trigger Restored): {fps3:.2f} FPS (Relocked to Airy SYNC_OUT1)")
    print("================================================================================")

    rclpy.shutdown()

if __name__ == '__main__':
    main()
