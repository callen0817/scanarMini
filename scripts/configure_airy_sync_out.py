#!/usr/bin/env python3
import urllib.request
import urllib.parse
import json
import sys

def get_current_settings():
    url = "http://192.168.1.200/setting_data.json"
    req = urllib.request.Request(url)
    with urllib.request.urlopen(req, timeout=3) as resp:
        data = json.loads(resp.read().decode('utf-8'))
    return data

def set_angle_pulse(pulse_step=60.0, pulse_width_ns=1000000, start_angle=0.0):
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
        'first_group_switch': 'ON',
        'first_start_angle': str(start_angle),
        'first_pulse_step': str(pulse_step),
        'first_pulse_width': str(pulse_width_ns),
        'save_param_pulse': 'Save'
    }
    
    encoded_data = urllib.parse.urlencode(post_data).encode('utf-8')
    req = urllib.request.Request(url, data=encoded_data, method='POST')
    req.add_header('Content-Type', 'application/x-www-form-urlencoded')
    
    print(f"Sending POST to {url} with PulseStep={pulse_step}°, PulseWidth={pulse_width_ns}ns...")
    with urllib.request.urlopen(req, timeout=5) as resp:
        print(f"Response status: {resp.status}")
        
    # Verify update
    new_data = get_current_settings()
    print("\n--- Verified Live Airy Settings from Device ---")
    print(f"GroupSelect (Pulse Output Switch): {new_data.get('GroupSelect')}")
    print(f"PulseStart:                        {new_data.get('PulseStart')} DEG")
    print(f"PulseStep:                         {new_data.get('PulseStep')} DEG")
    print(f"PulseWidth:                        {new_data.get('PulseWidth')} ns")
    
    step_val = float(new_data.get('PulseStep', 360.0))
    pulses_per_rev = 360.0 / step_val if step_val > 0 else 0
    freq_hz = 10.0 * pulses_per_rev
    print(f"Hardware SYNC_OUT1 Frequency @ 600 RPM (10 Hz rotation): {freq_hz:.2f} Hz")
    return new_data

if __name__ == '__main__':
    step = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
    width = int(sys.argv[2]) if len(sys.argv) > 2 else 1000000
    set_angle_pulse(pulse_step=step, pulse_width_ns=width)
