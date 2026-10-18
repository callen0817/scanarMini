#!/usr/bin/env python3
import urllib.request
import urllib.parse
import json
import time
import sys

def verify_and_configure_airy_sync():
    url_setting = 'http://192.168.1.200/setting_data.json'
    url_post = 'http://192.168.1.200/Parameter_Setting.html'
    
    for attempt in range(5):
        try:
            req = urllib.request.urlopen(url_setting, timeout=2.0)
            settings = json.loads(req.read().decode('utf-8'))
            
            step = float(settings.get('PulseStep', 0.0))
            width = int(settings.get('PulseWidth', 0))
            group = int(settings.get('GroupSelect', 0))
            
            if abs(step - 120.0) < 0.1 and width == 1000000 and group == 1:
                print(f'[AirySync] Verified: PulseStep={step} deg, PulseWidth={width} ns, GroupSelect={group}. Expected ~30 Hz active.')
                return True
            else:
                print(f'[AirySync] Warning: Non-nominal config detected (step={step}, width={width}, group={group}). Configuring 120 deg / 1 ms...')
                data = urllib.parse.urlencode({
                    'mode1': 'on',
                    'first_group_switch': 'ON',
                    'first_start_angle': '0',
                    'first_pulse_width': '1000000',
                    'first_pulse_step': '120.0',
                    'save_param_pulse': 'Save'
                }).encode('utf-8')
                post_req = urllib.request.Request(url_post, data=data, method='POST')
                urllib.request.urlopen(post_req, timeout=3.0)
                time.sleep(1.0)
        except Exception as e:
            print(f'[AirySync] Attempt {attempt+1}/5 connection to Airy web server: {e}')
            time.sleep(1.0)
            
    print('[AirySync] Note: Could not confirm Airy web server. Proceeding with active hardware state.')
    return False

if __name__ == '__main__':
    verify_and_configure_airy_sync()
