#!/usr/bin/env python3
import os
import sys
import time
import json
import signal
import subprocess
import urllib.request
import urllib.parse

def verify_airy_hardware_sync():
    print('--> [Phase 1/4] Verifying Airy Hardware SYNC_OUT1 Configuration...')
    url_setting = 'http://192.168.1.200/setting_data.json'
    url_diag = 'http://192.168.1.200/diag_data.json'
    url_post = 'http://192.168.1.200/Parameter_Setting.html'
    
    for attempt in range(5):
        try:
            req_set = urllib.request.urlopen(url_setting, timeout=2.0)
            settings = json.loads(req_set.read().decode('utf-8'))
            
            req_diag = urllib.request.urlopen(url_diag, timeout=2.0)
            diag = json.loads(req_diag.read().decode('utf-8'))
            
            step = float(settings.get('PulseStep', 0.0))
            width = int(settings.get('PulseWidth', 0))
            group = int(settings.get('GroupSelect', 0))
            rpm = diag.get('rpm')
            
            if rpm is None:
                print(f'[AirySync WARN] RPM absent from diagnostic endpoint. Attempt {attempt+1}/5...')
                time.sleep(1.0)
                continue
                
            rpm = int(rpm)
            
            if abs(step - 120.0) < 0.1 and width == 1000000 and group == 1 and abs(rpm - 600) < 30:
                print(f'[AirySync OK] Step: {step:.1f} deg | Width: {width} ns | Group: {group} | Measured RPM: {rpm}')
                print('              Airy SYNC_OUT1 is configured for an expected ~30 Hz cadence from ~600 RPM and a 120 deg pulse step.')
                return True
            else:
                print(f'[AirySync WARN] Non-nominal config (step={step}, width={width}, group={group}, rpm={rpm}). Configuring 120 deg / 1 ms...')
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
            print(f'[AirySync] Connection attempt {attempt+1}/5: {e}')
            time.sleep(1.0)
            
    print('[AirySync ERROR] Could not verify and lock nominal Airy hardware sync parameters!')
    return False

def run_room_scan():
    dataset_dir = '/home/scanar/scanarMini/diagnostics/m12_9_room_dataset'
    export_dir = '/home/scanar/scanarMini/diagnostics/m12_9_room_export'
    
    # 0. Clean old dataset
    import shutil
    if os.path.exists(dataset_dir):
        shutil.rmtree(dataset_dir)
    os.makedirs(os.path.join(dataset_dir, 'lidar'), exist_ok=True)
    os.makedirs(os.path.join(dataset_dir, 'depth'), exist_ok=True)
    os.makedirs(os.path.join(dataset_dir, 'color'), exist_ok=True)
    
    # 1. Verify Airy Hardware Sync (Fail-Closed Gate)
    if not verify_airy_hardware_sync():
        raise SystemExit('FATAL: Airy hardware sync verification failed. Aborting startup.')

    env_cmd = 'source /opt/ros/humble/setup.bash && source /home/scanar/scanarMini/install/setup.bash'

    # 2. Launch Sensor Bringup
    print('\n--> [Phase 2/4] Starting Sensor Bringup (Airy 45-deg + Gemini 336L)...')
    p_bringup = subprocess.Popen(
        ['bash', '-c', f'{env_cmd} && ros2 launch scan_routine sensor_bringup.launch.py use_hardware_trigger:=true'],
        stdout=open('/tmp/m12_9_bringup.log', 'w'),
        stderr=subprocess.STDOUT,
        preexec_fn=os.setsid
    )
    time.sleep(5.0)

    # 3. Launch FAST-LIVO2
    print('\n--> [Phase 3/4] Starting FAST-LIVO2 Odometry & Mapping...')
    p_fastlivo = subprocess.Popen(
        ['bash', '-c', f'{env_cmd} && ros2 launch fast_livo mapping_robosense.launch.py'],
        stdout=open('/tmp/m12_9_fastlivo.log', 'w'),
        stderr=subprocess.STDOUT,
        preexec_fn=os.setsid
    )
    time.sleep(4.0)

    # 4. Launch Multi-process Recorders
    print('\n--> [Phase 4/4] Starting Asynchronous Multi-Process Capture Recorders...')
    p_rec_lidar = subprocess.Popen(
        ['bash', '-c', f'{env_cmd} && python3 /home/scanar/scanarMini/src/scan_routine/scripts/record_lidar.py {dataset_dir}'],
        stdout=open('/tmp/m12_9_rec_lidar.log', 'w'),
        stderr=subprocess.STDOUT,
        preexec_fn=os.setsid
    )
    p_rec_cam = subprocess.Popen(
        ['bash', '-c', f'{env_cmd} && python3 /home/scanar/scanarMini/src/scan_routine/scripts/record_camera.py {dataset_dir}'],
        stdout=open('/tmp/m12_9_rec_cam.log', 'w'),
        stderr=subprocess.STDOUT,
        preexec_fn=os.setsid
    )

    print('\n' + '='*75)
    print('>>> SYSTEM FULLY ACTIVE & RECORDING! READY FOR ROOM WALK <<<')
    print('Protocol:')
    print('  1. Hold steady for 5-10s stationary start')
    print('  2. Walk perimeter deliberately (~0.5 - 0.8 m/s)')
    print('  3. Observe walls, details, corners, floor-wall intersections, doorway geometry')
    print('  4. Perform stationary ~90-deg observation pans at key viewpoints')
    print('  5. Return to start position')
    print('  6. Hold steady for 5-10s stationary end')
    print('='*75)

    def shutdown(sig, frame):
        print('\n[Stopping Capture...] Gracefully stopping recorders and sensor stack...')
        try:
            os.killpg(os.getpgid(p_rec_lidar.pid), signal.SIGINT)
            os.killpg(os.getpgid(p_rec_cam.pid), signal.SIGINT)
            time.sleep(2.0)
        except Exception:
            pass
        try:
            os.killpg(os.getpgid(p_fastlivo.pid), signal.SIGINT)
            time.sleep(1.0)
        except Exception:
            pass
        try:
            os.killpg(os.getpgid(p_bringup.pid), signal.SIGINT)
            time.sleep(2.0)
        except Exception:
            pass
        if '--no-reconstruct' not in sys.argv:
            print('[Capture Complete] Post-processing deliverables and coverage metrics...')
            from process_m12_9_room_scan import process_room_scan
            process_room_scan(dataset_dir, export_dir)
        else:
            print('[Capture Complete] Recorders stopped. Dataset preserved for evaluation.')
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown)

    t0 = time.time()
    lidar_dir = os.path.join(dataset_dir, 'lidar')
    depth_dir = os.path.join(dataset_dir, 'depth')
    color_dir = os.path.join(dataset_dir, 'color')
    
    try:
        while True:
            time.sleep(1.0)
            elapsed = time.time() - t0
            n_lidar = len(os.listdir(lidar_dir)) if os.path.exists(lidar_dir) else 0
            n_depth = len(os.listdir(depth_dir)) if os.path.exists(depth_dir) else 0
            n_color = len(os.listdir(color_dir)) if os.path.exists(color_dir) else 0
            
            l_rate = n_lidar / elapsed if elapsed > 0 else 0
            d_rate = n_depth / elapsed if elapsed > 0 else 0
            c_rate = n_color / elapsed if elapsed > 0 else 0
            
            print(f'\r[RECORDING {elapsed:5.1f}s] Airy Sweeps: {n_lidar:4d} ({l_rate:4.1f} Hz) | Depth: {n_depth:4d} ({d_rate:4.1f} Hz) | RGB: {n_color:4d} ({c_rate:4.1f} Hz)', end='', flush=True)
    except KeyboardInterrupt:
        shutdown(None, None)

if __name__ == '__main__':
    run_room_scan()
