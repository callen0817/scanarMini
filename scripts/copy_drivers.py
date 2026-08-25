import os
import shutil

SCAN_AR_DIR = "/home/scanar/scan_ar"
CAM_DIR = "/home/scanar/scanAR Cam"
MINI_DIR = "/home/scanar/scanarMini"

DRIVERS_TO_COPY = [
    ("/home/scanar/scan_ar/src/rslidar_sdk", "src/rslidar_sdk", "RoboSense Airy Lidar Driver"),
    ("/home/scanar/scan_ar/src/rslidar_msg", "src/rslidar_msg", "RoboSense Airy Lidar Messages"),
    ("/home/scanar/scan_ar/src/scan_description", "src/scan_description", "Robot Description / URDF Models"),
    ("/home/scanar/scan_ar/src/scan_routine", "src/scan_routine", "Sensor Bringup & Scan Routine Launchers"),
    ("/home/scanar/scanAR Cam/third_party/orbbec_camera", "src/orbbec_camera", "Orbbec Gemini 336L Official Driver")
]

def main():
    print("Copying driver and description packages into scanarMini...")
    for src_full, dest_rel, desc in DRIVERS_TO_COPY:
        dest_full = os.path.join(MINI_DIR, dest_rel)
        if not os.path.exists(src_full):
            print(f"Warning: Source driver path does not exist: {src_full}")
            continue
        
        # Remove destination if it already exists
        if os.path.exists(dest_full):
            print(f"Destination exists, removing: {dest_full}")
            shutil.rmtree(dest_full)
            
        print(f"Copying {src_full} -> {dest_full} ({desc})...")
        shutil.copytree(src_full, dest_full, ignore=shutil.ignore_patterns('__pycache__', 'build', 'install', '.git'))
        
    print("Driver copying completed successfully.")

if __name__ == "__main__":
    main()
