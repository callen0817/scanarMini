# scanarMini

Handheld reality-capture scanner: RoboSense Airy LiDAR (45° tilt, built-in IMU) + Orbbec Gemini 336L RGB-D on Jetson Orin NX, ROS 2 Humble. SLAM is FAST-LIVO2 (currently LIO only, `img_en: 0`). Current milestone: M12 (RGB colorization + Airy/Gemini depth fusion); see `docs/SCANARMINI_ROADMAP_M0_M30.md`.

**Full orientation guide (layout, how to run everything, script catalog, known issues): `docs/QUICKSTART.md`.**

## Essentials

- Build: `source /opt/ros/humble/setup.bash && colcon build --symlink-install && source install/setup.bash`
- Sensors: `ros2 launch scan_routine sensor_bringup.launch.py`
- SLAM: `ros2 launch fast_livo mapping_robosense.launch.py use_rviz:=true`
- GUI: `./launch_scanhub.sh` (add `--start-stack` to also launch sensors + SLAM; uses `$ROS_DOMAIN_ID` (default 0) + CycloneDDS; other terminals must match)
- Room scan (self-contained): `python3 diagnostics/m12_9_room_scan_orchestrator.py` — deletes the previous dataset/export on each run

## Project code vs vendor code

- Project code: `src/scan_routine/`, `src/scanhub_gui/`, `scripts/`, `diagnostics/`, `config/`
- Vendor code (configure, rarely edit): `src/FAST-LIVO2`, `src/rslidar_sdk`, `src/orbbec_camera`, `src/rpg_vikit`, `src/rslidar_msg`

## Gotchas

- Many docs (`architecture_decisions.md`, `project_roadmap.md`, `hardware_sync_orchestration.md`, `config/sensor_layout.md`, `config/temporal_sync.yaml`) describe the predecessor project *scanR* (IG-2 IMU as master clock). scanarMini has no IG-2: timing is Airy SYNC_OUT1 (120° step → 30 Hz) triggering the Gemini.
- Scripts referencing `/home/scanar/scanR/...` (`diagnostics/m7_*` etc.) are legacy and won't run here.
- Calibration source of truth: `config/production_extrinsics.yaml` (profile `scanarV4_45deg`) and `src/FAST-LIVO2/config/robosense.yaml`. `scanhub_app.py` loads both at import, falling back to hardcoded copies if the yaml can't be read.
- Absolute paths `/home/scanar/scanarMini/...` are hardcoded throughout.
