# scanarMini Quickstart & Codebase Map

Orientation guide: what things are, where they live, and how to run them.
Derived from reading the code and shell history (October 2026, milestone M12.9). Commands reflect what the code expects; verify on hardware.

---

## 1. Mental Model

The system has four layers. Each layer needs the one above it running.

```
Layer 1  SENSORS   rslidar_sdk (Airy LiDAR + IMU) + orbbec_camera (Gemini RGB/depth)
            │      ros2 launch scan_routine sensor_bringup.launch.py
            ▼
Layer 2  SLAM      FAST-LIVO2 → pose (/aft_mapped_to_init) + world-frame cloud (/cloud_registered)
            │      ros2 launch fast_livo mapping_robosense.launch.py
            ▼
Layer 3  CONSUMERS GUI (scanhub_app.py), recorders, M12.x scan scripts → write to data/ or diagnostics/
            ▼
Layer 4  OFFLINE   scripts reading saved files → PLY clouds, metrics, plots (no ROS needed)
```

Knowing a script's layer tells you what must be running first.

**Hardware:** RoboSense Airy LiDAR (45° tilt, Ethernet) with built-in IMU, Orbbec Gemini 336L RGB-D camera (USB 3), Jetson Orin NX, JetPack 6, ROS 2 Humble.

**Timing:** Airy spins at 600 rpm (10 Hz). Its SYNC_OUT1 fires every 120° → 30 Hz hardware trigger into the Gemini (`sync_mode: SECONDARY`).

**Lineage:** scanarMini evolved from an earlier project, *scanR*, which used an InertialSense IG-2 as master IMU/clock. Several docs (`architecture_decisions.md`, `project_roadmap.md`, `hardware_sync_orchestration.md`, `config/sensor_layout.md`, `config/temporal_sync.yaml`) still describe the IG-2 setup. `docs/SCANARMINI_ROADMAP_M0_M30.md` is the current roadmap.

---

## 2. Directory Map

```
scanarMini/
├── launch_scanhub.sh          starts the GUI (add --start-stack to also launch sensors + SLAM)
├── src/                       ROS 2 workspace (colcon builds this)
│   ├── scan_routine/          ★ project package: launch files + helper nodes
│   │   ├── launch/sensor_bringup.launch.py   sensors + static TFs
│   │   ├── launch/main.launch.py             sensors + SLAM + foxglove (all-in-one)
│   │   └── scripts/   topic_relay, verify_airy_sync, record_lidar/camera, recorders
│   ├── scanhub_gui/           ★ PyQt GUI (plain Python, not a ROS package)
│   ├── scan_description/      URDF/xacro + STL of the rig
│   ├── FAST-LIVO2/            SLAM (vendor, lightly modified). config/robosense.yaml = project settings
│   ├── rslidar_sdk/ rslidar_msg/   Airy driver (vendor). config/drone_config.yaml = project settings
│   ├── orbbec_camera/         Gemini driver (vendor)
│   └── rpg_vikit/             FAST-LIVO2 dependency (vendor)
├── config/                    ★ calibration: production_extrinsics.yaml (active), kalibr yamls, cyclonedds.xml
├── calibration/               raw calibration sources (factory, CAD, kalibr bag)
├── scripts/                   ★ milestone test/analysis scripts (M8 → M12.8) + C++ benchmarks
├── diagnostics/               ★ newest work (M12.9) + legacy scanR-era (M7) scripts + output folders
├── data/                      scan outputs, one folder per session
├── docs/                      roadmaps, ADRs, this file
├── build/ install/ log/       colcon output (gitignored)
└── tests/ slam/ mapping/ launch/ gui/ export/ reports/   empty placeholders
```

★ = project code. Everything else under `src/` is vendor code that is mostly only configured.

---

## 3. Build

```bash
cd ~/scanarMini
source /opt/ros/humble/setup.bash
colcon build --symlink-install          # or: --packages-select scan_routine fast_livo
source install/setup.bash               # required in every new terminal
```

Packages: `scan_routine scan_description fast_livo rslidar_sdk rslidar_msg orbbec_camera orbbec_camera_msgs orbbec_description vikit_common vikit_py vikit_ros`.

Python scripts run directly from source (`python3 /path/script.py`); no rebuild needed after editing them. Launch files and YAML configs need a rebuild unless built with `--symlink-install`.

---

## 4. Running the Live System

Use one terminal per layer.

**Terminal 1 — sensors**
```bash
ros2 launch scan_routine sensor_bringup.launch.py
```
Runs `verify_airy_sync.py` (checks/fixes the Airy 120° / 30 Hz trigger via its web UI at 192.168.1.200), then starts both drivers, the IR topic relay, and the static TF tree.

**Terminal 2 — SLAM**
```bash
ros2 launch fast_livo mapping_robosense.launch.py use_rviz:=true
```
Currently pure LiDAR-inertial (`img_en: 0` in `robosense.yaml`). Setting `img_en: 1` enables visual fusion (planned for M13).

**Terminal 3 — a consumer**, one of:
```bash
./launch_scanhub.sh                                                       # GUI (--start-stack also launches sensors + SLAM)
python3 scripts/run_m12_8_field_scan.py --session-id MyScan --duration 0  # 0 = stop with Ctrl+C
rviz2    # or: ros2 run rqt_image_view rqt_image_view
```

**All-in-one alternative** (sensors + SLAM + Foxglove bridge on port 8765):
```bash
ros2 launch scan_routine main.launch.py
```

**Health check**
```bash
ros2 topic hz /rslidar_points            # ~10 Hz
ros2 topic hz /rslidar_imu_data          # ~200 Hz
ros2 topic hz /camera/color/image_raw    # ~30 Hz
ros2 topic hz /aft_mapped_to_init        # SLAM pose
python3 scripts/m12_7_preflight_check.py # checks all of the above
```

> **ROS domain / DDS:** `launch_scanhub.sh` uses `$ROS_DOMAIN_ID` (default 0) and forces CycloneDDS; `.bashrc` sets neither, so other terminals use domain 0 with the default DDS. If the GUI receives nothing, export the same values in every terminal:
> `export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`

---

## 5. End-to-End Scan Workflows

### A. M12.9 Room Scan (newest, self-contained)

```bash
cd ~/scanarMini/diagnostics
python3 m12_9_room_scan_orchestrator.py                    # Ctrl+C to stop → auto-processes
python3 m12_9_room_scan_orchestrator.py --no-reconstruct   # stop without processing
```

1. Verifies Airy sync (aborts if it cannot), starts sensors, then SLAM, then `record_lidar.py` + `record_camera.py`.
2. Raw capture goes to `diagnostics/m12_9_room_dataset/` (`lidar/*.bin`, `depth/*.png`, `color/*.jpg`, `trajectory.tum`, `*_index.json`, `*_camera_info.json`).
3. On Ctrl+C, runs `process_m12_9_room_scan.py` → `diagnostics/m12_9_room_export/` (`airy_only.ply`, `gemini_only.ply`, `experimental_gated_fused.ply`, `raw_registered_overlay.ply`, `coverage_metadata.json`).
4. Subprocess logs go to `/tmp/m12_9_*.log`.

**Warning:** each run deletes the previous dataset and export folders. Copy them elsewhere first if you want to keep them.

Re-run offline steps only:
```bash
python3 diagnostics/process_m12_9_room_scan.py [dataset_dir] [export_dir]
python3 diagnostics/evaluate_motion_sanity.py [dataset_dir]
```

### B. M12.8 Field Scan (needs sensors + SLAM already running)

```bash
python3 scripts/run_m12_8_field_scan.py --session-id M12_8_Field_Scan_03 --duration 30 --countdown 5
python3 scripts/inspect_m12_8_pointclouds.py data/M12_8_Field_Scan_03
```

Output in `data/<session>/`: `pointcloud_{airy,stereo,fused_rgb}.{ply,pcd}`, `trajectory.csv`, `trajectory_tum.txt`, `flash_anchors.json`, `scan_metadata.json`. The GUI's Save button produces the same bundle.

---

## 6. Script Catalog by Layer

### Needs sensors + SLAM running

| Script | Args | Purpose |
|---|---|---|
| `scripts/m12_7_preflight_check.py` | — | Check all topic rates |
| `scripts/evaluate_m10_lio_slam.py` | `TEST_NAME SECONDS` (e.g. `M10.3_TRANS_5M 30`) | SLAM motion test → `data/m10_trajectories` |
| `scripts/run_m12_8_field_scan.py` | `--session-id --duration --countdown` | Full scan export |
| `scripts/m12_6_production_fusion_exporter.py` | `[sec] [name]` | M12.6 capture + fusion |
| `scripts/m12_7_dynamic_session_runner.py` | `[sec] [name]` | M12.7 motion capture |
| `scripts/m12_colorizer_node.py`, `record_and_verify_m12_3.py`, `validate_m12_projection.py` | — | M12.1–12.3 colorization |
| `scripts/benchmark_sensor_pipeline_rates.py` | `[sec]` | Topic rates → `data/sensor_rate_benchmark.json` |
| `src/scan_routine/scripts/lightweight_capture_recorder.py` | `[out_dir]` | Single-process recorder (alternative to record_lidar + record_camera) |

### Needs sensors only (no SLAM)

| Script | Args | Purpose |
|---|---|---|
| `scripts/test_m8_airy_qualification.py` | — | Airy LiDAR + IMU rates, gravity vector |
| `scripts/test_m8_4_gemini_benchmark.py` | `[sec] [name]` | Gemini stream benchmark |
| `scripts/test_m8_5_full_stack_throughput.py` | `SECONDS` | All streams concurrently |
| `scripts/diagnose_airy_cadence_and_sweep.py` | `[sec]` | LiDAR frame timing |
| `scripts/diagnose_depth_hardware_trigger.py` | `[sec]` | Is the Gemini triggered at 30 Hz? |
| `scripts/diagnose_individual_stream_rates.py` | — | Per-topic rates in isolation |
| `scripts/verify_m9_clock_alignment.py` | `[sec]` | LiDAR ↔ camera clock alignment |

### Self-launching (starts its own stack)

`diagnostics/m12_9_room_scan_orchestrator.py`, `test1_isolated_airy_fastlivo.py`, `test2_airy_fastlivo_gemini.py`, `test3_isolated_gemini_trigger.py`, `run_30s_transport_test.py`, `run_final_registration_test.py`, `query_orbbec_extrinsics.py`

### Direct hardware (no ROS)

| Script | Purpose |
|---|---|
| `src/scan_routine/scripts/verify_airy_sync.py` | Check/set Airy trigger to 120° |
| `scripts/configure_airy_sync_out.py [step_deg] [width_ns]` | Manually set Airy trigger (120 → 30 Hz) |
| `scripts/test_m8_1b_airy_mode_query.py` | Raw Airy UDP packet interrogation |

### Offline (reads saved files, no ROS)

| Script | Reads |
|---|---|
| `diagnostics/process_m12_9_room_scan.py`, `evaluate_motion_sanity.py` | `m12_9_room_dataset` |
| `scripts/inspect_m12_8_pointclouds.py [dir]` | M12.8 output |
| `scripts/visualize_m12_6_export.py`, `visualize_m12_7_dynamic_qualification.py [name]` | Produce PNG charts |
| `scripts/m12_5_*`, `analyze_m12_5_planes.py`, `evaluate_bias_models.py`, `test_m12_7_classifier_on_bench.py` | `data/M12_4_Characterization` |
| `scripts/analyze_stationary_drift.py`, `test_m12_7d_pose_interpolation.py` | M12.7 trajectories |
| `scripts/test_rcl_basis_vectors.py`, `test_m8_3_lidar_imu_transform.py`, `calculate_tf_angles.py` | Extrinsic math checks |
| `src/scanhub_gui/diagnostic_analyzer.py [path]` | GUI diagnostic folders |

### Legacy — will not work here

These reference `/home/scanar/scanR/...` and the IG-2: `diagnostics/m7_*`, `run_m7_3_trial.py`, `analyze_failure_window.py`, `analyze_session_2336.py`, `audit_upstream_packet_loss.py`, `classify_lidar_gaps.py`, `analyze_rosbag_truth.py`, `verify_hardware_sync.py`. `scripts/copy_*.py` were one-time migration tools.

Several plotting scripts save PNGs to `~/.gemini/antigravity-cli/...` (a previous AI tool's directory), not into the repo.

### C++ benchmarks

`scripts/CMakeLists.txt` builds into `scripts/build/`: `benchmark_lidar_subscriber`, `benchmark_dds_transport`, `benchmark_livo_consumer`, `benchmark_m9_sync`, `test_pointcloud_equivalence`. The cadence monitor is a ROS node: `ros2 run scan_routine cpp_lidar_cadence_monitor`.

---

## 7. Where to Change Things

| Want to change… | Edit |
|---|---|
| Camera resolution / FPS / laser / sync mode | `src/scan_routine/launch/sensor_bringup.launch.py` → `launch_arguments` |
| LiDAR IP / ports / RPM | `src/rslidar_sdk/config/drone_config.yaml` |
| SLAM tuning, VIO on/off (`img_en`), extrinsics used by SLAM | `src/FAST-LIVO2/config/robosense.yaml` |
| Sensor-to-sensor calibration (used by offline processing) | `config/production_extrinsics.yaml` |
| TF tree (RViz) | static TF nodes in `sensor_bringup.launch.py` |

---

## 8. Network & Hardware Reference

| Item | Value |
|---|---|
| Airy web UI / device IP | `192.168.1.200` |
| Host IP (Airy destination) | `192.168.1.102` |
| Airy ports | 6699 MSOP (points), 7788 DIFOP (device info), 6688 IMU |
| Airy NIC | `enP8p1s0` (PCIe Ethernet) |
| Airy serial | `030000003009` |
| Gemini serial | `CPC6463000XW` (USB 3.2 Gen 1) |

### Key ROS topics

| Topic | Source | Rate |
|---|---|---|
| `/rslidar_points` | Airy | 10 Hz |
| `/rslidar_imu_data` | Airy IMU | 200 Hz |
| `/camera/color/image_raw` | Gemini RGB 1280×800 | 30 Hz |
| `/camera/depth/image_raw` | Gemini depth 848×480 | 30 Hz |
| `/cam0/image_raw`, `/cam1/image_raw` | topic_relay (from IR; IR currently disabled) | — |
| `/aft_mapped_to_init` | FAST-LIVO2 pose (IMU frame in world) | ~10 Hz |
| `/cloud_registered` | FAST-LIVO2 world-frame cloud | ~10 Hz |

---

## 9. Known Issues (as of 2026-10-18)

1. **Fixed (2026-10):** GUI lock deadlock (now `threading.RLock`) and stale GUI extrinsics (GUI now loads `robosense.yaml` and `production_extrinsics.yaml`, with hardcoded scanarV4 fallbacks).
2. **Launcher calls a missing script:** `launch_scanhub.sh --start-stack` runs `scripts/configure_airy_ptp_sync.py`, which does not exist; the failure is hidden by `|| true`, so the Airy sync check is skipped.
3. **IMU frame convention mismatch:** `factory_transforms.T_airy_lidar_airy_imu` in `production_extrinsics.yaml` is ~120° rotated from the LiDAR ↔ IMU rotation used by FAST-LIVO2 `extrinsic_R` and the launch TF. `T_cam0_airy_imu` follows the factory frame, not the `/rslidar_imu_data` frame. Matters once VIO (`img_en: 1`) is enabled.
4. **Safety aborts not wired:** ADR-006 (storage) and ADR-007 (sensor failure) are not implemented in the current GUI; `StorageMonitor`/`SensorMonitor` are imported but unused.
5. **Minor:** the orchestrator passes `use_hardware_trigger:=true`, which `sensor_bringup.launch.py` does not declare (silently ignored). The latest room scan's surface classification reports 99.47% "other", which is suspicious.
6. **Portability:** absolute paths to `/home/scanar/scanarMini` are hardcoded throughout.
