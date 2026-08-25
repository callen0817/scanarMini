# scanR Master Engineering Roadmap (Frozen v1.0)

## Product Vision & Performance Tiers

`scanR` is a commercial handheld wearable reality capture system built around the Seeed J4012 + Jetson Orin NX 16GB, designed to directly compete with—and ultimately exceed—the capabilities of NavVis VLX and FARO Orbis.

The device boots directly into `scanHUB`, the dedicated reality capture operating system. The Jetson compute unit is an implementation detail; the operator experiences `scanR` as a unified commercial hardware appliance.

### Formalized Performance Tiers
* **Production Candidate (v1.0 RC):** Achieve at least **75%** of NavVis VLX / FARO Orbis performance according to defined benchmark metrics.
* **Commercial Release (v1.0):** Stable, field-ready commercial appliance with zero critical issues, single-script installer, and full disaster recovery documentation.
* **Future Target (v2.x):** Match or exceed NavVis VLX and FARO Orbis performance across all accuracy, point density, noise, and workflow metrics.

### Permanent Scope & Risk Governance Rules
> **RULE 1 (Scope Creep Gate):** If a proposed feature or task does not directly increase the probability of shipping Commercial Release v1.0, it belongs in the Post-MVP backlog.

> **RULE 2 (Defect / Risk Rule):** If a proposed change cannot be demonstrated to reduce risk, fix a known defect, or directly advance the current milestone's Definition of Done, it is deferred to the Post-MVP backlog.

> **RULE 3 (Shippable Milestone Rule):** Every milestone must build cleanly from a fresh checkout on Seeed J4012 hardware (JetPack 6 + ROS 2 Humble), run cleanly on physical hardware, and be committed/pushed to the `scanR` repository before starting the next milestone.

> **RULE 4 (One Authoritative Source Per Subsystem):** Every subsystem has exactly one authoritative publisher or owner. Redundant estimation, duplicate transforms, or competing publishers are strictly prohibited in production.

---

## Governing Engineering Rules (Permanent)

### 1. Reality First
* No simulated sensor streams.
* No mocked SLAM.
* No synthetic point clouds.
* No "headless verification."
* Every feature must be visually and empirically verified on physical `scanR` hardware.

### 2. Visualization-Only GUI Rule
`scanHUB` is a visualization layer only. It must never estimate, integrate, infer, or synthesize:
* position
* heading
* odometry
* trajectory

All spatial coordinates and trajectory points originate exclusively from the authoritative FAST-LIVO2 pose estimate on ROS 2 topics (`/aft_mapped_to_init` and `/LIVO2/imu_propagate`).

### 3. Hardware Detection & Failure Protection
Every sensor must be physically detected before capture initiation. If any required sensor fails or disconnects during capture:
* automatically stop capture
* safely flush and save project metadata
* display a blocking error dialog identifying the failed hardware link

### 4. Storage & Thermal Protection
`scanHUB` continuously monitors free storage and system thermals. At **10% remaining NVMe storage** or thermal limit:
* automatically stop capture
* safely save project files
* notify the operator with a high-priority alert

---

## Commercial MVP Roadmap (Milestones 0 – 17)

### Milestone 0: Project Foundation
* **Status:** ✅ Complete
* **Deliverables:** `scanHUB` GUI architecture, `scanR` single-hardware profile, self-contained workspace (`/home/scanar/scanR`), repository organization, colcon build system.

### Milestone 1: Hardware Integration
* **Status:** ✅ Complete
* **Deliverables:** RoboSense Airy 3D LiDAR, Orbbec Gemini 336L RGB-D camera, InertialSense IG-2 Dual RTK IMU, VITURE XR Glasses HUD, storage monitoring, RTK detection, hardware driver integration.

### Milestone 2: Sensor Visualization
* **Status:** ✅ Complete
* **Deliverables:** Live RGB preview (`/image_raw`), live 2D LiDAR slice preview (`/rslidar_points`), live IMU heading, RTK status badge, AR HUD overlay, FARO Flash anchor interface, live NVMe storage display.

### Milestone 3: FAST-LIVO2 SLAM Integration & Forward-Compatible Architecture
* **Status:** ✅ Complete
* **Deliverables:** FAST-LIVO2 SLAM engine integrated and compiled, CAD extrinsics configured, Airy 45° mounting transformation, ROS 2 launch system (`mapping_scanR.launch.py`), SLAM startup pipeline, C++ driver timestamp bug fixed (`inertial_sense_ros2`), LiDAR system clock fix (`rslidar_sdk`), verified **20 Hz** authoritative SLAM pose stream on `/aft_mapped_to_init`, single-chain TF tree verified (`camera_init` $\to$ `aft_mapped` $\to$ `base_link` $\to$ {`imu_link`, `rslidar`, `camera_link`}), GUI converted to visualization-only layer.
* **Forward-Compatible Interface:** Reserved ROS 2 depth topics (`/camera/depth/image_raw` / `/camera/depth/camera_info`) and TF depth optical frames in the v1.0 pipeline architecture to ensure future v2.x depth fusion plugs in without refactoring.

---

### Milestone 4: First Real Capture Validation
* **Status:** ✅ Core Sync & Tracking Verified (Pending M4.5 Diagnostic Walk)
* **Pass/Fail Criteria:**
  1. *Arrow Following:* Verify arrow follows pose and walking direction cleanly without 180° offset.
  2. *Trajectory Line:* Verify cyan trajectory line draws continuously without jumps or snapping to origin.
  3. *FARO Anchors:* Verify FARO Flash Anchors (`w00`, `w01`, `w02`) are placed at distinct physical locations using SLAM pose.
  4. *World-Frame Map:* Verify accumulated point cloud remains fixed in world coordinates while scanner moves.
  5. *Export Validation:* Verify exported dataset opens cleanly in point cloud viewer.
  6. *Complete Office Capture:* Complete a full office capture session.

---

### Milestone 4.5: Diagnostic Instrumentation & Truth Validation (CURRENT)
* **Status:** 🔄 Active / Instrumentation Implemented
* **Deliverables:**
  1. *GUI State Machine Fixes:* Resolved FARO Flash Scan button and Stop Scan button state machine bugs.
  2. *One-Click Diagnostic Capture Mode:* One-button toggle in GUI to record synchronized telemetry CSVs (`imu.csv`, `lidar.csv`, `camera.csv`, `slam.csv`, `tf.csv`, `gui.csv`, `system.csv`).
  3. *Automated ROS 2 Bag Recording:* Automatically spawns `ros2 bag record` during diagnostic validation walks.
  4. *Automated Post-Run Diagnostic Summary:* Generates `summary.txt` report with timing jitter, point cloud density, and motion smoothness metrics.
  5. *Standalone Diagnostic Analyzer Tool:* [diagnostic_analyzer.py](file:///home/scanar/scanR/src/scanhub_gui/diagnostic_analyzer.py) to analyze walk directories, detect timestamp gaps, point count drops, and velocity spikes.
* **Pass/Fail Criteria:** Diagnostic mode cleanly records synchronized telemetry and ROS bag during physical walk test, generating actionable summary metrics.

---

### Milestone 5: System Diagnostics
* **Purpose:** Visibility-only diagnostic overlay for hardware & SLAM debugging.
* **Pass/Fail Criteria:** Real-time visibility for LiDAR Hz, Camera FPS, IMU Hz, SLAM Hz, CPU %, GPU %, RAM %, NVMe storage MB/s, Sensor status, Timestamp skew, and dropped frames. Zero performance optimization overhead.

### Milestone 6: Hardware Time Synchronization
* **Implementation:** Production hardware PPS & VSYNC synchronization (Airy ↔ IG-2, Gemini 336L ↔ IG-2).
* **Pass/Fail Criteria:** Inter-sensor time skew $< 1.0\text{ ms}$, hardware PPS lock verified across Airy, Gemini, and IG-2 with zero dropped pulses.

### Milestone 7: Production Calibration
* **Calibration Pipeline:** FAST-Calib and Kalibr for camera intrinsics, camera $\leftrightarrow$ IMU extrinsics, LiDAR $\leftrightarrow$ camera extrinsics, LiDAR $\leftrightarrow$ IMU extrinsics.
* **Pass/Fail Criteria:** Camera reprojection error $< 0.5\text{ px}$, LiDAR-IMU translation error $< 1\text{ mm}$, rotation error $< 0.05^\circ$, automated calibration report output.

### Milestone 8: Golden Dataset Validation (Regression Milestone)
* **Regression Test Suite:** Official regression milestone running fixed environments (`GD-01` through `GD-09`: office, hallway, warehouse, outdoor campus).
* **Pass/Fail Criteria:** Quantitative measurement of drift ($< 0.15\%$), accuracy, and repeatability across all Golden Environments.

### Milestone 9: Indoor Production Validation
* **Validation Testing:** Multiple long indoor scans, complex room layouts, active stress testing, loop closures.
* **Pass/Fail Criteria:** Complete $500+\text{ m}^2$ indoor capture sessions with zero tracking loss and clean loop closures ($< 2.0\text{ cm}$ loop error).

### Milestone 10: Outdoor Validation
* **Validation Scenarios:** Outdoor scanning, GPS, RTK GNSS recovery, indoor $\leftrightarrow$ outdoor transitions.
* **Pass/Fail Criteria:** Smooth RTK reacquisition within $< 5\text{ s}$ of outdoor transition, zero trajectory teleportation.

### Milestone 11: Commercial Capture Workflow
* **Operator Experience:** Polished operator UX, coverage heatmap, missed-area highlighting, project recovery after unexpected power cycle, dataset management interface.
* **Pass/Fail Criteria:** Successful operator-guided scan resumption after intentional pause and power cycle.

### Milestone 12: Export Pipeline
* **Export Formats:** E57, PLY, LAS, structured metadata JSON, panoramic imagery.
* **Pass/Fail Criteria:** Verified schema-compliant import into CloudCompare, Revit, and Leica Cyclone with zero errors.

### Milestone 13: Performance Benchmarking
* **Data Gathering & Analysis:** Measure drift, accuracy, point density, noise level, system runtime, CPU/GPU/RAM usage, and NVMe write rates across test sessions. Compare against past software builds and NavVis VLX / FARO Orbis reference datasets.
* **Pass/Fail Criteria:** Complete quantitative performance report generated and logged in `docs/performance_benchmarking.md`.

### Milestone 14: Production Candidate (v1.0 RC)
* **Release Gate Decision:** Decision gate evaluating benchmark data from M13. All critical software bugs resolved, feature complete, benchmark target achieved ($\ge 75\%$ NavVis VLX / FARO Orbis capability), ready for long-duration field soak testing.
* **Pass/Fail Criteria:** 8-hour continuous soak test with zero crashes, release candidate candidate tag generated.

### Milestone 15: Hardware Qualification
* **Hardware Robustness Criteria:**
  1. *Power Failure:* Sudden power loss causes zero file corruption; auto-recovers project on reboot.
  2. *Ethernet Disconnect:* System detects unplugged LiDAR, pauses cleanly, and resumes upon reconnect.
  3. *USB Disconnect:* System detects unplugged IMU/Camera, pauses cleanly, and resumes upon reconnect.
  4. *SSD Nearly Full:* System safely auto-stops at 10% NVMe storage without corrupting project.
  5. *Thermals:* System gracefully handles high ambient temperatures ($> 45^\circ\text{C}$).

### Milestone 16: One-Command System Appliance Installer
* **Appliance Deployment:** Single-command installer script (`install_scanR.sh`) for clean Seeed J4012 running Ubuntu 22.04 + JetPack 6 + ROS 2 Humble.
* **Pass/Fail Criteria:** Fresh Jetson flashed with stock JetPack 6 builds into a fully working `scanHUB` appliance in $< 15\text{ minutes}$ via a single command, automatically configuring udev rules, network interfaces, systemd auto-start services, and environment permissions.

### Milestone 17: Commercial Release v1.0
* **Definition of Done:**
  1. Complete user documentation, installation guide, and disaster recovery guide.
  2. Release candidate accepted with zero known critical bugs.
  3. `scanR` v1.0 commercial appliance ships.

---

## Post-MVP Innovation Roadmap (v2.x — Deferred)

These research and innovation features are strictly deferred until Commercial Release v1.0 ships. The v1.0 architecture is forward-compatible with these pipelines:

* **v2.1: Stereo Depth Integration:** Gemini 336L stereo depth stream integration to improve near-field density, thin-object reconstruction, and texture alignment (utilizing forward-compatible v1.0 ROS interfaces).
* **v2.2: Hybrid LiDAR + Stereo Mapping:** Depth confidence weighting, adaptive fusion, occlusion recovery.
* **v2.3: Collaborative SLAM (C-SLAM):** Multiple `scanR` units sharing local maps, streaming keyframes, live teammate rendering.
* **v2.4: Multi-Session Registration:** Automatic revisit alignment, permanent site maps.
* **v2.5: Semantic Reconstruction:** Automatic classification of walls, doors, windows, furniture, and objects.
* **v2.6: Enterprise Features:** Optional cloud synchronization, fleet management, remote cloud processing.
