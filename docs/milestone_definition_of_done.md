# scanR Project: Definition of Done (DoD) & Governing Product Charter

## Project Charter Summary
* **Product Name:** scanR
* **Software Platform:** scanHUB
* **Mission:** Build a commercially deployable wearable/mobile reality capture device for indoor and outdoor scanning using LiDAR-Visual-Inertial SLAM. This is a production product, not a research prototype or technology demonstration.
* **Benchmark:** Match or exceed the operational experience and final deliverable quality of NavVis VLX and FARO Orbis. MVP targets at least 75% of benchmark operational capability.

---

## Governing Milestone Definition of Done (DoD)

A milestone is complete **ONLY** when the feature has been:

1. **Built successfully**
2. **Demonstrated on the actual scanR hardware**
3. **Verified visually by you (the operator)**
4. **Documented**
5. **Committed to the scanR repository**
6. **Confirmed not to have regressed previously completed functionality**

---

## Permanent Engineering Rules

1. **Reality First (No Simulated Data):** All development, testing, and validation must run against real physical sensors or raw hardware data streams. No simulated sensors, mocked SLAM, synthetic point clouds, or headless acceptance.
2. **Visualization-Only GUI Rule:** `scanHUB` is a visualization layer only. It must never estimate, integrate, infer, or synthesize robot position, heading, odometry, or trajectory when a SLAM pose exists. All position, orientation, trajectory, and anchor placement must originate from the authoritative FAST-LIVO2 pose estimate.
3. **Hardware & SLAM Frozen for MVP:** 
   * **LiDAR:** RoboSense Airy 3D LiDAR (45° pitch tilt mount)
   * **Stereo Vision:** Orbbec Gemini 336L (Global Shutter, IR-Pass)
   * **IMU / GNSS:** InertialSense IG-2 Dual RTK
   * **Compute:** Seeed J4012 / Jetson Orin NX 16GB
   * **Time Sync:** Hardware PPS / VSYNC hardwired trigger loop (IG-2 Strobe → Airy + Gemini VSYNC_IN)
   * **HUD / Display:** VITURE XR Display Glasses / Herelink V1.1 Controller Screen
   * **SLAM Fusion Engine:** Fast-LIVO2
4. **Hardware Detection & Controlled Sensor Failure Stop:** Every sensor must be physically detected before use. If any required sensor fails or disconnects during capture, `scanHUB` must instantly stop acquisition, save all buffered project data cleanly to disk, finalize metadata, and present a clear blocking popup error dialog identifying the failed sensor.
5. **Storage Protection Threshold:** `scanHUB` must continuously display remaining storage. When free storage reaches **10%**, `scanHUB` must automatically and safely stop recording, flush all pending writes, save project metadata, and alert the user.
6. **Competitive Point Cloud Validation:** Release candidates are verified by overlaying exported colorized point clouds directly against benchmark datasets (NavVis VLX / FARO Orbis).
7. **Single Implementation Branch:** One active, verified branch with incremental milestone validation and complete documentation deliverables.

---

## Milestone 4.5 Definition of Done (DoD)

1. **State Machine Integrity:** `FARO Flash Anchor` and `Stop Scan` UI buttons respond exclusively to valid operational states (`recording == true` and `FAST-LIVO2 pose available`).
2. **Diagnostic Capture Mode:** `scanHUB` records synchronized telemetry CSVs (`imu.csv`, `lidar.csv`, `camera.csv`, `slam.csv`, `tf.csv`, `gui.csv`, `system.csv`) and launches an automated background ROS bag recording (`diagnostic_bag`).
3. **Automated Report Generation:** `scanHUB` generates a `summary.txt` report summarizing sensor message rates, inter-frame timestamp jitter, point count distributions, and pose velocity smoothness.
4. **Standalone Analyzer Tool:** [diagnostic_analyzer.py](file:///home/scanar/scanR/src/scanhub_gui/diagnostic_analyzer.py) compiles cleanly and parses any diagnostic walk directory to provide quantitative pass/fail timing and drift metrics.
