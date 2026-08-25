# scanR Performance Benchmarking Record

This permanent document tracks quantitative performance metrics for `scanR` across every roadmap milestone. It provides an objective engineering record to ensure every milestone demonstrates measurable progress toward matching or exceeding **NavVis VLX** and **FARO Orbis** reality capture benchmarks.

---

## Formalized Performance Tiers & Target Standards

| Metric Category | v1.0 RC Target (75% Gate) | v1.0 Commercial Release Target | v2.x Long-Term Target (Competitor Equal/Exceed) |
| :--- | :--- | :--- | :--- |
| **Relative Precision** | $< 2.0 \text{ cm}$ local planar residual | $< 1.5 \text{ cm}$ local planar residual | $< 0.8 \text{ cm}$ local planar residual |
| **Absolute Accuracy** | $< 5.0 \text{ cm}$ (with RTK / GCP) | $< 3.0 \text{ cm}$ (with RTK / GCP) | $< 1.5 \text{ cm}$ (with RTK / GCP) |
| **Localization Drift** | $< 0.20\%$ of total path distance | $< 0.15\%$ of total path distance | $< 0.08\%$ of total path distance |
| **Repeatability** | $< 2.5 \text{ cm}$ (same path 3x) | $< 1.5 \text{ cm}$ (same path 5x) | $< 0.8 \text{ cm}$ (same path 10x) |
| **Loop-Closure Accuracy** | $< 3.0 \text{ cm}$ on 100m loop | $< 2.0 \text{ cm}$ on 100m loop | $< 1.0 \text{ cm}$ on 100m loop |
| **LiDAR Point Rate** | 20 Hz ($80,000+$ pts/sec) | 20 Hz ($80,000+$ pts/sec) | 20 Hz ($100,000+$ pts/sec) |
| **Camera Frame Rate** | 30 FPS RGB ($640 \times 480$) | 30 FPS RGB ($640 \times 480$) | 30 FPS 4K Panoramic / RGB |
| **SLAM Update Rate** | $\ge 20 \text{ Hz}$ on `/aft_mapped_to_init` | $\ge 20 \text{ Hz}$ on `/aft_mapped_to_init` | $\ge 20 \text{ Hz}$ on `/aft_mapped_to_init` |
| **Time Sync Skew** | $< 2.0 \text{ ms}$ (Driver software) | $< 1.0 \text{ ms}$ (Hardware PPS) | $< 0.1 \text{ ms}$ (Hardware PPS) |
| **Capture Speed** | $150 \text{ m}^2 / \text{min}$ walking speed | $200 \text{ m}^2 / \text{min}$ walking speed | $300+ \text{ m}^2 / \text{min}$ walking speed |
| **Startup-to-Ready Time** | $< 20 \text{ seconds}$ ZUPT ritual | $< 15 \text{ seconds}$ ZUPT ritual | $< 10 \text{ seconds}$ ZUPT ritual |
| **Battery Runtime** | $> 1.5 \text{ hours}$ continuous | $> 2.5 \text{ hours}$ continuous | $> 4.0 \text{ hours}$ (hot-swappable) |
| **Max Sustained Temp** | $< 65^\circ\text{C}$ Jetson ambient | $< 60^\circ\text{C}$ Jetson ambient | $< 55^\circ\text{C}$ Jetson ambient |
| **CPU / GPU / RAM** | $< 80\%$ CPU, $< 70\%$ RAM | $< 70\%$ CPU, $< 60\%$ RAM | $< 50\%$ CPU, $< 50\%$ RAM |
| **SSD Write Speed** | $> 50 \text{ MB/sec}$ sustained | $> 100 \text{ MB/sec}$ sustained | $> 200 \text{ MB/sec}$ sustained |
| **Dataset Size / Min** | $\approx 250 \text{ MB / minute}$ | $\approx 200 \text{ MB / minute}$ | $\approx 150 \text{ MB / minute}$ (compressed) |
| **Export Duration** | $< 0.5 \times$ capture duration | $< 0.25 \times$ capture duration | $< 0.1 \times$ capture duration |
| **System Stability** | Zero crashes / 4 hours test | Zero crashes / 8 hours test | Zero crashes / 24 hours test |

---

## Milestone Performance Tracking Log

### Milestone 3 — FAST-LIVO2 Integration & Sensor Synchronization
* **Date:** August 3, 2026
* **Hardware Profile:** Seeed J4012 + Jetson Orin NX 16GB, RoboSense Airy (45° pitch mount), Orbbec Gemini 336L, InertialSense IG-2 IMU.
* **Empirical Measurements:**
  * **SLAM Output Topic:** `/aft_mapped_to_init` streaming at **20.21 Hz** ($20.2\text{ Hz}$ average, std dev $0.027\text{s}$).
  * **LIO Residual:** $0.017\text{ m}$ ($1.7\text{ cm}$ average residual).
  * **VIO Sparse Tracking:** Active with 10 visual map points per keyframe.
  * **Sensor Time Sync Skew:** Synchronized to system ROS time domain ($< 2.0\text{ ms}$ inter-sensor offset).
  * **TF Tree:** Verified single-chain hierarchy `camera_init` $\to$ `aft_mapped` $\to$ `base_link` $\to$ {`imu_link`, `rslidar`, `camera_link`}.

---

### Milestone 4 — First Real Capture Validation
* **Date:** In Progress
* **Test Environment:** Indoor office corridor loop ($15 \text{m}$ path).
* **Metrics to Record During Operator Walkthrough:**
  * **30-Minute Continuous Capture:** *Pending operator walk*
  * **Localization Drift (% of path):** *Pending operator walk*
  * **Trajectory Continuity & Motion:** *Pending operator walk*
  * **FARO Anchor Positional Separation ($w00, w01, w02$):** *Pending operator walk*
  * **Map Accumulated Noise / Double Walls:** *Pending operator walk*
  * **SSD Write Rate & Dataset Size:** *Pending operator walk*
  * **Export Duration & File Integrity:** *Pending operator walk*
  * **Overlay Quality vs Competitor Dataset (% match):** *Target: $\ge 75\%$*

---

### Milestone 5 — System Health & Diagnostics Panel
* **Target Date:** Upcoming
* **Metrics to Track:**
  * **LiDAR Hz:** Target $20.0\text{ Hz}$
  * **Camera FPS:** Target $30.0\text{ FPS}$
  * **IMU Hz:** Target $200.0\text{ Hz}$
  * **SLAM Hz:** Target $20.0\text{ Hz}$
  * **Time Sync Skew (Camera/LiDAR/IMU):** Target $< 1.0\text{ ms}$
  * **Resource Usage:** CPU %, GPU %, RAM %, NVMe write MB/s, Temperature °C.

---

### Milestone 6 — Hardware Time Synchronization (PPS & VSYNC)
* **Target Date:** Upcoming
* **Metrics to Track:**
  * **Hardware PPS Sync Delta:** Target $< 0.1\text{ ms}$
  * **Timestamp Drift Rate:** Target $0.00\text{ ms/hour}$

---

### Milestone 7 — Production Calibration
* **Target Date:** Upcoming
* **Metrics to Track:**
  * **Camera Reprojection Error:** Target $< 0.5\text{ pixels}$
  * **LiDAR-IMU Extrinsic Error:** Target $< 0.001\text{ m}$ translation, $< 0.05^\circ$ rotation
  * **Camera-LiDAR Extrinsic Error:** Target $< 0.002\text{ m}$ translation, $< 0.10^\circ$ rotation

---

### Milestone 8 — SLAM Performance & Drift Validation
* **Target Date:** Upcoming
* **Metrics to Track:**
  * **Planar Residual (Relative Precision):** Target $< 1.5\text{ cm}$
  * **Loop Closure Error:** Target $< 2.0\text{ cm}$ on $100\text{m}$ closed loop
  * **Repeatability:** Target $< 1.5\text{ cm}$ variation over 5 repeated walks
  * **ZUPT Bias Convergence Time:** Target $< 10\text{ seconds}$

---

### Milestone 9–24 — Future Milestones
*(Performance metrics will be updated sequentially as each milestone is executed on physical hardware.)*
