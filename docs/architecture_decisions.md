# scanR Architecture Decision Records (ADR) & Authoritative System Mapping

This document records the foundational architectural decisions, hardware selections, and system governance rules for the `scanR` handheld reality capture system. These decisions preserve project integrity, prevent scope creep, and ensure long-term maintainability.

---

## Authoritative System Sources (Governance Mapping)

To eliminate ambiguity across all subsystems and UI components:

| Data Type | Authoritative Source Node / Hardware | ROS 2 Topic | Rule / Restriction |
| :--- | :--- | :--- | :--- |
| **Pose & Trajectory** | `FAST-LIVO2` SLAM Engine | `/aft_mapped_to_init` / `/LIVO2/imu_propagate` | `scanHUB` must NEVER estimate pose independently. |
| **IMU Motion & Orientation**| `InertialSense IG-2` Dual RTK | `/imu` | Direct hardware IMU measurement. |
| **GNSS & RTK Lock** | `InertialSense IG-2` Dual RTK | `/gps1/pos_vel` / `/NavSatFix` | Authoritative geographic coordinate & RTK status. |
| **3D Point Cloud Ranging** | `RoboSense Airy` 3D LiDAR | `/rslidar_points` | Primary spatial ranging data source. |
| **RGB Imagery** | `Orbbec Gemini 336L` Camera | `/image_raw` | Primary visual texturing source. |
| **Stereo Depth (Forward-Compatible Interface)** | `Orbbec Gemini 336L` Camera | `/camera/depth/image_raw` / `/camera/depth/camera_info` | Reserved interface for v2.x depth fusion. |
| **Operator UI & HUD** | `scanHUB` Qt/C++ Application | N/A | **Visualization and operator interface only.** |

---

## Official Configuration Hierarchy

`scanR` implements a 3-tier configuration hierarchy to prevent experimental features from affecting production stability:

1. **Production (Default):**
   * The only supported customer configuration.
   * Hardened SLAM parameters, strict error checking, clean HUD overlay, automatic safety stops enabled.
2. **Engineering:**
   * Diagnostic features enabled (`scanHUB` Diagnostics Panel, verbose SLAM logs, time sync skew graphs, raw point cloud debug tools).
3. **Experimental:**
   * Sandbox mode for testing new algorithms, research features, and post-MVP pipelines (e.g., stereo depth fusion, C-SLAM keyframe streaming).

---

## Permanent Scope & Risk Governance Rules

1. **Rule 1 (Scope Creep Gate):** If a proposed feature or task does not directly increase the probability of shipping Commercial Release v1.0, it belongs in the Post-MVP backlog.
2. **Rule 2 (Defect / Risk Rule):** If a proposed change cannot be demonstrated to reduce risk, fix a known defect, or directly advance the current milestone's Definition of Done, it is deferred to the Post-MVP backlog.
3. **Rule 3 (Shippable Milestone Gate):** Before any milestone is declared complete, it must build cleanly from a fresh checkout on Seeed J4012 hardware (JetPack 6 + ROS 2 Humble), run cleanly on physical hardware, and be committed/pushed to the git repository.
4. **Rule 4 (One Authoritative Source Per Subsystem):** Every subsystem has exactly one authoritative publisher or owner. Redundant estimation, duplicate transforms, or competing publishers are strictly prohibited in production.

---

## Architectural Decision Records (ADRs)

### ADR-001: Selection of FAST-LIVO2 over FAST-LIO / LIO-SAM
* **Context:** Need a tightly-coupled LiDAR-Visual-Inertial SLAM engine capable of real-time execution on NVIDIA Jetson Orin NX (16GB).
* **Decision:** Selected `FAST-LIVO2` for its tightly-coupled visual-inertial-lidar fusion, direct patch-based visual tracking, and sparse voxel map structure.
* **Rationale:** Pure LiDAR SLAM (FAST-LIO) suffers from degeneracy in long featureless corridors or open spaces. Tightly-coupled VIO constraints from the Gemini 336L camera eliminate corridor degeneracy while maintaining real-time performance ($20\text{ Hz}$).

### ADR-002: 45° Sky Pitch Mount for RoboSense Airy LiDAR
* **Context:** Handheld/wearable Reality capture requires maximum vertical field of view for floor, ceiling, and wall coverage.
* **Decision:** Mechanically mounted the RoboSense Airy LiDAR at a $45^\circ$ skyward pitch tilt relative to horizontal (`base_link`).
* **Rationale:** A flat horizontal mount causes significant ground/ceiling blind spots directly above and below the operator. The 45° tilt directs the Airy dome to capture floor, walls, and ceiling simultaneously in a single forward walk.

### ADR-003: InertialSense IG-2 IMU Frame as `base_link` Center
* **Context:** Need a primary coordinate system origin for the physical device.
* **Decision:** Designated the physical center of the InertialSense IG-2 IMU as the system origin (`base_link`).
* **Rationale:** High-rate IMU angular velocity and linear acceleration are measured at the IG-2. Setting `base_link` at the IG-2 simplifies spatial transforms ($T_{\text{IMU}}^{\text{LiDAR}}$, $T_{\text{IMU}}^{\text{Camera}}$) and rigid-body kinematic propagation.

### ADR-004: Offline-First Appliance Architecture
* **Context:** Field reality capture environments (tunnels, construction basements, remote sites) frequently lack internet or cell connectivity.
* **Decision:** `scanR` operates as a 100% offline-first appliance. All SLAM processing, data storage, point cloud colorization, and export execute locally on the Jetson Orin NX NVMe storage.
* **Rationale:** Eliminates reliance on cloud processing, ensures zero data loss in disconnected environments, and provides immediate local dataset inspection.

### ADR-005: Visualization-Only GUI Rule for `scanHUB`
* **Context:** Risk of GUI code estimating or synthesizing robot motion when SLAM pose updates fluctuate or lag.
* **Decision:** `scanHUB` is strictly prohibited from estimating, integrating, inferring, or synthesizing pose, heading, odometry, or trajectory.
* **Rationale:** GUI-level motion synthesis introduces drift discrepancies between the displayed map and the exported SLAM dataset. Requiring all UI elements to consume `/aft_mapped_to_init` guarantees the operator sees the exact state of the SLAM engine.

### ADR-006: Automatic Capture Abort at 10% Free Storage
* **Context:** Unhandled NVMe storage exhaustion causes corrupted SLAM log files, truncated point clouds, and lost capture sessions.
* **Decision:** `scanHUB` automatically halts capture, flushes all pending file writes, and saves project metadata when free storage drops to $\le 10\%$.
* **Rationale:** Protects data integrity, ensures current session files are finalized cleanly, and prevents filesystem lockups.

### ADR-007: Controlled Sensor Failure Abort
* **Context:** If a sensor fails mid-capture (e.g., USB cable snag or Ethernet disconnect), continued scanning produces invalid or un-colorized point clouds.
* **Decision:** If any required sensor fails or stops streaming data during capture, `scanHUB` instantly halts capture, saves project state cleanly, and displays a blocking alert identifying the failed sensor link.
* **Rationale:** Prevents operators from unknowingly walking for miles with dead sensors, protecting session quality.

### ADR-008: Forward-Compatible Modular Stereo Depth Architecture
* **Context:** Gemini 336L hardware supports stereo depth estimation, but complex hybrid fusion algorithms are deferred to v2.x to prevent v1.0 MVP scope creep.
* **Decision:** Standardized modular ROS 2 topics (`/camera/depth/image_raw` and `/camera/depth/camera_info`) and TF camera depth optical frames in the v1.0 architecture.
* **Rationale:** Allows Gemini 336L stereo depth streams to be published without breaking v1.0 SLAM, enabling seamless drop-in fusion in v2.x without major architectural refactoring.
