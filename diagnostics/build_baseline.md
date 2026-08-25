# Software & Codebase Build Baseline Report

**Project:** scanR / scanHUB  
**Environment:** ROS 2 Humble / Ubuntu 22.04 LTS / Python 3.10  
**Report Generated:** 2026-07-29  

---

## Discovered Software Assets & Repositories

### 1. ROS 2 Core Workspaces & Source Nodes
- **`scan_ar` (`/home/scanar/scan_ar/src`):** Primary workspace containing:
  - `FAST-LIVO2`: Core LiDAR-Inertial-Visual SLAM backend
  - `rslidar_sdk` / `rslidar_msg`: RoboSense Airy driver and custom message protocols
  - `inertial-sense-sdk`: Hardware SDK for InertialSense IG-2 GNSS/INS module
  - `scan_routine` & `scan_description`: System URDFs, transformation frames, and startup routines
  - `calibration`: Extrinsic matrix configurations for camera, LiDAR, and IMU
- **`dev2_ws` (`/home/scanar/dev2_ws/src`):** Secondary development workspace containing:
  - `inertial_sense_ros`: ROS 2 wrapper node for IG-2 module
  - `reality_filter`: Real-time point cloud noise and voxel downsampling filters
  - `rslidar_sdk` & `rslidar_msg`

### 2. Standalone Operations & GUI Scripts
- **`navvis_gui.py` (`/home/scanar/navvis_gui.py`):** Commercial PyQt GUI controller implementing state machine workflow, dataset creation, and live process monitoring.
- **`sensor_health_chek.py` (`/home/scanar/sensor_health_chek.py`):** Real-time ROS 2 diagnostic monitor tracking topic rates and stale sensor data.
- **`calc_tf.py` (`/home/scanar/calc_tf.py`):** Transformation frame calculator for multi-sensor spatial extrinsics.
- **`device_2_ig2.yaml` / `device_2_elp.yaml`:** Hardware calibration definitions for Device 2.
- **`zupt_smoothing_node.cpp` / `zupt_filter.yaml`:** Zero Velocity Update filter for NavVis-style static initialization routines.

---

## Consolidated scanHUB Architecture

The scanHUB platform consolidates these components into a single unified directory structure under `/home/scanar/scanR/`:

```
scanR/
├── docs/
│   ├── milestone_definition_of_done.md
│   ├── project_roadmap.md
│   └── validation_checklists/
├── engineering/
│   ├── hardware_inventory.md
│   ├── build_baseline.md
│   └── calibration_notes.md
├── milestones/
│   ├── M0_discovery.md
│   ├── M1_scanhub_restoration.md
│   ├── M2_hardware_bringup.md
│   └── ...
└── src/
    ├── scanhub_gui/
    ├── scanr_drivers/
    ├── scanr_slam/
    └── scanr_bringup/
```
