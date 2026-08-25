# scanR Intentional Design Limitations (v1.0 Commercial Release)

This document records the intentional design constraints and boundaries for `scanR` Commercial Release v1.0. These limitations represent deliberate architectural choices to ensure software stability, hardware reliability, and commercial field readiness without scope creep.

---

## Intentional System Boundaries (v1.0)

| System Domain | Intentional v1.0 Constraint | Rationale / Future Post-MVP Roadmap |
| :--- | :--- | :--- |
| **LiDAR Ranging** | Single RoboSense Airy 3D LiDAR only. | Multi-LiDAR arrays increase compute load and hardware cost. Single 45° tilted Airy provides sufficient vertical coverage for v1.0. |
| **Stereo Depth Fusion** | Interfaces reserved (`/camera/depth/*`); active algorithm fusion deferred to v2.x. | Avoids real-time compute overload on Orin NX during v1.0 MVP; interface is forward-compatible for v2.1. |
| **Operator Model** | Single operator per capture session; no collaborative SLAM (C-SLAM). | Multi-operator map sharing and distributed pose graph merging deferred to v2.3. |
| **Connectivity** | 100% Offline-First appliance operation; zero cloud dependencies. | Guarantees reliability in subterranean, basement, and remote field sites with no network signal. Cloud backup deferred to v2.6. |
| **Semantic AI** | Pure spatial point cloud colorization; no automated AI BIM classification. | Semantic wall/door/window classification deferred to v2.5. |
| **UI Motion Estimation** | `scanHUB` is a visualization layer only; zero independent motion integration. | Prevents drift discrepancies between displayed UI map and exported SLAM dataset (ADR-005). |
| **Storage Management** | Automatic capture abort at 10% free NVMe storage threshold. | Protects dataset integrity and prevents filesystem corruption (ADR-006). |
| **Hardware Failures** | Automatic capture abort on any required sensor failure or disconnect. | Prevents silent data corruption and invalid un-colorized point cloud recording (ADR-007). |
