# scanR Golden Dataset Reference Registry

To prevent software regressions and objectively measure progress toward matching **NavVis VLX** and **FARO Orbis** performance, `scanR` uses a permanent set of reference environments called **Golden Datasets**.

These reference scans are captured in fixed, repeatable real-world environments and are preserved permanently. After any change to FAST-LIVO2, sensor drivers, time synchronization, calibration, or export pipelines, the system is re-run against these environments to catch regressions immediately.

---

## Golden Dataset Reference Scans

| ID | Environment | Description / Challenge | Primary Metric Tested |
| :--- | :--- | :--- | :--- |
| **GD-01** | **Small Office** | Enclosed $50 \text{ m}^2$ office with furniture, doorway, and glass partitions. | Local planar accuracy, near-field detail. |
| **GD-02** | **Long Corridor** | $60 \text{ m}$ featureless indoor hallway with white walls. | Geometric degeneracy prevention, drift rate. |
| **GD-03** | **Large Warehouse** | $1,000+ \text{ m}^2$ open space with high ceilings and steel columns. | Long-range LiDAR ranging, voxel map scale. |
| **GD-04** | **Multi-Story Stairwell** | Tight vertical spiral stairwell across 3 floors. | Vertical Z-drift, steep pitch tilt tracking. |
| **GD-05** | **Outdoor Campus** | Outdoor courtyard with trees, sidewalks, and building facades. | Sunlight immunity, vegetation density. |
| **GD-06** | **Indoor ↔ Outdoor Transition** | Continuous walk from indoor office out through glass doors into bright sunlight. | Exposure transition, RTK GNSS acquisition. |
| **GD-07** | **Low-Light Environment** | Dimly lit basement / mechanical room with pipe networks. | VIO low-light performance, LiDAR structural tracking. |
| **GD-08** | **Feature-Poor Environment** | Smooth white corridor with uniform flooring and zero visual texture. | LIO constraint stability without visual features. |
| **GD-09** | **Dynamic Environment** | Active hallway with moving people and passing vehicles. | Dynamic object rejection, moving ghost removal. |

---

## Regression Testing Protocol

Before approving any release candidate (v1.0 RC):

1. **Re-run Session:** Perform a scan walk along the exact marked ground-truth control path for each Golden Environment (GD-01 through GD-09).
2. **Compute Metrics:**
   * Measure localization drift vs marked control points ($< 0.15\%$ target).
   * Measure local planar residual ($< 1.5\text{ cm}$ target).
   * Verify zero double-wall artifacts or point cloud splitting.
   * Measure export duration and file size.
3. **Log Benchmarks:** Record findings in `docs/performance_benchmarking.md`.
