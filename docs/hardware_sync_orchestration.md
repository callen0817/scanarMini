# scanR Hardware Synchronization & Extrinsic Validation Orchestration

**System Architecture:** `scanR` Handheld Reality Capture System  
**Master Timing Reference:** InertialSense IG-2 Dual RTK GNSS/INS  
**Connected Sensors:** Orbbec Gemini 336L Camera & RoboSense RS-LiDAR Airy  
**Date:** August 2026  

---

## 1. Objective & Scope

With the physical sync cable constructed, this orchestration protocol shifts `scanR` into a multi-stage hardware synchronization validation pass before plugging in and powering the system.

The timing chain is established as:

$$\text{IG-2 (Master Reference)} \longrightarrow \text{Gemini 336L + RoboSense Airy} \longrightarrow \text{FAST-LIVO2 SLAM} \longrightarrow \text{scanHUB Viewer}$$

**Key Governance Directive:** Time synchronization and spatial extrinsics are strictly isolated. Spatial extrinsics ($T_{\text{IMU}}^{\text{Camera}}, T_{\text{IMU}}^{\text{LiDAR}}$) remain frozen at CAD baseline values during timing bring-up.

---

## 2. Staged Milestone M5 Implementation Protocol

To ensure hardware safety, prevent bus contention, and isolate potential failures (electrical vs driver vs timing vs SLAM), Milestone 5 is strictly partitioned into six sequential sub-stages:

```
+-----------------------------------------------------------------------------+
| M5.0 — Pre-Cable Electrical Preflight & Power-OFF Continuity Checks         |
+-----------------------------------------------------------------------------+
                                       |
                                       v
+-----------------------------------------------------------------------------+
| M5.1 — IG-2 Strobe Input & PPS/GPRMC Output Hardware Driver Verification    |
| (Power IG-2 Standalone - NO external sync cables connected)                 |
+-----------------------------------------------------------------------------+
                                       |
                                       v
+-----------------------------------------------------------------------------+
| M5.2 — Gemini 336L ↔ IG-2 Hardware Strobe Timing Verification               |
| (Connect 336L to IG-2 - Verify actual camera trigger frequency vs FPS)      |
+-----------------------------------------------------------------------------+
                                       |
                                       v
+-----------------------------------------------------------------------------+
| M5.3 — RoboSense Airy ↔ IG-2 PPS & GPRMC Time Sync Verification             |
| (Connect Airy to IG-2 - Verify PPS / GPRMC clock lock)                      |
+-----------------------------------------------------------------------------+
                                       |
                                       v
+-----------------------------------------------------------------------------+
| M5.4 — Combined Multi-Sensor Temporal Alignment Pass                        |
| (Run all 3 sensors together - Measure temporal alignment & jitter)          |
+-----------------------------------------------------------------------------+
                                       |
                                       v
+-----------------------------------------------------------------------------+
| M5.5 — FAST-LIVO2 Full SLAM Integration & System Validation (Tests A, B, C, D)|
+-----------------------------------------------------------------------------+
```

---

## 3. Physical Wiring Reference Architecture

The physical sync cable establishes the IG-2 as the single timing reference hub with a common ground:

```
                    IG-2 Dual RTK
                     H1-1 GND
                      /     \
                     /       \
                  336L       Airy
                   
H1-6 STROBE  <--- 336L Pin 3 (VSYNC_OUT)
H1-7 Tx0     ---> MAX3232 ---> Airy Pin 1 (GPS_GPRMC)
H1-14 PPS    ---------------> Airy Pin 3 (GPS_PPS)
H1-1 GND     ---------------> Airy Pin 4 (GND)
H1-1 GND     ---------------> 336L Pin 8 (GND)
```

**Pin Disambiguation:** 
- Gemini 336L Pin 3 (`VSYNC_OUT`) $\rightarrow$ IG-2 H1-6 (`STROBE INPUT`).
- Gemini 336L Pin 8 (`GND`) $\rightarrow$ IG-2 H1-1 (`GND`).
- Gemini 336L Pin 4: **UNCONNECTED / NOT USED**. *(Note: Historical Kalibr calibration dataset references Pin 4, but physical system operation is frozen on validated Pin 3).*

---

## 4. Sub-Stage M5.0: Pre-Cable Electrical Preflight & Power-OFF Checks

**KEEP CABLES UNPLUGGED AND POWER OFF UNTIL ALL M5.0 CHECKS PASS.**

### A. Electrical Level & Pin Direction Verification Protocols

1. **IG-2 Pin H1-6 (STROBE Input):**
   - Confirm pin is configured strictly as **STROBE INPUT** (rising edge trigger) in IG-2 flash config.
   - Confirm pin is **NOT** configured as an active output to prevent bus contention.
2. **IG-2 Pin H1-7 (Tx0 GPRMC Output):**
   - **Measure actual electrical output logic level** (3.3V TTL into MAX3232 transceiver, RS-232 level output to Airy Pin 1) with digital multimeter.
   - Verify MAX3232 VCC is powered (~3.3V from IG-2 H1-3) and T1OUT provides valid RS-232 level signal to Airy Pin 1.
3. **IG-2 Pin H1-14 (GPS.TIMEPULSE Output):**
   - **Measure actual high-level pulse voltage** using digital multimeter (watch for 1-PPS pulse activity).
   - Confirm compatibility with RoboSense Airy Pin 3 (`GPS_PPS`) input.
   - **Do not connect until voltage compatibility is explicitly confirmed.**
4. **Gemini 336L Pin 3 (`VSYNC_OUT`):**
   - Confirm Orbbec SDK / camera driver outputs VSYNC strobe pulses on Pin 3.
   - **Dynamically derive expected trigger frequency** from active camera stream parameters:
     - 336L Depth FPS: `XX Hz`
     - 336L RGB FPS: `XX Hz`
     - Expected `VSYNC_OUT` Frequency: `XX Hz`
   - *Do not assume or hardcode 20 Hz; verify against active stream mode.*
5. **Common Ground Reference:**
   - Confirm IG-2 Pin H1-1 serves as the single common ground for 336L (Pin 8), MAX3232, and Airy (Pin 4).
6. **Software Driver Safety:**
   - Verify no ROS node or background script attempts to write to H1-6 or H1-14 as GPIO output pins.

---

### B. Power-OFF Multimeter Continuity & Short-Circuit Test Matrix

With all devices **POWERED OFF**, measure resistance / continuity across cable pins using a digital multimeter:

| Test Target | Point A | Point B | Expected Result | Verified |
| :--- | :--- | :--- | :--- | :---: |
| **336L Strobe** | 336L Pin 3 (`VSYNC_OUT`) | IG-2 Pin H1-6 (`STROBE`) | **CONTINUITY** ($< 1\ \Omega$) | [ ] |
| **336L Ground** | 336L Pin 8 (`GND`) | IG-2 Pin H1-1 (`GND`) | **CONTINUITY** ($< 1\ \Omega$) | [ ] |
| **Airy GPRMC** | Airy Pin 1 (`GPS_GPRMC`) | MAX3232 `T1OUT` | **CONTINUITY** ($< 1\ \Omega$) | [ ] |
| **Airy PPS** | Airy Pin 3 (`GPS_PPS`) | IG-2 Pin H1-14 (`GPS.TIMEPULSE`) | **CONTINUITY** ($< 1\ \Omega$) | [ ] |
| **Airy Ground** | Airy Pin 4 (`GND`) | IG-2 Pin H1-1 (`GND`) | **CONTINUITY** ($< 1\ \Omega$) | [ ] |
| **Short Check 1** | 336L Pin 3 | Ground (`GND`) | **NO SHORT** ($\infty\ \Omega$) | [ ] |
| **Short Check 2** | Airy Pin 3 | Ground (`GND`) | **NO SHORT** ($\infty\ \Omega$) | [ ] |
| **Short Check 3** | Airy Pin 1 | Ground (`GND`) | **NO SHORT** ($\infty\ \Omega$) | [ ] |

---

## 5. Sub-Stages M5.1 to M5.5 Execution Details

### M5.1 — IG-2 Hardware Driver Strobe & PPS Verification
- Power IG-2 module independently (NO external sync cables connected).
- Verify IG-2 firmware logs `DID_STROBE_IN_TIME` message stream.
- Verify IG-2 outputs 1-PPS pulse on Pin H1-14 and NMEA GPRMC string on Pin H1-7.

### M5.2 — 336L ↔ IG-2 Timing Verification
- Connect 336L camera sync cable.
- Power camera and capture `/image_raw` frame timestamps against IG-2 `DID_STROBE_IN_TIME` timestamps.
- **Verify IG-2 strobe trigger frequency matches camera's configured FPS** (`Depth FPS` / `RGB FPS`).

### M5.3 — RoboSense Airy ↔ IG-2 Timing Verification
- Connect Airy LiDAR sync cable.
- Power Airy LiDAR and inspect `/rslidar_points` timestamps.
- Verify Airy driver locks to IG-2 GPS/PPS clock (`use_lidar_clock: true`).

### M5.4 — Combined Multi-Sensor Temporal Alignment Pass
- Record stationary multi-sensor stream (IG-2 IMU, 336L camera, Airy LiDAR).
- Measure timestamp deltas across streams:
  - $(t_{\text{336L}} - t_{\text{IG2}})$, $(t_{\text{Airy}} - t_{\text{IG2}})$, $(t_{\text{336L}} - t_{\text{Airy}})$
- Pass criterion: Mean temporal offset $< 1.5\text{ ms}$, jitter $< 0.5\text{ ms}$.

### M5.5 — FAST-LIVO2 Integration & System Validation
- Feed synchronized streams into FAST-LIVO2 engine.
- Execute validation tests A, B, C, D:
  - **Test A:** 60-second stationary timing pass.
  - **Test B:** $360^\circ$ stationary rotation pass (heading updates, position stays anchored).
  - **Test C:** $5\text{--}10\text{ m}$ translation walk pass.
  - **Test D:** Multi-sensor temporal alignment pass under active motion.

---

## 6. Frozen Spatial CAD Extrinsics (Isolated from Timing)

Spatial extrinsics remain strictly isolated from time calibration:

### A. Gemini 336L Camera CAD Transform
- $X = 0.000000\text{ m}$
- $Y = +0.024018\text{ m}$ ($17.018\text{ mm}$ glass $+ 7.0\text{ mm}$ setback)
- $Z = -0.032258\text{ m}$
- $\text{RPY} = (0.0^\circ, 0.0^\circ, 0.0^\circ)$

### B. RoboSense Airy LiDAR Mechanical CAD Transform
- $X = 0.000000\text{ m}$
- $Y = +0.035814\text{ m}$
- $Z = -0.074422\text{ m}$
- $\text{Mechanical RPY} = (0.0^\circ, -45.0^\circ, 0.0^\circ)$

---

## 7. Preflight Deliverables & Sign-Off Checklist

- [x] **Wiring Architecture Confirmed:** Physical cable pinout matches IG-2 H1-1, H1-6, H1-7, H1-14.
- [x] **Sub-Stage Hierarchy Partitioned:** M5.0 to M5.5 explicitly structured.
- [x] **Power-OFF Preflight Test Matrix Added:** Continuity and short-circuit checks defined.
- [x] **Electrical Measurement Protocol Added:** H1-14 pulse voltage and H1-7 Tx0 logic levels require empirical measurement before connecting.
- [x] **Dynamic Camera FPS Derivation Specified:** Trigger frequency derived from camera stream parameters, not hardcoded.
- [x] **Spatial Extrinsics Isolated:** CAD values explicitly frozen during timing pass.
