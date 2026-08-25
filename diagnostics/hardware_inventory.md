# Hardware Inventory & Connection Status Report

**Device ID:** scanR Golden Rig (Device 2)  
**Host Platform:** Seeed Studio A608 / Jetson Orin NX 16GB (Ubuntu 22.04 / Linux 6.8 / ROS 2 Humble)  
**Report Generated:** 2026-07-29  

---

## Detected Physical Hardware

| Sensor / Subsystem | Model | Interface | System ID / Device Node | Connection Status |
| :--- | :--- | :--- | :--- | :--- |
| **Stereo Vision & RGB** | Orbbec Gemini 336L (Global Shutter) | USB 3.1 Direct Bus | `Bus 002 Device 003` (`2bc5:0807`) | **VERIFIED CONNECTED** |
| **LiDAR Sensor** | RoboSense Airy LiDAR | Ethernet UDP (192.168.1.200) | `eth0` / `enp*` Ethernet Interface | **NETWORK BUS** |
| **GNSS / INS Module** | InertialSense IG-2 Dual RTK | USB Virtual COM / TTL Serial | `/dev/ttyACM0` (`0483:5740`) | **VERIFIED CONNECTED** |
| **HUD / Operator Display** | VITURE Luma Ultra XR Glasses | USB 2.0 / Active HDMI Converter | `Bus 001 Device 007` (`35ca:1104`) | **VERIFIED CONNECTED** |
| **Microphone / Audio HUD** | VITURE Microphone | USB 2.0 | `Bus 001 Device 009` (`35ca:1102`) | **VERIFIED CONNECTED** |

---

## Hardwired Synchronization Wiring Configuration

- **Master Clock Engine:** InertialSense IG-2 Module
- **Gemini 336L Synchronization:** Gemini 336L Pin 3 (`VSYNC_OUT`) $\rightarrow$ IG-2 Pin H1-6 (`STROBE INPUT`), Pin 8 (`GND`) $\rightarrow$ IG-2 Pin H1-1 (`GND`). *(Gemini Pin 4 is UNCONNECTED)*
- **RoboSense Airy Synchronization:**
  - IG-2 Pin H1-7 (`Tx0`) $\rightarrow$ MAX3232 Transceiver $\rightarrow$ Airy Pin 1 (`GPS_GPRMC`)
  - IG-2 Pin H1-14 (`GPS.TIMEPULSE`) $\rightarrow$ Airy Pin 3 (`GPS_PPS`)
  - IG-2 Pin H1-3 (`3.3V VCC`) $\rightarrow$ MAX3232 VCC
  - IG-2 Pin H1-1 (`GND`) $\rightarrow$ MAX3232 GND & Airy Pin 4 (`GND`)
- **Clock Mode:** Master-referenced hardware timestamping (IG-2 strobe capture for camera frames + GPS/PPS time sync for LiDAR points).

---

## Local System Storage Baseline

- **Primary Storage (`/`):** NVMe SSD (915 GB total, 529 GB used, 340 GB available)
- **Free Space Percentage:** 37.1% free
- **10% Threshold Limit:** 91.5 GB remaining (Safety Auto-Stop triggered below this limit)
