#!/bin/bash
# scanHUB Launch Script — Commercial Reality Capture OS (scanR Platform)
source /opt/ros/humble/setup.bash
source /home/scanar/scanarMini/install/setup.bash 2>/dev/null || true
export ROS_DOMAIN_ID=2
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export PYTHONUNBUFFERED=1

echo "[scanHUB] Starting Commercial Reality Capture OS..."
exec python3 /home/scanar/scanarMini/src/scanhub_gui/scanhub_app.py "$@"
