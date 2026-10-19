#!/bin/bash
# scanHUB Launch Script — Commercial Reality Capture OS (scanR Platform)
source /opt/ros/humble/setup.bash
source /home/scanar/scanarMini/install/setup.bash 2>/dev/null || true
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export PYTHONUNBUFFERED=1

START_STACK=0
APP_ARGS=()
for arg in "$@"; do
    if [ "$arg" = "--start-stack" ] || [ "$arg" = "--with-sensors" ]; then
        START_STACK=1
    else
        APP_ARGS+=("$arg")
    fi
done

SENSORS_PID=""
SLAM_PID=""

cleanup_stack() {
    if [ -n "$SLAM_PID" ]; then
        echo "[scanHUB] Stopping FAST-LIVO2 SLAM stack..."
        kill -INT -"$SLAM_PID" 2>/dev/null || kill -INT "$SLAM_PID" 2>/dev/null || true
    fi
    if [ -n "$SENSORS_PID" ]; then
        echo "[scanHUB] Stopping Sensor Bringup stack..."
        kill -INT -"$SENSORS_PID" 2>/dev/null || kill -INT "$SENSORS_PID" 2>/dev/null || true
    fi
}
trap cleanup_stack EXIT INT TERM

if [ "$START_STACK" -eq 1 ]; then
    echo "[scanHUB] Verifying Airy SYNC_OUT1 hardware trigger..."
    python3 /home/scanar/scanarMini/scripts/configure_airy_sync_out.py >/dev/null 2>&1 || true
    echo "[scanHUB] Starting Sensor Bringup (Airy LiDAR + Gemini 336L)..."
    setsid ros2 launch scan_routine sensor_bringup.launch.py >/tmp/scanhub_sensor_bringup.log 2>&1 &
    SENSORS_PID=$!
    sleep 2
    echo "[scanHUB] Starting FAST-LIVO2 Odometry & Mapping..."
    setsid ros2 launch fast_livo mapping_robosense.launch.py use_rviz:=false >/tmp/scanhub_fast_livo.log 2>&1 &
    SLAM_PID=$!
fi

echo "[scanHUB] Starting Commercial Reality Capture OS..."
python3 /home/scanar/scanarMini/src/scanhub_gui/scanhub_app.py "${APP_ARGS[@]}"

