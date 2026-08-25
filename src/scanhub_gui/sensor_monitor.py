#!/usr/bin/env python3
"""
scanHUB Sensor Health & Failure Monitor for scanarMini
Monitors ROS 2 sensor topics in real time (Airy LiDAR, Airy IMU, Gemini Stereo IR).
Triggers safety alerts and auto-stops if required sensors stall or fail.
"""

import time
import threading

class SensorMonitor:
    def __init__(self):
        self.sensor_states = {
            "rslidar": {"name": "RoboSense Airy LiDAR", "topic": "/rslidar_points", "hz": 0.0, "count": 0, "last_time": None, "ok": False},
            "imu":     {"name": "RoboSense Airy IMU",   "topic": "/rslidar_imu_data", "hz": 0.0, "count": 0, "last_time": None, "ok": False},
            "camera":  {"name": "Gemini 336L Left IR",  "topic": "/cam0/image_raw",   "hz": 0.0, "count": 0, "last_time": None, "ok": False},
            "camera1": {"name": "Gemini 336L Right IR", "topic": "/cam1/image_raw",  "hz": 0.0, "count": 0, "last_time": None, "ok": False},
        }
        self.sync_states = {
            "gemini_strobe": {"name": "336L STROBE", "status": "WAITING", "last_stamp": None, "count": 0},
            "airy_trigger":  {"name": "AIRY TRIGGER", "status": "WAITING",  "last_stamp": None, "ok": False},
            "cam_airy_delta_ms": 999.0
        }
        self.last_cam_stamp = None
        self.last_imu_stamp = None
        self.start_time = time.time()
        self.is_monitoring = False
        self.ros_node = None
        self.thread = None

    def start_ros_listener(self):
        """
        Starts ROS 2 subscription thread using rclpy if available,
        or operates in fallback monitor mode.
        """
        try:
            import rclpy
            from rclpy.node import Node
            from sensor_msgs.msg import PointCloud2, Imu, Image

            from rclpy.qos import qos_profile_sensor_data

            if not rclpy.ok():
                rclpy.init()

            class ROS2HealthNode(Node):
                def __init__(node_self, outer_self):
                    super().__init__('scanhub_sensor_monitor')
                    node_self.outer = outer_self
                    
                    node_self.create_subscription(
                        PointCloud2, '/rslidar_points', 
                        lambda msg: node_self.callback('rslidar', msg), qos_profile_sensor_data)
                    node_self.create_subscription(
                        Imu, '/rslidar_imu_data', 
                        lambda msg: node_self.callback('imu', msg), qos_profile_sensor_data)
                    node_self.create_subscription(
                        Image, '/camera/left_ir/image_raw', 
                        lambda msg: node_self.callback('camera', msg), qos_profile_sensor_data)
                    node_self.create_subscription(
                        Image, '/camera/right_ir/image_raw', 
                        lambda msg: node_self.callback('camera1', msg), qos_profile_sensor_data)

                def callback(node_self, sensor_key, msg=None):
                    now = time.time()
                    st = node_self.outer.sensor_states[sensor_key]
                    st['count'] += 1
                    st['last_time'] = now
                    st['ok'] = True

                    if msg is not None and hasattr(msg, 'header'):
                        stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                        if sensor_key == 'camera':
                            node_self.outer.last_cam_stamp = stamp_sec
                            node_self.outer.sync_states['gemini_strobe']['count'] += 1
                            node_self.outer.sync_states['gemini_strobe']['last_stamp'] = stamp_sec
                            node_self.outer.sync_states['gemini_strobe']['status'] = "LOCKED"
                        elif sensor_key == 'imu':
                            node_self.outer.last_imu_stamp = stamp_sec
                            node_self.outer.sync_states['airy_trigger']['last_stamp'] = stamp_sec
                            node_self.outer.sync_states['airy_trigger']['status'] = "LOCKED"
                            node_self.outer.sync_states['airy_trigger']['ok'] = True

                        if node_self.outer.last_cam_stamp is not None and node_self.outer.last_imu_stamp is not None:
                            delta_ms = abs(node_self.outer.last_cam_stamp - node_self.outer.last_imu_stamp) * 1000.0
                            node_self.outer.sync_states['cam_airy_delta_ms'] = round(delta_ms, 2)

            self.ros_node = ROS2HealthNode(self)
            
            def spin_worker():
                try:
                    rclpy.spin(self.ros_node)
                except Exception:
                    pass

            self.thread = threading.Thread(target=spin_worker, daemon=True)
            self.thread.start()
            self.is_monitoring = True
        except ImportError:
            # Fallback if rclpy is not directly available in current script context
            self.is_monitoring = True

    def get_sync_status(self):
        """
        Returns dictionary of hardware synchronization status for scanHUB HUD.
        """
        return {
            "gemini_strobe": self.sync_states["gemini_strobe"]["status"],
            "airy_trigger": self.sync_states["airy_trigger"]["status"],
            "cam_airy_delta_ms": self.sync_states["cam_airy_delta_ms"]
        }

    def check_health(self, timeout_sec=3.0):
        """
        Evaluates health status of all sensors.
        Returns (all_healthy, failed_sensors_list, status_summary_dict)
        """
        now = time.time()
        elapsed = max(now - self.start_time, 1.0)
        failed_sensors = []
        summary = {}

        for key, info in self.sensor_states.items():
            last = info["last_time"]
            count = info["count"]

            if last is None:
                hz = 0.0
                ok = False
                status_str = "NO DATA"
            elif now - last > timeout_sec:
                hz = 0.0
                ok = False
                status_str = f"STALE ({now - last:.1f}s ago)"
                failed_sensors.append(info["name"])
            else:
                hz = round(count / elapsed, 1)
                ok = True
                status_str = f"OK ({hz} Hz)"

            info["hz"] = hz
            info["ok"] = ok

            summary[key] = {
                "name": info["name"],
                "topic": info["topic"],
                "hz": hz,
                "ok": ok,
                "status": status_str
            }

        all_healthy = (len(failed_sensors) == 0)
        return all_healthy, failed_sensors, summary
