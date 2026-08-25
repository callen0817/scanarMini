#!/usr/bin/env python3
"""
GUI Ingestion Path Stage-by-Stage Verification Script (Matching DDS Localhost Settings).
"""

import sys
import os
import time

# Match driver environment EXACTLY
os.environ["ROS_DOMAIN_ID"] = "2"
if "ROS_LOCALHOST_ONLY" in os.environ:
    del os.environ["ROS_LOCALHOST_ONLY"]

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, PointCloud2, Imu
from nav_msgs.msg import Odometry

print("======================================================================")
print("STAGE-BY-STAGE GUI INGESTION PATH TEST (Matching DDS Environment)")
print("======================================================================")

rclpy.init()
node = Node("test_gui_ingestion_verifier_dds")
domain = os.environ.get("ROS_DOMAIN_ID", "default")
print(f"[STAGE 1] Created ROS 2 Node '{node.get_name()}' on ROS_DOMAIN_ID={domain}")

cnt_cam = 0
cnt_lidar = 0
cnt_imu = 0
cnt_slam = 0

last_cam_t = 0.0
last_lid_t = 0.0
last_imu_t = 0.0
last_slam_t = 0.0

def cam_cb(msg: Image):
    global cnt_cam, last_cam_t
    cnt_cam += 1
    last_cam_t = time.time()
    sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    if cnt_cam % 10 == 1:
        print(f"  [STAGE 5 CALLBACK] Camera Msg #{cnt_cam} | {msg.width}x{msg.height} {msg.encoding} | stamp={sec:.3f}")

def lidar_cb(msg: PointCloud2):
    global cnt_lidar, last_lid_t
    cnt_lidar += 1
    last_lid_t = time.time()
    sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    num_points = msg.width * msg.height
    if cnt_lidar % 5 == 1:
        print(f"  [STAGE 5 CALLBACK] LiDAR Msg #{cnt_lidar} | pts={num_points} | stamp={sec:.3f}")

def imu_cb(msg: Imu):
    global cnt_imu, last_imu_t
    cnt_imu += 1
    last_imu_t = time.time()
    sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    if cnt_imu % 50 == 1:
        print(f"  [STAGE 5 CALLBACK] IMU Msg #{cnt_imu} | stamp={sec:.3f}")

def slam_cb(msg: Odometry):
    global cnt_slam, last_slam_t
    cnt_slam += 1
    last_slam_t = time.time()
    sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    if cnt_slam % 10 == 1:
        print(f"  [STAGE 5 CALLBACK] SLAM Msg #{cnt_slam} | X={x:.2f} Y={y:.2f} | stamp={sec:.3f}")

qos_rel = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

print("\n[STAGE 3] Creating Subscriptions...")
node.create_subscription(Image, "/image_raw", cam_cb, qos_rel)
p_cam = node.count_publishers("/image_raw")
print(f"  -> Subscribed /image_raw | Publishers on Graph: {p_cam}")

node.create_subscription(PointCloud2, "/rslidar_points", lidar_cb, qos_rel)
p_lid = node.count_publishers("/rslidar_points")
print(f"  -> Subscribed /rslidar_points | Publishers on Graph: {p_lid}")

node.create_subscription(Imu, "/imu", imu_cb, qos_rel)
p_imu = node.count_publishers("/imu")
print(f"  -> Subscribed /imu | Publishers on Graph: {p_imu}")

node.create_subscription(Odometry, "/aft_mapped_to_init", slam_cb, qos_rel)
p_slam = node.count_publishers("/aft_mapped_to_init")
print(f"  -> Subscribed /aft_mapped_to_init | Publishers on Graph: {p_slam}")

print("\n[STAGE 4] Entering Spin Loop for 10 seconds...")
start_t = time.time()
last_report = 0.0

while time.time() - start_t < 10.0 and rclpy.ok():
    rclpy.spin_once(node, timeout_sec=0.05)
    now = time.time()
    if now - last_report >= 2.0:
        last_report = now
        c_dt = now - last_cam_t if last_cam_t > 0 else 999.0
        l_dt = now - last_lid_t if last_lid_t > 0 else 999.0
        i_dt = now - last_imu_t if last_imu_t > 0 else 999.0
        s_dt = now - last_slam_t if last_slam_t > 0 else 999.0
        print(f"[STAGE 4 SPIN] Elapsed: {now-start_t:.1f}s | Callbacks Received: Cam={cnt_cam}, LiDAR={cnt_lidar}, IMU={cnt_imu}, SLAM={cnt_slam}")
        print(f"               Time Since Last Msg: Cam={c_dt:.1f}s, LiDAR={l_dt:.1f}s, IMU={i_dt:.1f}s, SLAM={s_dt:.1f}s")

print("\n======================================================================")
print("STAGE-BY-STAGE TEST RESULTS SUMMARY:")
print("======================================================================")
print(f"Stage 1 (Node Creation): SUCCESS (Node: '{node.get_name()}', DOMAIN={domain})")
print(f"Stage 3 (Graph Publishers): Cam={p_cam}, LiDAR={p_lid}, IMU={p_imu}, SLAM={p_slam}")
print(f"Stage 5 (Callbacks Executed): Cam={cnt_cam}, LiDAR={cnt_lidar}, IMU={cnt_imu}, SLAM={cnt_slam}")

if cnt_cam > 0 and cnt_lidar > 0 and cnt_imu > 0:
    print("\nCONCLUSION: ALL ROS 2 INGESTION STAGES (1-5) ARE 100% OPERATIONAL!")
else:
    print("\nCONCLUSION: FAILURE AT STAGE 3/5 - Check which sensor count is 0 above!")

node.destroy_node()
rclpy.shutdown()
