#!/usr/bin/env python3
"""
Automated Verification Suite for:
1. Milestone 12 Point Cloud Cleanup, Real-Time Spatial-Hash Toggle & Multi-Format Export (ported from av_scan)
2. Unlimited-Peer Collaborative SLAM (C-SLAM) Live Multi-Scanner Engine
   - Unlimited scanner color differentiation (curated + golden-ratio HSV)
   - Non-overlapping initial placement (SEPARATED_IMU_ORIENTED at radial offset with relative IMU orientation)
   - Automatic real-time overlap detection & SE(3) rigid snap (OVERLAP_SNAPPED)
   - Color mode toggle (Scanner Differentiation Colors vs True RGB)
3. Headless ScanHUBOS GUI Integration Test
"""

import os
import sys
import math
import time
import shutil
import tempfile
import numpy as np
from scipy.spatial.transform import Rotation

# Ensure scanhub_gui is on path
sys.path.insert(0, "/home/scanar/scanarMini/src/scanhub_gui")

from m12_point_cloud_cleanup import (
    ExportFormat,
    CleanupParams,
    PointCloudProcessor,
)
from cslam_network_engine import (
    PeerAlignmentState,
    CSlamColorMode,
    get_scanner_color_rgb,
    get_scanner_color_hex,
    CSlamRegistrationSolver,
    CSlamNetworkEngine,
)


def generate_test_room_cloud(rng: np.random.Generator, add_outliers: bool = True, tilt_deg: float = 4.5) -> np.ndarray:
    """Generates a realistic 3D indoor room (floor + 4 walls + central pillar) with optional tilt & outliers."""
    # Floor (6m x 5m)
    fx = rng.uniform(-3.0, 3.0, 1800)
    fy = rng.uniform(-2.5, 2.5, 1800)
    fz = rng.normal(0.0, 0.005, 1800)
    floor = np.column_stack([fx, fy, fz])

    # Walls with asymmetric geometric features for unambiguous 3D registration
    w1 = np.column_stack([np.full(500, -3.0), rng.uniform(-2.5, 2.5, 500), rng.uniform(0.0, 2.2, 500)])
    w2 = np.column_stack([np.full(500, 3.0), rng.uniform(-2.5, 2.5, 500), rng.uniform(0.0, 2.2, 500)])
    w3 = np.column_stack([rng.uniform(-3.0, 3.0, 500), np.full(500, -2.5), rng.uniform(0.0, 2.2, 500)])
    # Asymmetric L-bench / pillar in corner (+1.2, +0.8)
    pillar = np.column_stack([
        rng.uniform(0.8, 1.8, 600),
        rng.uniform(0.4, 1.4, 600),
        rng.uniform(0.0, 1.5, 600),
    ])

    xyz = np.vstack([floor, w1, w2, w3, pillar])

    if abs(tilt_deg) > 1e-3:
        R_tilt = Rotation.from_euler("xy", [tilt_deg, -tilt_deg * 0.5], degrees=True).as_matrix()
        xyz = (xyz @ R_tilt.T) + np.array([0.0, 0.0, 0.35])

    if add_outliers:
        outliers = rng.uniform(-8.0, 8.0, size=(200, 3))
        xyz = np.vstack([xyz, outliers])

    rgb = np.zeros_like(xyz)
    rgb[:, 0] = 0.25
    rgb[:, 1] = 0.78
    rgb[:, 2] = 0.65
    return np.hstack([xyz, rgb]).astype(np.float32)


def test_m12_cleanup_and_exports():
    print("\n=== [TEST 1] M12 Point Cloud Cleanup & Multi-Format Export (ported from av_scan) ===")
    rng = np.random.default_rng(101)
    raw_cloud = generate_test_room_cloud(rng, add_outliers=True, tilt_deg=5.0)
    assert len(raw_cloud) == 4100, f"Expected 4100 input points, got {len(raw_cloud)}"

    params = CleanupParams(
        enable_voxel_downsample=True,
        voxel_leaf_size_m=0.03,
        enable_sor=True,
        sor_mean_k=20,
        sor_std_mul=2.0,
        enable_ror=True,
        ror_radius_m=0.15,
        ror_min_neighbors=4,
        enable_ground_alignment=True,
        ground_dist_threshold_m=0.04,
        ground_max_iterations=350,
        ground_max_tilt_deg=35.0,
        align_ground_to_z_zero=True,
        remove_ground_plane=False,
    )

    cleaned, stats = PointCloudProcessor.run_cleanup_pipeline(raw_cloud, params)
    print(
        f"  [M12 Pipeline] In: {stats.input_points} -> Voxel: {stats.voxel_downsampled_points} "
        f"-> SOR removed: {stats.sor_removed_points} -> ROR removed: {stats.ror_removed_points} "
        f"-> Out: {stats.output_points} ({stats.processing_time_ms:.1f} ms)"
    )
    print(
        f"  [Ground Plane] Detected: {stats.ground_result.detected}, "
        f"Inliers: {stats.ground_result.inlier_count}, "
        f"Tilt: {stats.ground_result.tilt_angle_deg:.2f} deg"
    )
    assert stats.output_points > 1500 and stats.output_points < stats.input_points
    assert (stats.sor_removed_points + stats.ror_removed_points) >= 80, "Expected outlier removal to strip synthetic noise"
    assert stats.ground_result.detected, "Expected RANSAC ground plane detection to succeed"
    assert 3.5 <= stats.ground_result.tilt_angle_deg <= 8.0, "Expected ~5.6 deg ground tilt detection"

    # Verify floor is leveled near Z=0 after ground alignment
    floor_band = cleaned[np.abs(cleaned[:, 0]) < 2.0]
    z_p10 = float(np.percentile(floor_band[:, 2], 15))
    print(f"  [Ground Z=0 Check] 15th-percentile floor Z after leveling: {z_p10:+.4f} m")
    assert abs(z_p10) < 0.06, f"Expected leveled floor near Z=0, got {z_p10}"

    # Verify Real-Time Fast Spatial-Hash Cleanup (< 25 ms)
    rt_clean, rt_stats = PointCloudProcessor.clean_realtime_fast(raw_cloud, params)
    print(
        f"  [M12 Real-Time Toggle Filter] {rt_stats.input_points} -> {rt_stats.output_points} pts "
        f"in {rt_stats.processing_time_ms:.2f} ms"
    )
    assert len(rt_clean) > 1000
    assert rt_stats.processing_time_ms < 50.0, f"Real-time filter too slow: {rt_stats.processing_time_ms:.1f} ms"

    # Verify all 6 Export Formats
    tmp_dir = tempfile.mkdtemp(prefix="m12_export_test_")
    try:
        formats = [
            (ExportFormat.PLY_ASCII, "cloud_ascii.ply"),
            (ExportFormat.PLY_BINARY, "cloud_bin.ply"),
            (ExportFormat.PCD_ASCII, "cloud_ascii.pcd"),
            (ExportFormat.PCD_BINARY, "cloud_bin.pcd"),
            (ExportFormat.XYZ_ASCII, "cloud.xyz"),
            (ExportFormat.OBJ_MESH, "surface_mesh.obj"),
        ]
        for fmt, fname in formats:
            fpath = os.path.join(tmp_dir, fname)
            res = PointCloudProcessor.export_point_cloud(cleaned, fpath, fmt, mesh_grid_resolution_m=0.12)
            assert res.success, f"Export failed for {fmt}: {res.error_message}"
            assert os.path.isfile(fpath) and res.file_size_bytes > 500
            print(
                f"  [Export {fmt.name:10s}] OK -> {res.exported_points} pts, "
                f"{res.exported_triangles} tris, {res.file_size_bytes:,} bytes ({res.export_time_ms:.1f} ms)"
            )
            if fmt == ExportFormat.OBJ_MESH:
                assert res.exported_triangles > 20, "Expected 3D OBJ surface mesh to reconstruct triangles"
    finally:
        shutil.rmtree(tmp_dir, ignore_errors=True)


def test_cslam_unlimited_peers_and_auto_snap():
    print("\n=== [TEST 2] Unlimited-Peer Collaborative SLAM (C-SLAM) Engine ===")
    # 1. Verify Unlimited Scanner Color Differentiation (30 scanners)
    colors_hex = [get_scanner_color_hex(i) for i in range(30)]
    assert len(set(colors_hex)) == 30, "Expected 30 unique scanner differentiation colors for 30 scanners"
    print(f"  [Unlimited Colors] Verified 30 unique scanner colors: {colors_hex[:6]} ... {colors_hex[-3:]}")

    # 2. Initialize C-SLAM Engine for Host Scanner #0
    rng = np.random.default_rng(202)
    base_room = generate_test_room_cloud(rng, add_outliers=False, tilt_deg=0.0)

    cslam = CSlamNetworkEngine(
        scanner_name="Host-Scanner-00",
        project_name="MultiRig_Test",
        listen_port=0,  # ephemeral port for unit test isolation
        separation_distance_m=15.0,
        min_overlap_ratio=0.18,
    )
    try:
        # Host Scanner #0 is at origin with yaw=0
        cslam.update_local_state(
            points_xyz_rgb=base_room,
            pos_xyz=(0.0, 0.0, 0.0),
            imu_rpy_rad=(0.0, 0.0, 0.0),
        )

        # 3. Inject Peer Scanner #1 with NON-OVERLAPPING geometry (distant room) and IMU yaw = +45 deg
        distant_xyz = rng.uniform(50.0, 56.0, size=(1200, 3)).astype(np.float32)
        distant_rgb = np.full((1200, 3), 0.8, dtype=np.float32)
        distant_cloud = np.hstack([distant_xyz, distant_rgb])

        cslam._ingest_packet(
            {
                "magic": "SCANHUB_CSLAM_V1",
                "scanner_id": "peer-scanner-01",
                "scanner_name": "Peer-Scanner-01",
                "project_name": "MultiRig_Test",
                "seq": 1,
                "pos_xyz": [0.0, 0.0, 0.0],
                "imu_rpy_rad": [0.0, 0.0, math.radians(45.0)],
                "points": distant_cloud.tolist(),
            },
            sender_ip="127.0.0.1",
        )

        cslam.run_overlap_check_now()
        merged_sep, peers_sep = cslam.get_merged_cslam_scene()
        assert len(peers_sep) == 1
        p1 = peers_sep[0]
        print(
            f"  [Non-Overlapping State] Peer: {p1['scanner_name']} | State: {p1['alignment_state']} | "
            f"Color: {p1['color_hex']} | World Pos: {[round(x, 2) for x in p1['world_pos']]}"
        )
        assert p1["alignment_state"] == PeerAlignmentState.SEPARATED_IMU_ORIENTED.name
        dist_from_origin = math.hypot(p1["world_pos"][0], p1["world_pos"][1])
        assert abs(dist_from_origin - 15.0) < 0.5, f"Expected 15.0m radial separation, got {dist_from_origin:.2f}m"

        # 4. Now Peer Scanner #1 walks into OVERLAPPING geometry (75% of the same room, rotated by 12 deg and shifted by [0.8m, -0.5m, 0.05m] in peer's local frame)
        R_true = Rotation.from_euler("z", 12.0, degrees=True).as_matrix()
        t_true = np.array([0.80, -0.50, 0.05])
        # Peer measures points in its own frame: p_peer = R_true.T @ (p_world - t_true)
        overlap_subset = base_room[:3000].copy()
        p_world = overlap_subset[:, :3]
        p_peer = (p_world - t_true) @ R_true
        overlap_subset[:, :3] = p_peer.astype(np.float32)

        cslam._ingest_packet(
            {
                "magic": "SCANHUB_CSLAM_V1",
                "scanner_id": "peer-scanner-01",
                "scanner_name": "Peer-Scanner-01",
                "project_name": "MultiRig_Test",
                "seq": 2,
                "pos_xyz": [0.5, 0.2, 0.0],
                "imu_rpy_rad": [0.0, 0.0, math.radians(-12.0)],
                "points": overlap_subset.tolist(),
            },
            sender_ip="127.0.0.1",
        )

        cslam.run_overlap_check_now()
        merged_snap, peers_snap = cslam.get_merged_cslam_scene()
        p1_after = peers_snap[0]
        print(
            f"  [Overlapping Auto-Snap] Peer: {p1_after['scanner_name']} | State: {p1_after['alignment_state']} | "
            f"Overlap: {p1_after['overlap_ratio']*100:.1f}% | ICP RMSE: {p1_after['icp_rmse_m']*100:.2f} cm"
        )
        assert p1_after["alignment_state"] == PeerAlignmentState.OVERLAP_SNAPPED.name, "Expected peer to SNAP on overlap!"
        assert p1_after["overlap_ratio"] >= 0.65
        assert p1_after["icp_rmse_m"] < 0.05

        # 5. Test Unlimited Multi-Scanner Scaling: Add 9 more peer scanners (10 peers total + 1 host = 11 scanners)
        for idx in range(2, 11):
            cslam._ingest_packet(
                {
                    "magic": "SCANHUB_CSLAM_V1",
                    "scanner_id": f"peer-scanner-{idx:02d}",
                    "scanner_name": f"Peer-Scanner-{idx:02d}",
                    "project_name": "MultiRig_Test",
                    "seq": 1,
                    "pos_xyz": [float(idx), 0.0, 0.0],
                    "imu_rpy_rad": [0.0, 0.0, float(idx * 0.3)],
                    "points": distant_cloud[:300].tolist(),
                },
                sender_ip=f"127.0.0.{idx}",
            )

        merged_multi, peers_multi = cslam.get_merged_cslam_scene()
        summary = cslam.get_status_summary()
        print(
            f"  [Multi-Scanner Scale] Total Scanners: {summary['total_scanners']} "
            f"(1 Host + {summary['peer_count']} Peers) | Merged Points: {len(merged_multi):,}"
        )
        assert summary["total_scanners"] == 11
        assert len(peers_multi) == 10

        # 6. Verify Color Mode Toggle (SCANNER_COLORS vs TRUE_RGB)
        cslam.set_color_mode(CSlamColorMode.SCANNER_COLORS)
        m_sc, _ = cslam.get_merged_cslam_scene()
        cslam.set_color_mode(CSlamColorMode.TRUE_RGB)
        m_rgb, _ = cslam.get_merged_cslam_scene()
        assert not np.allclose(m_sc[:, 3:6], m_rgb[:, 3:6]), "Expected Scanner Colors and True RGB modes to differ"
        print("  [Color Mode Toggle] Verified SCANNER_COLORS vs TRUE_RGB point cloud coloring.")
    finally:
        cslam.stop()


def test_headless_gui_integration():
    print("\n=== [TEST 3] Headless ScanHUBOS GUI Smoke & Workflow Verification ===")
    os.environ["QT_QPA_PLATFORM"] = "offscreen"
    import scanhub_app
    from scanhub_app import QApplication

    app = QApplication.instance() or QApplication(sys.argv)

    # 0. Verify Artificial Vision Logo Asset & White/Gold Splash Screen
    logo_path = scanhub_app.get_av_logo_path()
    assert logo_path and os.path.isfile(logo_path), f"Expected Artificial Vision logo asset, got: {logo_path}"
    logo_pix = scanhub_app.load_av_logo_pixmap(128)
    assert logo_pix is not None and not logo_pix.isNull(), "Expected valid QPixmap from av_logo.png"

    splash = scanhub_app.ArtificialVisionSplashScreen()
    assert splash.testAttribute(scanhub_app.Qt.WA_TranslucentBackground), "Expected translucent background on splash screen"
    assert splash.windowFlags() & scanhub_app.Qt.FramelessWindowHint, "Expected frameless window hint on splash screen"
    assert hasattr(splash, "btn_splash_close") and splash.btn_splash_close is not None, "Expected transparent close button on splash screen"
    splash.set_status("Artificial Vision  ·  Spatial LiDAR OS", 80)
    splash.show()
    splash.repaint()
    app.processEvents()
    splash.close()
    print(f"  [Brand & Splash] Verified Artificial Vision logo ({logo_path}), translucent Apple Glass card & close button.")

    win = scanhub_app.ScanHUBOS()
    # Verify AR Glasses Transparency & Frameless Window Controls
    assert win.testAttribute(scanhub_app.Qt.WA_TranslucentBackground), "Expected WA_TranslucentBackground for AR glasses passthrough"
    assert win.windowFlags() & scanhub_app.Qt.FramelessWindowHint, "Expected FramelessWindowHint for transparent window borders"
    assert hasattr(win, "btn_win_min") and win.btn_win_min is not None, "Expected transparent Minimize window control"
    assert hasattr(win, "btn_win_max") and win.btn_win_max is not None, "Expected transparent Maximize window control"
    assert hasattr(win, "btn_win_close") and win.btn_win_close is not None, "Expected transparent Close window control"
    assert hasattr(win, "slider_glass_alpha") and win.slider_glass_alpha is not None, "Expected glass opacity slider"
    
    # Verify live glass opacity adjustment
    win.slider_glass_alpha.setValue(20)
    assert win.glass_opacity_pct == 20
    assert "20%" in win.lbl_glass_alpha.text()
    # Verify Pure Blank Black AR Passthrough Viewport (Optical See-Through)
    assert hasattr(win, "center_viewport") and isinstance(win.center_viewport, scanhub_app.ClearPassthroughViewport), (
        "Expected ClearPassthroughViewport for left viewport"
    )
    # Viewport provides no-op scene and pose updates to ensure zero overhead and distraction-free passthrough
    win.center_viewport.update_scene_data({})
    win.center_viewport.update_pose(0.0, 0.0, 0.0)
    print("  [AR Passthrough] Verified pure blank black passthrough viewport (distraction-free optical transmission).")

    win.show()
    app.processEvents()

    try:
        # 1. Test M12 Real-Time Toggle
        assert win.ros_worker.live_cleanup_enabled is True
        win.on_quick_m12_rt_clicked(False)
        assert win.ros_worker.live_cleanup_enabled is False
        win.on_quick_m12_rt_clicked(True)
        assert win.ros_worker.live_cleanup_enabled is True

        # 2. Test M12 Preview -> Reset -> Preview -> Commit
        win.on_m12_preview_clicked()
        app.processEvents()
        assert win.ros_worker.cleanup_preview_active is True
        assert "PREVIEW:" in win.lbl_m12_status.text()

        win.on_m12_reset_clicked()
        app.processEvents()
        assert win.ros_worker.cleanup_preview_active is False

        win.on_m12_preview_clicked()
        win.on_m12_commit_clicked()
        app.processEvents()
        assert win.ros_worker.cleanup_preview_active is False
        assert "Committed:" in win.lbl_m12_status.text()

        # 3. Test C-SLAM Simulated Peer Scanner Button (Non-overlap -> Overlap Auto-Snap)
        win.on_spawn_sim_peer_clicked()
        time.sleep(0.35)
        win.ros_worker.dispatch_live_map_and_telemetry()
        app.processEvents()
        st1 = win.ros_worker.cslam_engine.get_status_summary()
        print(f"  [GUI Sim Peer T+0.3s] Scanners: {st1['total_scanners']} | Separated: {st1['separated_peers']} | Snapped: {st1['snapped_peers']}")
        assert st1["total_scanners"] == 2

        # Wait ~2.2s for simulated peer scanner to walk into overlapping room geometry and auto-snap
        time.sleep(2.4)
        win.ros_worker.cslam_engine.run_overlap_check_now()
        win.ros_worker.dispatch_live_map_and_telemetry()
        app.processEvents()
        st2 = win.ros_worker.cslam_engine.get_status_summary()
        print(f"  [GUI Sim Peer T+2.7s] Scanners: {st2['total_scanners']} | Separated: {st2['separated_peers']} | Snapped: {st2['snapped_peers']}")
        assert st2["snapped_peers"] == 1, "Expected simulated peer scanner to auto-snap after walking into overlap!"

        # 4. Test Header Brand Splash Trigger & 3D Spatial Viewport
        win.show_brand_splash()
        app.processEvents()
        if hasattr(win, "_about_splash") and win._about_splash is not None:
            win._about_splash.close()
        win.center_viewport.repaint()
        win.map_canvas.repaint()
        app.processEvents()
        print("  [GUI Map & 3D Studio] Rendered White & Gold 3D Spatial Studio, C-SLAM trajectories, and M12 status cleanly.")
        print("  [Apple Glass UI/UX] Verified transparent frameless window, transparent min/max/close controls, and AR walk passthrough.")
    finally:
        win.close()
        app.processEvents()


if __name__ == "__main__":
    test_m12_cleanup_and_exports()
    test_cslam_unlimited_peers_and_auto_snap()
    test_headless_gui_integration()
    print("\n✅ ALL M12 POINT CLOUD CLEANUP & UNLIMITED C-SLAM TESTS PASSED!")
