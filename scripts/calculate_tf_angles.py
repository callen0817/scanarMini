import numpy as np
import scipy.spatial.transform as transform

def main():
    # 1. T_airy_lidar_airy_imu
    T_lidar_imu = np.array([
        [ 0.012367, -0.999919, -0.003080,  0.004250 ],
        [-0.999888, -0.012340, -0.008465,  0.004180 ],
        [ 0.008427,  0.003184, -0.999959, -0.004460 ],
        [ 0,         0,         0,         1        ]
    ])
    
    R_lidar_imu = T_lidar_imu[:3, :3]
    t_lidar_imu = T_lidar_imu[:3, 3]
    
    r_limu = transform.Rotation.from_matrix(R_lidar_imu)
    euler_limu = r_limu.as_euler('xyz', degrees=False)
    quat_limu = r_limu.as_quat()  # [x, y, z, w]
    
    print("Airy Lidar -> Airy IMU:")
    print(f"  Translation: {t_lidar_imu}")
    print(f"  Euler (XYZ rad): {euler_limu}")
    print(f"  Euler (XYZ deg): {np.degrees(euler_limu)}")
    print(f"  Quat (XYZW): {quat_limu}")
    
    # 2. T_cam0_airy_lidar (Cam0 optical relative to Airy Lidar)
    T_cam0_lidar = np.array([
        [ 0, -1,  0, +0.047640 ],
        [ 0,  0, -1, +0.014311 ],
        [ 1,  0,  0, -0.055700 ],
        [ 0,  0,  0,  1        ]
    ])
    
    R_cam0_lidar = T_cam0_lidar[:3, :3]
    t_cam0_lidar = T_cam0_lidar[:3, 3]
    
    r_c0l = transform.Rotation.from_matrix(R_cam0_lidar)
    euler_c0l = r_c0l.as_euler('xyz', degrees=False)
    quat_c0l = r_c0l.as_quat()
    
    print("\nCam0 Optical -> Airy Lidar:")
    print(f"  Translation: {t_cam0_lidar}")
    print(f"  Euler (XYZ rad): {euler_c0l}")
    print(f"  Euler (XYZ deg): {np.degrees(euler_c0l)}")
    print(f"  Quat (XYZW): {quat_c0l}")
    
    # 3. Cam0 -> Cam1 (Stereo baseline)
    # p_cam1 = T_cam1_cam0 * p_cam0
    # Cam1 is 95.2793 mm to the right of Cam0. In Cam0 optical frame, +X is right, +Y is down, +Z is forward.
    # Therefore, Cam1 translation relative to Cam0 is [-0.0952793, 0, 0] ?
    # Let's verify: Cam0 is +47.63965 mm left, Cam1 is -47.63965 mm right.
    # Separation = 95.2793 mm.
    # Relative translation from Cam0 to Cam1 is [-0.0952793, 0, 0] meters.
    print("\nCam0 Optical -> Cam1 Optical:")
    print(f"  Translation: [-0.0952793, 0.0, 0.0]")
    print(f"  Euler (XYZ rad): [0.0, 0.0, 0.0]")

if __name__ == "__main__":
    main()
