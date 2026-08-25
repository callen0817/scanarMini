from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node 
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

def generate_launch_description():
    sensor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('scan_routine'), 'launch', 'sensor_bringup.launch.py')
        ])
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('fast_livo'), 'launch', 'mapping_robosense.launch.py')
        ]),
        launch_arguments={'use_rviz': 'True'}.items()
    )

    foxglove_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource([
            os.path.join(get_package_share_directory('foxglove_bridge'), 'launch', 'foxglove_bridge_launch.xml')
        ]),
        launch_arguments={'port': '8765'}.items()
    )

    # tf_bridge = Node(
    #     package='tf_ros',
    #     executable='static_transform_publisher',
    #     name='static_tf_aft_mapped_to_imu',
    #     arguments=['0','0','0','0','0','0','aft_mapped','imu_link']
    # )

    return LaunchDescription([
        sensor_launch,
        slam_launch,
        foxglove_launch
        # tf_bridge
    ])