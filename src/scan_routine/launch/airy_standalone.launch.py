import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    rslidar_config = '/home/scanar/scanarMini/src/rslidar_sdk/config/drone_config.yaml'
    
    rslidar_node = Node(
        package='rslidar_sdk',
        executable='rslidar_sdk_node',
        name='rslidar_sdk_node',
        namespace='rslidar_sdk',
        output='screen',
        parameters=[{'config_path': rslidar_config}]
    )

    return LaunchDescription([
        rslidar_node
    ])
