import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory("livox_cloud_to_scan")
    config_file = os.path.join(pkg_dir, "config", "livox_cloud_to_scan.yaml")

    return LaunchDescription([
        Node(
            package="livox_cloud_to_scan",
            executable="livox_cloud_to_scan_node",
            name="livox_cloud_to_scan",
            output="screen",
            parameters=[config_file],
        )
    ])
