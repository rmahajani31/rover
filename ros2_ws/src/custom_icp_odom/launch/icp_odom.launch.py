import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory("custom_icp_odom")
    config_file = os.path.join(pkg_dir, "config", "icp_odom.yaml")

    return LaunchDescription([
        Node(
            package="custom_icp_odom",
            executable="custom_icp_odom_node",
            name="custom_icp_odom",
            output="screen",
            parameters=[config_file],
        )
    ])
