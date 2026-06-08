import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory("custom_fastlio_preprocess")
    config_file = os.path.join(pkg_dir, "config", "preprocess.yaml")

    return LaunchDescription([
        Node(
            package="custom_fastlio_preprocess",
            executable="preprocess_node",
            name="custom_fastlio_preprocess",
            output="screen",
            parameters=[config_file],
        )
    ])
