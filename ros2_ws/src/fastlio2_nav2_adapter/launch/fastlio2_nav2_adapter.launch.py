import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("fastlio2_nav2_adapter")
    config_file = os.path.join(pkg_share, "config", "fastlio2_nav2_adapter.yaml")

    return LaunchDescription([
        Node(
            package="fastlio2_nav2_adapter",
            executable="fastlio2_nav2_adapter_node",
            name="fastlio2_nav2_adapter",
            output="screen",
            parameters=[config_file],
        ),
    ])
