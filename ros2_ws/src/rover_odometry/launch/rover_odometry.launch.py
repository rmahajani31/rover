import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory("rover_odometry")
    config_file = os.path.join(pkg_dir, "rover_odometry.yaml")

    return LaunchDescription([
        Node(
            package="rover_odometry",
            executable="odometry_node",
            name="odometry",
            output="screen",
            parameters=[config_file],
        ),
    ])