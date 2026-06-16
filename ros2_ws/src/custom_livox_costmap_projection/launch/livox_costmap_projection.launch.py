import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory("custom_livox_costmap_projection")
    config_file = os.path.join(pkg_dir, "config", "livox_costmap_projection.yaml")

    return LaunchDescription([
        Node(
            package="custom_livox_costmap_projection",
            executable="livox_costmap_projection_node",
            name="livox_costmap_projection_node",
            output="screen",
            parameters=[config_file],
        )
    ])
