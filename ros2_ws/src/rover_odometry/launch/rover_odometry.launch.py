import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory("rover_odometry")
    default_config_file = os.path.join(pkg_dir, "config", "rover_odometry_mapping.yaml")
    config_file = LaunchConfiguration("config_file")

    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=default_config_file,
            description="Absolute path to the rover_odometry parameters YAML file.",
        ),
        Node(
            package="rover_odometry",
            executable="odometry_node",
            name="odometry",
            output="screen",
            parameters=[config_file],
        ),
    ])
