import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory("custom_lidar_deskew")
    config_file = os.path.join(pkg_dir, "config", "lidar_deskew.yaml")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulated time from /clock, usually for rosbag playback.",
        ),
        Node(
            package="custom_lidar_deskew",
            executable="lidar_deskew_node",
            name="custom_lidar_deskew",
            output="screen",
            parameters=[
                config_file,
                {"use_sim_time": use_sim_time},
            ],
        ),
    ])
