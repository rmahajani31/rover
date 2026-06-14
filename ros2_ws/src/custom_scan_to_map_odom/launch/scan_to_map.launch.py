import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory("custom_scan_to_map_odom")
    config_file = os.path.join(pkg_dir, "config", "scan_to_map.yaml")
    # Launch-time override lets the same config run with or without odom -> base_link TF.
    publish_tf = LaunchConfiguration("publish_tf")

    return LaunchDescription([
        DeclareLaunchArgument(
            "publish_tf",
            default_value="false",
            description="Broadcast odom -> base_link TF for primary Nav2 odometry use.",
        ),
        Node(
            package="custom_scan_to_map_odom",
            executable="custom_scan_to_map_odom_node",
            name="custom_scan_to_map_odom",
            output="screen",
            parameters=[
                config_file,
                {"publish_tf": publish_tf},
            ],
        )
    ])
