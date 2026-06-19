import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory("custom_imu_propagator")
    default_config_file = os.path.join(pkg_dir, "config", "imu_propagator.yaml")

    config_file = LaunchConfiguration("config_file")
    imu_topic = LaunchConfiguration("imu_topic")

    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=default_config_file,
            description="Path to the IMU propagator parameter file.",
        ),
        DeclareLaunchArgument(
            "imu_topic",
            default_value="/livox/imu",
            description="Input sensor_msgs/msg/Imu topic.",
        ),
        Node(
            package="custom_imu_propagator",
            executable="custom_imu_propagator_node",
            name="custom_imu_propagator",
            output="screen",
            parameters=[
                config_file,
                {"imu_topic": imu_topic},
            ],
        ),
    ])
