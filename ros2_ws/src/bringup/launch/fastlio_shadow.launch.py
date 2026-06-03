import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_dir = get_package_share_directory("bringup")
    default_fastlio_config = os.path.join(
        bringup_dir,
        "config",
        "mid360_fastlio2.yaml",
    )

    fastlio_package = LaunchConfiguration("fastlio_package")
    fastlio_launch_file = LaunchConfiguration("fastlio_launch_file")
    fastlio_config = LaunchConfiguration("fastlio_config")

    livox_cloud_to_scan_launch = PathJoinSubstitution([
        FindPackageShare("livox_cloud_to_scan"),
        "launch",
        "livox_cloud_to_scan.launch.py",
    ])

    fastlio_launch = PathJoinSubstitution([
        FindPackageShare(fastlio_package),
        "launch",
        fastlio_launch_file,
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            "fastlio_package",
            default_value="official_fast_lio2",
            description="ROS 2 package name for the FAST-LIO2 port.",
        ),
        DeclareLaunchArgument(
            "fastlio_launch_file",
            default_value="mapping_mid360.launch.py",
            description="FAST-LIO2 launch file to include from the package launch directory.",
        ),
        DeclareLaunchArgument(
            "fastlio_config",
            default_value=default_fastlio_config,
            description=(
                "MID-360 FAST-LIO2 config. If your FAST-LIO2 port uses a "
                "different launch argument name, adjust this wrapper after cloning it."
            ),
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_link_to_livox_tf",
            arguments=[
                "0.0", "0.0", "0.3175",
                "0", "0", "0",
                "base_link", "livox_frame",
            ],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(livox_cloud_to_scan_launch),
        ),
        GroupAction([
            SetRemap(src="/Odometry", dst="/fastlio_odom"),
            SetRemap(src="Odometry", dst="/fastlio_odom"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(fastlio_launch),
                launch_arguments={
                    "config_file": fastlio_config,
                }.items(),
            ),
        ]),
    ])
