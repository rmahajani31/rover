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
    adapter_dir = get_package_share_directory("fastlio2_nav2_adapter")
    preprocess_dir = get_package_share_directory("custom_fastlio_preprocess")
    livox_cloud_to_scan_dir = get_package_share_directory("livox_cloud_to_scan")

    # Jetson-side bringup: Livox driver is started separately in CustomMsg mode.
    # Phase 3 keeps official FAST-LIO2 as the odometry source and replaces only
    # the obstacle-perception cloud path with custom_fastlio_preprocess.
    default_fastlio_config_path = os.path.join(bringup_dir, "config")
    adapter_config = os.path.join(adapter_dir, "config", "fastlio2_nav2_adapter.yaml")
    preprocess_config = os.path.join(preprocess_dir, "config", "preprocess.yaml")
    livox_cloud_to_scan_config = os.path.join(
        livox_cloud_to_scan_dir,
        "config",
        "livox_cloud_to_scan.yaml",
    )

    fastlio_package = LaunchConfiguration("fastlio_package")
    fastlio_launch_file = LaunchConfiguration("fastlio_launch_file")
    fastlio_config_path = LaunchConfiguration("fastlio_config_path")
    fastlio_config = LaunchConfiguration("fastlio_config")
    fastlio_rviz = LaunchConfiguration("fastlio_rviz")

    fastlio_launch = PathJoinSubstitution([
        FindPackageShare(fastlio_package),
        "launch",
        fastlio_launch_file,
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            "fastlio_package",
            default_value="fast_lio",
            description="ROS 2 package name for Ericsii/FAST_LIO_ROS2.",
        ),
        DeclareLaunchArgument(
            "fastlio_launch_file",
            default_value="mapping.launch.py",
            description="FAST-LIO2 launch file to include from the package launch directory.",
        ),
        DeclareLaunchArgument(
            "fastlio_config_path",
            default_value=default_fastlio_config_path,
            description="Directory containing the FAST-LIO2 YAML config.",
        ),
        DeclareLaunchArgument(
            "fastlio_config",
            default_value="mid360_fastlio2.yaml",
            description="FAST-LIO2 YAML config filename.",
        ),
        DeclareLaunchArgument(
            "fastlio_rviz",
            default_value="false",
            description="Let FAST-LIO2 start its own RViz instance if true.",
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_link_to_livox_tf",
            # Measured MID-360 mounting relative to the rover base frame.
            arguments=[
                "0.0", "0.0", "0.3175",
                "0", "0", "0",
                "base_link", "livox_frame",
            ],
        ),
        Node(
            package="custom_fastlio_preprocess",
            executable="preprocess_node",
            name="custom_fastlio_preprocess",
            output="screen",
            parameters=[preprocess_config],
        ),
        Node(
            package="livox_cloud_to_scan",
            executable="livox_cloud_to_scan_node",
            name="livox_cloud_to_scan",
            output="screen",
            parameters=[
                livox_cloud_to_scan_config,
                {
                    "input_topic": "/custom/points_for_nav2",
                    "output_topic": "/scan_from_livox",
                    "target_frame": "base_link",
                },
            ],
        ),
        GroupAction([
            # Keep the FAST-LIO2 odometry topic stable for the adapter and bags.
            SetRemap(src="/Odometry", dst="/fastlio_odom"),
            SetRemap(src="Odometry", dst="/fastlio_odom"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(fastlio_launch),
                launch_arguments={
                    "config_path": fastlio_config_path,
                    "config_file": fastlio_config,
                    "rviz": fastlio_rviz,
                }.items(),
            ),
        ]),
        Node(
            package="fastlio2_nav2_adapter",
            executable="fastlio2_nav2_adapter_node",
            name="fastlio2_nav2_adapter",
            output="screen",
            # Publishes /nav2_odom and, by default, odom -> base_link.
            parameters=[adapter_config],
        ),
    ])
