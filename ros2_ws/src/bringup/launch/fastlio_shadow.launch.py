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
    default_fastlio_config_path = os.path.join(bringup_dir, "config")

    # Shadow mode uses Livox CustomMsg for FAST-LIO, so the normal PointCloud2
    # scan projector is fed through an explicit /livox/points conversion here.
    livox_cloud_to_scan_dir = get_package_share_directory("livox_cloud_to_scan")
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
            arguments=[
                "0.0", "0.0", "0.3175",
                "0", "0", "0",
                "base_link", "livox_frame",
            ],
        ),
        # This converter is intentionally launched only in FAST-LIO shadow mode.
        # Standard jetson.launch.py still expects /livox/lidar to be PointCloud2.
        Node(
            package="livox_cloud_to_scan",
            executable="livox_custom_msg_to_pointcloud2_node",
            name="livox_custom_msg_to_pointcloud2",
            output="screen",
            parameters=[
                livox_cloud_to_scan_config,
                {
                    "input_topic": "/livox/lidar",
                    "output_topic": "/livox/points",
                    "frame_id": "livox_frame",
                },
            ],
        ),
        Node(
            package="livox_cloud_to_scan",
            executable="livox_cloud_to_scan_node",
            name="livox_cloud_to_scan",
            output="screen",
            parameters=[
                livox_cloud_to_scan_config,
                {
                    "input_topic": "/livox/points",
                    "output_topic": "/scan_from_livox",
                    "target_frame": "base_link",
                },
            ],
        ),
        GroupAction([
            # Ericsii/FAST_LIO_ROS2 publishes Odometry by default; keep the
            # shadow odometry name stable for bags, RViz, and later comparison.
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
    ])
