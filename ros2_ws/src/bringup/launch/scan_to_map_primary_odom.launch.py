import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    preprocess_dir = get_package_share_directory("custom_fastlio_preprocess")
    scan_to_map_dir = get_package_share_directory("custom_scan_to_map_odom")
    livox_cloud_to_scan_dir = get_package_share_directory("livox_cloud_to_scan")

    preprocess_config = os.path.join(preprocess_dir, "config", "preprocess.yaml")
    scan_to_map_config = os.path.join(scan_to_map_dir, "config", "scan_to_map.yaml")
    livox_cloud_to_scan_config = os.path.join(
        livox_cloud_to_scan_dir,
        "config",
        "livox_cloud_to_scan.yaml",
    )

    publish_static_livox_tf = LaunchConfiguration("publish_static_livox_tf")
    start_preprocess = LaunchConfiguration("start_preprocess")
    start_scan_projection = LaunchConfiguration("start_scan_projection")

    return LaunchDescription([
        DeclareLaunchArgument(
            "publish_static_livox_tf",
            default_value="true",
            description="Publish the measured base_link -> livox_frame static transform.",
        ),
        DeclareLaunchArgument(
            "start_preprocess",
            default_value="true",
            description="Start custom_fastlio_preprocess before scan-to-map odometry.",
        ),
        DeclareLaunchArgument(
            "start_scan_projection",
            default_value="true",
            description="Start Livox cloud projection for Nav2 obstacle input.",
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_link_to_livox_tf",
            condition=IfCondition(publish_static_livox_tf),
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
            condition=IfCondition(start_preprocess),
            parameters=[preprocess_config],
        ),
        Node(
            package="livox_cloud_to_scan",
            executable="livox_cloud_to_scan_node",
            name="livox_cloud_to_scan",
            output="screen",
            condition=IfCondition(start_scan_projection),
            parameters=[
                livox_cloud_to_scan_config,
                {
                    "input_topic": "/custom/points_for_nav2",
                    "output_topic": "/scan_from_livox",
                    "target_frame": "base_link",
                },
            ],
        ),
        Node(
            package="custom_scan_to_map_odom",
            executable="custom_scan_to_map_odom_node",
            name="custom_scan_to_map_odom",
            output="screen",
            parameters=[
                scan_to_map_config,
                {
                    "publish_tf": True,
                    "odom_topic": "/nav2_odom",
                    "publish_diagnostics": False,
                    "tf_publish_rate_hz": 20.0,
                },
            ],
        ),
    ])
