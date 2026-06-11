import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    preprocess_dir = get_package_share_directory("custom_fastlio_preprocess")
    icp_odom_dir = get_package_share_directory("custom_icp_odom")

    preprocess_config = os.path.join(preprocess_dir, "config", "preprocess.yaml")
    icp_odom_config = os.path.join(icp_odom_dir, "config", "icp_odom.yaml")

    publish_static_livox_tf = LaunchConfiguration("publish_static_livox_tf")
    start_preprocess = LaunchConfiguration("start_preprocess")

    return LaunchDescription([
        DeclareLaunchArgument(
            "publish_static_livox_tf",
            default_value="true",
            description="Publish the measured base_link -> livox_frame static transform.",
        ),
        DeclareLaunchArgument(
            "start_preprocess",
            default_value="true",
            description="Start custom_fastlio_preprocess before custom ICP odometry.",
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
            package="custom_icp_odom",
            executable="custom_icp_odom_node",
            name="custom_icp_odom",
            output="screen",
            parameters=[
                icp_odom_config,
                # This bringup is for shadow-mode validation only.
                {
                    "publish_tf": False,
                },
            ],
        ),
    ])
