import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory("custom_lio_ekf")
    default_config_file = os.path.join(pkg_dir, "config", "lio_ekf.yaml")

    config_file = LaunchConfiguration("config_file")
    input_topic = LaunchConfiguration("input_topic")
    imu_topic = LaunchConfiguration("imu_topic")
    odom_topic = LaunchConfiguration("odom_topic")
    diagnostics_topic = LaunchConfiguration("diagnostics_topic")
    publish_tf = LaunchConfiguration("publish_tf")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=default_config_file,
            description="Path to the LIO EKF parameter file.",
        ),
        DeclareLaunchArgument(
            "input_topic",
            default_value="/custom/deskewed_points",
            description="Input deskewed LiDAR cloud topic.",
        ),
        DeclareLaunchArgument(
            "imu_topic",
            default_value="/livox/imu",
            description="Input sensor_msgs/msg/Imu topic.",
        ),
        DeclareLaunchArgument(
            "odom_topic",
            default_value="/custom/lio_ekf_odom",
            description="Corrected EKF LiDAR-inertial odometry topic.",
        ),
        DeclareLaunchArgument(
            "diagnostics_topic",
            default_value="/custom/lio_ekf_diagnostics",
            description="EKF diagnostics topic.",
        ),
        DeclareLaunchArgument(
            "publish_tf",
            default_value="false",
            description="Broadcast odom -> base_link TF for primary Nav2 odometry use.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulated time from /clock, usually for rosbag playback.",
        ),
        Node(
            package="custom_lio_ekf",
            executable="custom_lio_ekf_node",
            name="custom_lio_ekf",
            output="screen",
            parameters=[
                config_file,
                {
                    "input_topic": input_topic,
                    "imu_topic": imu_topic,
                    "odom_topic": odom_topic,
                    "diagnostics_topic": diagnostics_topic,
                    "publish_tf": publish_tf,
                    "use_sim_time": use_sim_time,
                },
            ],
        ),
    ])
