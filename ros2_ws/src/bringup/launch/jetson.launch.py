from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # The Jetson owns Livox preprocessing so the Pi can stay focused on
    # navigation, odometry, and micro-ROS communication.
    livox_cloud_to_scan_launch = PathJoinSubstitution([
        FindPackageShare("livox_cloud_to_scan"),
        "launch",
        "livox_cloud_to_scan.launch.py",
    ])

    return LaunchDescription([
        # Keep the Livox frame tied to the rover body frame before projecting
        # the 3D cloud into a planar scan.
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
            PythonLaunchDescriptionSource(livox_cloud_to_scan_launch)
        ),
    ])
