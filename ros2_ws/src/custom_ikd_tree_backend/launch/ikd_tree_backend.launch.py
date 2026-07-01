from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # This launch file only exposes package parameters; consumers create the node.
    default_config = PathJoinSubstitution([
        FindPackageShare("custom_ikd_tree_backend"),
        "config",
        "ikd_tree_backend.yaml",
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=default_config,
            description="Path to the ikd-tree backend parameter file.",
        ),
    ])
