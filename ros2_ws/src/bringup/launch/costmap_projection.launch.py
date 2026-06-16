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
    costmap_projection_dir = get_package_share_directory("custom_livox_costmap_projection")

    # Costmap projection stack: Livox driver is started separately in CustomMsg mode.
    # /custom/obstacle_cloud is the Nav2 costmap obstacle source. /scan_from_livox
    # is kept only as the LaserScan input AMCL needs for map localization.
    default_fastlio_config_path = os.path.join(bringup_dir, "config")
    adapter_config = os.path.join(adapter_dir, "config", "fastlio2_nav2_adapter.yaml")
    preprocess_config = os.path.join(preprocess_dir, "config", "preprocess.yaml")
    livox_cloud_to_scan_config = os.path.join(
        livox_cloud_to_scan_dir,
        "config",
        "livox_cloud_to_scan.yaml",
    )
    costmap_projection_config = os.path.join(
        costmap_projection_dir,
        "config",
        "livox_costmap_projection.yaml",
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
        Node(
            package="custom_fastlio_preprocess",
            executable="preprocess_node",
            name="custom_fastlio_preprocess",
            output="screen",
            parameters=[preprocess_config],
        ),
        Node(
            package="custom_livox_costmap_projection",
            executable="livox_costmap_projection_node",
            name="livox_costmap_projection_node",
            output="screen",
            parameters=[costmap_projection_config],
        ),
        Node(
            package="livox_cloud_to_scan",
            executable="livox_cloud_to_scan_node",
            name="livox_cloud_to_scan_for_amcl",
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
            parameters=[adapter_config],
        ),
    ])
