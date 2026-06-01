import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory("bringup")
    rover_odometry_dir = get_package_share_directory("rover_odometry")
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")

    # Navigation is Pi-side: odometry and the base transform are launched here,
    # while the Jetson is expected to already be publishing /scan_from_livox.
    default_params = os.path.join(bringup_dir, "config", "nav2_params_livox.yaml")
    odometry_launch = os.path.join(rover_odometry_dir, "launch", "rover_odometry.launch.py")
    odometry_config = os.path.join(rover_odometry_dir, "config", "rover_odometry.yaml")
    localization_launch = os.path.join(nav2_bringup_dir, "launch", "localization_launch.py")
    navigation_launch = os.path.join(nav2_bringup_dir, "launch", "navigation_launch.py")

    map_yaml = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")

    return LaunchDescription([
        DeclareLaunchArgument(
            "map",
            description="Absolute path to the map YAML file used by AMCL.",
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params,
            description="Absolute path to the Nav2 parameters file.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation time if true.",
        ),
        DeclareLaunchArgument(
            "autostart",
            default_value="true",
            description="Automatically startup the Nav2 stack.",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(odometry_launch),
            launch_arguments={
                "config_file": odometry_config,
            }.items(),
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_footprint_to_base_link_tf",
            # rover_odometry publishes odom -> base_footprint; Nav2 components
            # use base_link as the robot body frame.
            arguments=[
                "0", "0", "0",
                "0", "0", "0",
                "base_footprint", "base_link",
            ],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(localization_launch),
            # AMCL consumes the saved map passed at launch time.
            launch_arguments={
                "map": map_yaml,
                "use_sim_time": use_sim_time,
                "autostart": autostart,
                "params_file": params_file,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(navigation_launch),
            # The navigation stack reuses the same Livox-tuned Nav2 params as
            # localization so costmaps and controllers agree on frames/topics.
            launch_arguments={
                "use_sim_time": use_sim_time,
                "autostart": autostart,
                "params_file": params_file,
            }.items(),
        ),
    ])
