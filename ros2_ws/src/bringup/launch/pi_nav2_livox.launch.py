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

    default_params = os.path.join(bringup_dir, "config", "nav2_params_livox.yaml")
    odometry_launch = os.path.join(rover_odometry_dir, "launch", "rover_odometry.launch.py")
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
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_footprint_to_base_link_tf",
            arguments=[
                "0", "0", "0",
                "0", "0", "0",
                "base_footprint", "base_link",
            ],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(localization_launch),
            launch_arguments={
                "map": map_yaml,
                "use_sim_time": use_sim_time,
                "autostart": autostart,
                "params_file": params_file,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(navigation_launch),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "autostart": autostart,
                "params_file": params_file,
            }.items(),
        ),
    ])
