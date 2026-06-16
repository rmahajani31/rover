import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_dir = get_package_share_directory("bringup")
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")

    # Pi-side bringup: FAST-LIO2 odometry, AMCL scan, and custom obstacle cloud
    # arrive from Jetson. AMCL uses /scan_from_livox; costmaps use
    # /custom/obstacle_cloud.
    default_params = os.path.join(
        bringup_dir,
        "config",
        "nav2_params_fast_lio2_costmap_projection.yaml",
    )
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
            description="Absolute path to the Nav2 parameters file for custom costmap projection.",
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
            PythonLaunchDescriptionSource(localization_launch),
            # AMCL provides map -> odom; FAST-LIO2 adapter provides odom -> base_link.
            launch_arguments={
                "map": map_yaml,
                "use_sim_time": use_sim_time,
                "autostart": autostart,
                "params_file": params_file,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(navigation_launch),
            # Do not launch rover_odometry here; /nav2_odom is supplied by FAST-LIO2.
            launch_arguments={
                "use_sim_time": use_sim_time,
                "autostart": autostart,
                "params_file": params_file,
            }.items(),
        ),
    ])
