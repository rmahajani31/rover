import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_dir = get_package_share_directory("bringup")
    rover_odometry_dir = get_package_share_directory("rover_odometry")

    rover_odometry_launch = os.path.join(
        rover_odometry_dir, "rover_odometry.launch.py"
    )
    slam_params_default = os.path.join(
        bringup_dir, "slam_toolbox_async.yaml"
    )

    ns = LaunchConfiguration("namespace")
    joy_dev = LaunchConfiguration("joy_dev")
    joy_deadzone = LaunchConfiguration("joy_deadzone")
    joy_autorepeat = LaunchConfiguration("joy_autorepeat")
    gamepad_cfg = LaunchConfiguration("gamepad_config")
    slam_params_file = LaunchConfiguration("slam_params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")

    decls = [
        DeclareLaunchArgument(
            "namespace",
            default_value="",
            description="Namespace for the rover stack",
        ),
        DeclareLaunchArgument(
            "joy_dev",
            default_value="/dev/input/js0",
            description="Joystick device path",
        ),
        DeclareLaunchArgument(
            "joy_deadzone",
            default_value="0.06",
            description="Deadzone for joy_node",
        ),
        DeclareLaunchArgument(
            "joy_autorepeat",
            default_value="60.0",
            description="Autorepeat rate (Hz) for joy_node",
        ),
        DeclareLaunchArgument(
            "gamepad_config",
            default_value=PathJoinSubstitution([
                FindPackageShare("gamepad_adapter"),
                "gamepad_adapter.yaml",
            ]),
            description="gamepad_adapter parameters YAML",
        ),
        DeclareLaunchArgument(
            "slam_params_file",
            default_value=slam_params_default,
            description="Absolute path to slam_toolbox parameters YAML",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation time if true",
        ),
    ]

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        namespace=ns,
        output="screen",
        parameters=[{
            "dev": joy_dev,
            "deadzone": joy_deadzone,
            "autorepeat_rate": joy_autorepeat,
        }],
        respawn=True,
    )

    gamepad_adapter = Node(
        package="gamepad_adapter",
        executable="gamepad_adapter",
        name="adapter_node",
        namespace=ns,
        output="screen",
        parameters=[gamepad_cfg],
        respawn=True,
    )

    odometry_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rover_odometry_launch),
    )

    base_footprint_to_base_link_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_footprint_to_base_link_tf",
        arguments=[
            "0", "0", "0",
            "0", "0", "0",
            "base_footprint", "base_link",
        ],
    )

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("slam_toolbox"),
                "launch",
                "online_async_launch.py",
            ])
        ]),
        launch_arguments={
            "slam_params_file": slam_params_file,
            "use_sim_time": use_sim_time,
        }.items(),
    )

    return LaunchDescription(decls + [
        joy_node,
        gamepad_adapter,
        odometry_launch,
        base_footprint_to_base_link_tf,
        slam_toolbox_launch,
    ])