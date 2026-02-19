import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import yaml

from launch.actions import TimerAction
from launch.event_handlers import OnProcessStart

def generate_launch_description():
    """Launch all the nodes for the rover"""

    # Define all the launch arguments
    ns = LaunchConfiguration('namespace')
    joy_dev = LaunchConfiguration('joy_dev')
    joy_deadzone = LaunchConfiguration('joy_deadzone')
    joy_autorepeat = LaunchConfiguration('joy_autorepeat')

    gamepad_cfg = LaunchConfiguration('gamepad_config')
    mixer_cfg   = LaunchConfiguration('mixer_config')
    driver_cfg  = LaunchConfiguration('driver_config')
    odometry_cfg = LaunchConfiguration('odometry_config')

    pkg_share = FindPackageShare(package='rover_description').find('rover_description')
    default_model_path = os.path.join(pkg_share, 'src', 'description', 'rover_description_v2.urdf')
    print("default_model_path: ", default_model_path)

    decls = [
        DeclareLaunchArgument('namespace', default_value='',
                              description='Namespace for the rover stack'),
        DeclareLaunchArgument('joy_dev', default_value='/dev/input/js0',
                              description='Joystick device path'),
        DeclareLaunchArgument('joy_deadzone', default_value='0.06',
                              description='Deadzone for joy_node'),
        DeclareLaunchArgument('joy_autorepeat', default_value='60.0',
                              description='Autorepeat rate (Hz) for joy_node'),

        DeclareLaunchArgument(
            'gamepad_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('gamepad_adapter'), 'config', 'gamepad_adapter.yaml'
            ]),
            description='gamepad_adapter parameters YAML'
        ),
        DeclareLaunchArgument(
            'mixer_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('arcade_mixer'), 'config', 'arcade_mixer.yaml'
            ]),
            description='arcade_mixer parameters YAML'
        ),
        DeclareLaunchArgument(
            'driver_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('tb6612_driver'), 'config', 'tb6612_driver.yaml'
            ]),
            description='tb6612_driver parameters YAML'
        ),
        DeclareLaunchArgument(
            'odometry_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('rover_odometry'), 'config', 'rover_odometry.yaml'
            ]),
            description='rover_odometry parameters YAML'
        ),
        DeclareLaunchArgument(name='model', default_value=default_model_path, description='Absolute path to robot model file')
    ]

    # Define all the nodes
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        namespace=ns,
        output='screen',
        parameters=[{
            'dev': joy_dev,
            'deadzone': joy_deadzone,
            'autorepeat_rate': joy_autorepeat,
        }],
        respawn=True,
    )

    gamepad_adapter = Node(
        package='gamepad_adapter',
        executable='gamepad_adapter',
        name='adapter_node',
        namespace=ns,
        output='screen',
        parameters=[gamepad_cfg],
        respawn=True,
    )

    arcade_mixer = Node(
        package='arcade_mixer',
        executable='arcade_mixer',
        name='arcade_mixer_node',
        namespace=ns,
        output='screen',
        parameters=[mixer_cfg],
        respawn=True,
    )

    tb6612_driver = Node(
        package='tb6612_driver',
        executable='tb6612_driver',
        name='tb6612_driver_node',
        namespace=ns,
        output='screen',
        parameters=[driver_cfg],
        respawn=True,
    )

    # Odometry nodes

    # Load odometry config for parameters and transforms
    rover_odometry_dir = get_package_share_directory('rover_odometry')
    odometry_config_path = os.path.join(rover_odometry_dir, 'config', 'rover_odometry.yaml')
    
    # Load config to extract parameters and transform values
    with open(odometry_config_path, 'r') as f:
        odometry_config = yaml.safe_load(f)
    
    # Extract encoder_odom parameters (ROS2 can't parse files with multiple top-level keys)
    odometry_params = odometry_config['odometry']['ros__parameters']
    # transforms_config = odometry_config['transforms']

    odometry_node = Node(
        package='rover_odometry',
        executable='odometry',
        name='odometry',
        output='screen',
        parameters=[odometry_params],
        respawn=True,
    )

    # Static transform publishers
    # laser_tf = transforms_config['base_to_laser']
    # base_to_laser_tf = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='base_to_laser_tf',
    #     arguments=[
    #         str(laser_tf['x']), str(laser_tf['y']), str(laser_tf['z']),
    #         str(laser_tf['roll']), str(laser_tf['pitch']), str(laser_tf['yaw']),
    #         laser_tf['parent_frame'], laser_tf['child_frame']
    #     ]
    # )

    # camera_tf = transforms_config['base_to_camera']
    # base_to_camera_tf = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='base_to_camera_tf',
    #     arguments=[
    #         str(camera_tf['x']), str(camera_tf['y']), str(camera_tf['z']),
    #         str(camera_tf['yaw']), str(camera_tf['pitch']), str(camera_tf['roll']),
    #         camera_tf['parent_frame'], camera_tf['child_frame']
    #     ]
    # )

    # Sllidar launch file
    sllidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('sllidar_ros2'),
                'launch',
                'view_sllidar_a2m12_launch.py'
            ])
        ]),
        launch_arguments={
            'serial_port': '/dev/tty_rover_lidar'
        }.items()
    )

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}],
    )

    # SLAM Toolbox launch file
    slam_params_file = PathJoinSubstitution([
        FindPackageShare('bringup'),
        'config',
        'slam_toolbox_async.yaml'
    ])
    
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('slam_toolbox'),
                'launch',
                'online_async_launch.py'
            ])
        ]),
        launch_arguments={
            'slam_params_file': slam_params_file
        }.items()
    )

    # --- ADDED NAV2 PARAMETERS ---
    nav2_params_path = '/home/rmahajani/Documents/projects/rover/ros2_ws/src/rover_nav/config/nav2_params.yaml'
    
    # Define the Nav2 launch inclusion
    nav2_navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            ])
        ]),
        launch_arguments={
            'params_file': nav2_params_path
        }.items()
    )
    

    # Define the launch plan
    return LaunchDescription(decls + [
        # joy_node,
        # gamepad_adapter,
        # arcade_mixer,
        # tb6612_driver,
        odometry_node,
        sllidar_launch,
        joint_state_publisher_node,
        robot_state_publisher_node,
        # slam_toolbox_launch,
        # nav2_navigation_launch
    ])