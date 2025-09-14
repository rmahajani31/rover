from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

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

    # Define the launch plan
    return LaunchDescription(decls + [
        joy_node,
        gamepad_adapter,
        arcade_mixer,
        tb6612_driver
    ])