from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    """Launch joystick teleop nodes that publish TwistStamped on /cmd_vel."""

    ns = LaunchConfiguration('namespace')
    joy_dev = LaunchConfiguration('joy_dev')
    joy_deadzone = LaunchConfiguration('joy_deadzone')
    joy_autorepeat = LaunchConfiguration('joy_autorepeat')
    gamepad_cfg = LaunchConfiguration('gamepad_config')

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
    ]

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

    return LaunchDescription(decls + [
        joy_node,
        gamepad_adapter,
    ])
