from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    input_topic = LaunchConfiguration("input_topic")
    imu_topic = LaunchConfiguration("imu_topic")
    odom_topic = LaunchConfiguration("odom_topic")
    diagnostics_topic = LaunchConfiguration("diagnostics_topic")
    publish_tf = LaunchConfiguration("publish_tf")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription([
        DeclareLaunchArgument(
            "input_topic",
            default_value="/custom/deskewed_points",
            description="Input deskewed LiDAR cloud topic.",
        ),
        DeclareLaunchArgument(
            "imu_topic",
            default_value="/livox/imu",
            description="Input sensor_msgs/msg/Imu topic.",
        ),
        DeclareLaunchArgument(
            "odom_topic",
            default_value="/custom/lio_ekf_odom",
            description="Corrected EKF LiDAR-inertial odometry topic.",
        ),
        DeclareLaunchArgument(
            "diagnostics_topic",
            default_value="/custom/lio_ekf_diagnostics",
            description="EKF diagnostics topic.",
        ),
        DeclareLaunchArgument(
            "publish_tf",
            default_value="false",
            description="Broadcast odom -> base_link TF for primary Nav2 odometry use.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulated time from /clock, usually for rosbag playback.",
        ),
        Node(
            package="custom_lio_ekf",
            executable="custom_lio_ekf_node",
            name="custom_lio_ekf",
            output="screen",
            parameters=[{
                "input_topic": input_topic,
                "imu_topic": imu_topic,
                "odom_topic": odom_topic,
                "diagnostics_topic": diagnostics_topic,
                "publish_tf": publish_tf,
                "use_sim_time": use_sim_time,
            }],
        ),
    ])
