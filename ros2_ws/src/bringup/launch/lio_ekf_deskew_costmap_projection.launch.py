import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    preprocess_dir = get_package_share_directory("custom_fastlio_preprocess")
    deskew_dir = get_package_share_directory("custom_lidar_deskew")
    lio_ekf_dir = get_package_share_directory("custom_lio_ekf")
    livox_cloud_to_scan_dir = get_package_share_directory("livox_cloud_to_scan")
    costmap_projection_dir = get_package_share_directory("custom_livox_costmap_projection")

    preprocess_config = os.path.join(preprocess_dir, "config", "preprocess.yaml")
    deskew_config = os.path.join(deskew_dir, "config", "lidar_deskew.yaml")
    lio_ekf_config = os.path.join(lio_ekf_dir, "config", "lio_ekf.yaml")
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

    publish_static_livox_tf = LaunchConfiguration("publish_static_livox_tf")
    start_preprocess = LaunchConfiguration("start_preprocess")
    start_lidar_deskew = LaunchConfiguration("start_lidar_deskew")
    start_costmap_projection = LaunchConfiguration("start_costmap_projection")
    start_scan_projection = LaunchConfiguration("start_scan_projection")

    deskew_input_topic = LaunchConfiguration("deskew_input_topic")
    deskew_output_topic = LaunchConfiguration("deskew_output_topic")
    costmap_projection_input_topic = LaunchConfiguration("costmap_projection_input_topic")
    lio_ekf_input_topic = LaunchConfiguration("lio_ekf_input_topic")

    return LaunchDescription([
        DeclareLaunchArgument(
            "publish_static_livox_tf",
            default_value="true",
            description="Publish the measured base_link -> livox_frame static transform.",
        ),
        DeclareLaunchArgument(
            "start_preprocess",
            default_value="true",
            description="Start custom_fastlio_preprocess before deskew and LIO EKF odometry.",
        ),
        DeclareLaunchArgument(
            "start_lidar_deskew",
            default_value="true",
            description="Start custom_lidar_deskew between preprocessing and LIO EKF odometry.",
        ),
        DeclareLaunchArgument(
            "start_costmap_projection",
            default_value="true",
            description="Start custom Livox obstacle cloud projection for Nav2 costmaps.",
        ),
        DeclareLaunchArgument(
            "start_scan_projection",
            default_value="true",
            description="Start Livox LaserScan projection for AMCL localization.",
        ),
        DeclareLaunchArgument(
            "deskew_input_topic",
            default_value="/custom/points_for_deskew",
            description="Timestamp-preserving PointCloud2 input topic for custom_lidar_deskew.",
        ),
        DeclareLaunchArgument(
            "deskew_output_topic",
            default_value="/custom/deskewed_points",
            description="Deskewed PointCloud2 output topic.",
        ),
        DeclareLaunchArgument(
            "lio_ekf_input_topic",
            default_value="/custom/deskewed_points",
            description="PointCloud2 input topic consumed by custom_lio_ekf.",
        ),
        DeclareLaunchArgument(
            "costmap_projection_input_topic",
            default_value="/custom/points_preprocessed",
            description=(
                "PointCloud2 input topic consumed by custom_livox_costmap_projection. "
                "Keep obstacle projection on the non-deskewed preprocessed cloud to avoid "
                "adding deskew latency to Nav2 collision monitoring."
            ),
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_link_to_livox_tf",
            condition=IfCondition(publish_static_livox_tf),
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
            condition=IfCondition(start_preprocess),
            parameters=[preprocess_config],
        ),
        Node(
            package="custom_lidar_deskew",
            executable="lidar_deskew_node",
            name="custom_lidar_deskew",
            output="screen",
            condition=IfCondition(start_lidar_deskew),
            parameters=[
                deskew_config,
                {
                    "lidar_topic": deskew_input_topic,
                    "output_topic": deskew_output_topic,
                },
            ],
        ),
        Node(
            package="custom_livox_costmap_projection",
            executable="livox_costmap_projection_node",
            name="livox_costmap_projection_node",
            output="screen",
            condition=IfCondition(start_costmap_projection),
            parameters=[
                costmap_projection_config,
                {
                    "input_cloud_topic": costmap_projection_input_topic,
                    # Nav2 consumes /custom/obstacle_cloud. The projected occupancy
                    # grid is debug-only here and needs exact odom-timestamp TF,
                    # which is too strict while the Phase 10 EKF is still slow.
                    "publish_occupancy_grid": False,
                },
            ],
        ),
        Node(
            package="livox_cloud_to_scan",
            executable="livox_cloud_to_scan_node",
            name="livox_cloud_to_scan_for_amcl",
            output="screen",
            condition=IfCondition(start_scan_projection),
            parameters=[
                livox_cloud_to_scan_config,
                {
                    "input_topic": "/custom/points_for_nav2",
                    "output_topic": "/scan_from_livox",
                    "target_frame": "base_link",
                },
            ],
        ),
        Node(
            package="custom_lio_ekf",
            executable="custom_lio_ekf_node",
            name="custom_lio_ekf",
            output="screen",
            parameters=[
                lio_ekf_config,
                {
                    "input_topic": lio_ekf_input_topic,
                    "imu_topic": "/livox/imu",
                    "odom_topic": "/nav2_odom",
                    "publish_tf": True,
                    "tf_publish_rate_hz": 20.0,
                    "publish_predicted_odom": False,
                    "stop_tf_on_tracking_degraded": False,
                    "imu_prediction.use_accel_translation": True,
                    "imu_prediction.calibrate_initial_imu": True,
                    "imu_frame": "livox_frame",
                    "lidar_frame": "livox_frame",
                },
            ],
        ),
    ])
