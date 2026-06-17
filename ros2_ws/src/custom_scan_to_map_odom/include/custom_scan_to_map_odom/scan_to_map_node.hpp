#pragma once

#include <cstddef>
#include <memory>
#include <mutex>
#include <string>

#include <Eigen/Geometry>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include "custom_scan_to_map_odom/diagnostics.hpp"
#include "custom_scan_to_map_odom/local_map_config.hpp"
#include "custom_scan_to_map_odom/local_map_manager.hpp"
#include "custom_scan_to_map_odom/ros_conversions.hpp"
#include "custom_scan_to_map_odom/scan_to_map_optimizer.hpp"

namespace custom_scan_to_map_odom
{

// ROS wrapper that filters scans, runs scan-to-map registration, maintains the
// local map, and publishes odometry/TF for Nav2.
class ScanToMapNode : public rclcpp::Node
{
public:
  explicit ScanToMapNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  void declareParameters();
  void readParameters();

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  CloudTPtr filterScan(const CloudTConstPtr& cloud) const;

  CloudTPtr transformCloud(
    const CloudTConstPtr& cloud,
    const Eigen::Isometry3d& transform) const;

  bool lookupLidarToBaseTransform(
    const rclcpp::Time& stamp,
    Eigen::Isometry3d& T_lidar_base);

  void ensureTfListener();
  void ensureTfBroadcaster();
  void publishLatestTransform();

  bool publishOdometry(
    const std_msgs::msg::Header& header,
    const Eigen::Isometry3d& T_odom_lidar,
    const OptimizationStats& stats);

  void publishPath(
    const std_msgs::msg::Header& header,
    const nav_msgs::msg::Odometry& odom);

  void publishLocalMap(const std_msgs::msg::Header& header);

  bool shouldPublishLocalMap(const std_msgs::msg::Header& header) const;

  void publishDiagnostics(
    const std_msgs::msg::Header& header,
    const ScanToMapDiagnostics& diagnostics);

  void publishTransform(
    const rclcpp::Time& stamp,
    const Eigen::Isometry3d& T_odom_child,
    const std::string& child_frame);

  ScanToMapOptimizerOptions optimizerOptionsFromParameters() const;
  PlaneFitterOptions planeFitterOptionsFromParameters() const;
  LocalMapConfig localMapConfigFromParameters() const;

  std::string input_topic_;
  std::string odom_topic_;
  std::string path_topic_;
  std::string local_map_topic_;
  std::string diagnostics_topic_;

  std::string odom_frame_;
  std::string base_frame_;
  std::string lidar_frame_;

  bool publish_tf_ = false;
  bool publish_path_ = true;
  bool publish_diagnostics_ = true;
  bool constrain_to_planar_ = true;
  bool stop_tf_on_tracking_degraded_ = true;
  double tf_publish_rate_hz_ = 20.0;
  int max_consecutive_tracking_failures_ = 3;
  int max_path_poses_ = 2000;

  double scan_voxel_leaf_size_ = 0.20;
  double min_range_ = 0.30;
  double max_range_ = 30.0;
  int max_points_per_scan_ = 3000;

  LocalMapConfig local_map_config_;

  int max_iterations_ = 5;
  int min_valid_correspondences_ = 100;

  double convergence_translation_epsilon_ = 0.001;
  double convergence_rotation_epsilon_ = 0.001;
  double max_pose_update_translation_ = 0.15;
  double max_pose_update_rotation_deg_ = 5.0;

  int k_neighbors_ = 5;
  double max_neighbor_distance_ = 1.0;
  double max_plane_error_ = 0.10;
  double max_point_to_plane_residual_ = 0.50;
  double min_plane_eigen_ratio_ = 5.0;

  std::size_t frame_count_ = 0;
  // Consecutive rejected frames mark odometry as degraded instead of pretending it is healthy.
  int consecutive_tracking_failures_ = 0;

  Eigen::Isometry3d current_pose_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d previous_odom_pose_ = Eigen::Isometry3d::Identity();
  // TF is published by a timer from the latest accepted pose to keep a steady TF cadence.
  Eigen::Isometry3d latest_T_odom_child_ = Eigen::Isometry3d::Identity();
  std::string latest_tf_child_frame_;
  std::string previous_odom_child_frame_;
  bool has_latest_tf_ = false;
  bool has_previous_odom_ = false;
  bool has_last_local_map_publish_stamp_ = false;
  rclcpp::Time previous_odom_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_local_map_publish_stamp_{0, 0, RCL_ROS_TIME};
  std::mutex latest_tf_mutex_;

  LocalMapManager local_map_manager_;
  std::unique_ptr<ScanToMapOptimizer> optimizer_;

  nav_msgs::msg::Path path_msg_;

  rclcpp::CallbackGroup::SharedPtr cloud_callback_group_;
  rclcpp::CallbackGroup::SharedPtr tf_callback_group_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr tf_timer_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr local_map_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
};

}  // namespace custom_scan_to_map_odom
