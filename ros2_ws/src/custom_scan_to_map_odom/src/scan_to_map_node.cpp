#include "custom_scan_to_map_odom/scan_to_map_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pcl/common/point_tests.h>
#include <pcl/filters/voxel_grid.h>
#include <tf2/exceptions.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include "custom_scan_to_map_odom/se3_utils.hpp"

using std::placeholders::_1;

namespace custom_scan_to_map_odom
{

namespace
{

void updateCloudLayout(const CloudTPtr& cloud)
{
  cloud->width = static_cast<std::uint32_t>(cloud->points.size());
  cloud->height = 1;
  cloud->is_dense = false;
}

double elapsedMilliseconds(
  const std::chrono::steady_clock::time_point& start,
  const std::chrono::steady_clock::time_point& end)
{
  return std::chrono::duration<double, std::milli>(end - start).count();
}

}  // namespace

ScanToMapNode::ScanToMapNode(const rclcpp::NodeOptions& options)
: Node("custom_scan_to_map_odom", options)
{
  declareParameters();
  readParameters();

  optimizer_ = std::make_unique<ScanToMapOptimizer>(optimizerOptionsFromParameters());

  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    input_topic_,
    rclcpp::SensorDataQoS(),
    std::bind(&ScanToMapNode::cloudCallback, this, _1));

  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);
  local_map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(local_map_topic_, 10);

  if (publish_path_) {
    path_pub_ = create_publisher<nav_msgs::msg::Path>(path_topic_, 10);
  }

  if (publish_diagnostics_) {
    diagnostics_pub_ =
      create_publisher<diagnostic_msgs::msg::DiagnosticArray>(diagnostics_topic_, 10);
  }

  path_msg_.header.frame_id = odom_frame_;

  if (publish_tf_) {
    RCLCPP_WARN(
      get_logger(),
      "publish_tf is true. Verify no other node is publishing %s -> %s before using Nav2.",
      odom_frame_.c_str(),
      base_frame_.c_str());
  }

  if (!rebuild_kdtree_every_frame_) {
    RCLCPP_WARN(
      get_logger(),
      "rebuild_kdtree_every_frame is false, but the current LocalMap backend requires a rebuild "
      "after map updates. The node will rebuild the k-d tree every frame.");
  }

  RCLCPP_INFO(get_logger(), "custom_scan_to_map_odom initialized");
}

void ScanToMapNode::declareParameters()
{
  declare_parameter<std::string>("input_topic", "/custom/points_preprocessed");
  declare_parameter<std::string>("odom_topic", "/custom/scan_to_map_odom");
  declare_parameter<std::string>("path_topic", "/custom/scan_to_map_path");
  declare_parameter<std::string>("local_map_topic", "/custom/local_map");
  declare_parameter<std::string>("diagnostics_topic", "/custom/scan_to_map_diagnostics");

  declare_parameter<std::string>("odom_frame", "odom");
  declare_parameter<std::string>("base_frame", "base_link");
  declare_parameter<std::string>("lidar_frame", "livox_frame");
  declare_parameter<bool>("publish_tf", false);

  declare_parameter<double>("scan_voxel_leaf_size", 0.20);
  declare_parameter<double>("min_range", 0.30);
  declare_parameter<double>("max_range", 30.0);
  declare_parameter<int>("max_points_per_scan", 3000);

  declare_parameter<int>("max_iterations", 5);
  declare_parameter<int>("min_valid_correspondences", 100);
  declare_parameter<double>("convergence_translation_epsilon", 0.001);
  declare_parameter<double>("convergence_rotation_epsilon", 0.001);
  declare_parameter<double>("max_pose_update_translation", 0.50);
  declare_parameter<double>("max_pose_update_rotation_deg", 10.0);

  declare_parameter<int>("k_neighbors", 5);
  declare_parameter<double>("max_neighbor_distance", 1.0);
  declare_parameter<double>("max_plane_error", 0.10);
  declare_parameter<double>("max_point_to_plane_residual", 0.50);
  declare_parameter<double>("min_plane_eigen_ratio", 5.0);

  declare_parameter<double>("map_voxel_leaf_size", 0.20);
  declare_parameter<double>("local_map_half_size_x", 20.0);
  declare_parameter<double>("local_map_half_size_y", 20.0);
  declare_parameter<double>("local_map_half_size_z", 4.0);
  declare_parameter<int>("max_map_points", 300000);
  declare_parameter<bool>("rebuild_kdtree_every_frame", true);

  declare_parameter<int>("publish_local_map_every_n_frames", 5);
  declare_parameter<bool>("publish_path", true);
  declare_parameter<bool>("publish_diagnostics", true);
}

void ScanToMapNode::readParameters()
{
  input_topic_ = get_parameter("input_topic").as_string();
  odom_topic_ = get_parameter("odom_topic").as_string();
  path_topic_ = get_parameter("path_topic").as_string();
  local_map_topic_ = get_parameter("local_map_topic").as_string();
  diagnostics_topic_ = get_parameter("diagnostics_topic").as_string();

  odom_frame_ = get_parameter("odom_frame").as_string();
  base_frame_ = get_parameter("base_frame").as_string();
  lidar_frame_ = get_parameter("lidar_frame").as_string();
  publish_tf_ = get_parameter("publish_tf").as_bool();

  scan_voxel_leaf_size_ = get_parameter("scan_voxel_leaf_size").as_double();
  min_range_ = get_parameter("min_range").as_double();
  max_range_ = get_parameter("max_range").as_double();
  max_points_per_scan_ = get_parameter("max_points_per_scan").as_int();

  max_iterations_ = get_parameter("max_iterations").as_int();
  min_valid_correspondences_ = get_parameter("min_valid_correspondences").as_int();
  convergence_translation_epsilon_ =
    get_parameter("convergence_translation_epsilon").as_double();
  convergence_rotation_epsilon_ =
    get_parameter("convergence_rotation_epsilon").as_double();
  max_pose_update_translation_ = get_parameter("max_pose_update_translation").as_double();
  max_pose_update_rotation_deg_ = get_parameter("max_pose_update_rotation_deg").as_double();

  k_neighbors_ = get_parameter("k_neighbors").as_int();
  max_neighbor_distance_ = get_parameter("max_neighbor_distance").as_double();
  max_plane_error_ = get_parameter("max_plane_error").as_double();
  max_point_to_plane_residual_ = get_parameter("max_point_to_plane_residual").as_double();
  min_plane_eigen_ratio_ = get_parameter("min_plane_eigen_ratio").as_double();

  map_voxel_leaf_size_ = get_parameter("map_voxel_leaf_size").as_double();
  local_map_half_size_x_ = get_parameter("local_map_half_size_x").as_double();
  local_map_half_size_y_ = get_parameter("local_map_half_size_y").as_double();
  local_map_half_size_z_ = get_parameter("local_map_half_size_z").as_double();
  max_map_points_ = get_parameter("max_map_points").as_int();
  rebuild_kdtree_every_frame_ = get_parameter("rebuild_kdtree_every_frame").as_bool();

  publish_local_map_every_n_frames_ =
    get_parameter("publish_local_map_every_n_frames").as_int();
  publish_path_ = get_parameter("publish_path").as_bool();
  publish_diagnostics_ = get_parameter("publish_diagnostics").as_bool();
}

void ScanToMapNode::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  const auto callback_start = std::chrono::steady_clock::now();

  ScanToMapDiagnostics diagnostics;
  diagnostics.map_initialized = local_map_.isInitialized();

  CloudTPtr raw_cloud = fromRosCloud(*msg);
  diagnostics.input_points = raw_cloud->size();

  CloudTPtr filtered_scan = filterScan(raw_cloud);
  diagnostics.downsampled_points = filtered_scan->size();

  if (filtered_scan->empty()) {
    diagnostics.optimization.status = "empty_filtered_scan";
    diagnostics.map_points = local_map_.size();

    if (publish_diagnostics_) {
      publishDiagnostics(msg->header, diagnostics);
    }

    return;
  }

  if (!local_map_.isInitialized()) {
    RCLCPP_INFO(
      get_logger(),
      "First scan callback: filtered_points=%zu",
      filtered_scan->size());

    current_pose_ = Eigen::Isometry3d::Identity();

    CloudTPtr first_scan_map = transformCloud(filtered_scan, current_pose_);
    RCLCPP_INFO(
      get_logger(),
      "First scan transformed: map_frame_points=%zu",
      first_scan_map->size());

    local_map_.initialize(first_scan_map);
    RCLCPP_INFO(
      get_logger(),
      "Local map initialized: points=%zu",
      local_map_.size());

    local_map_.downsample(map_voxel_leaf_size_);
    RCLCPP_INFO(
      get_logger(),
      "Local map downsampled: points=%zu",
      local_map_.size());

    local_map_.rebuildKdTree();
    RCLCPP_INFO(get_logger(), "Local map k-d tree built");

    diagnostics.map_initialized = true;
    diagnostics.map_points = local_map_.size();
    diagnostics.optimization.success = true;
    diagnostics.optimization.status = "map_initialized";
    diagnostics.optimization.input_points = filtered_scan->size();

    publishOdometry(msg->header, current_pose_, diagnostics.optimization);
    RCLCPP_INFO(get_logger(), "Initial odometry published");

    publishLocalMap(msg->header);
    RCLCPP_INFO(get_logger(), "Initial local map published");

    if (publish_diagnostics_) {
      publishDiagnostics(msg->header, diagnostics);
      RCLCPP_INFO(get_logger(), "Initial diagnostics published");
    }

    RCLCPP_INFO(
      get_logger(),
      "Initialized local map with %zu points",
      local_map_.size());

    ++frame_count_;
    return;
  }

  OptimizationStats stats;
  Eigen::Isometry3d optimized_pose = current_pose_;

  const auto optimization_start = std::chrono::steady_clock::now();
  const bool optimization_ok = optimizer_->optimize(
    filtered_scan,
    local_map_,
    current_pose_,
    optimized_pose,
    stats);
  const auto optimization_end = std::chrono::steady_clock::now();

  diagnostics.optimization_time_ms =
    elapsedMilliseconds(optimization_start, optimization_end);
  diagnostics.optimization = stats;

  if (optimization_ok) {
    current_pose_ = optimized_pose;

    const auto map_update_start = std::chrono::steady_clock::now();

    CloudTPtr scan_map = transformCloud(filtered_scan, current_pose_);
    local_map_.insertCloud(scan_map);

    if (local_map_.size() > static_cast<std::size_t>(max_map_points_)) {
      local_map_.downsample(map_voxel_leaf_size_);
    }

    const Eigen::Vector3d half_size(
      local_map_half_size_x_,
      local_map_half_size_y_,
      local_map_half_size_z_);
    local_map_.cropAround(current_pose_.translation(), half_size);
    local_map_.downsample(map_voxel_leaf_size_);

    local_map_.rebuildKdTree();

    const auto map_update_end = std::chrono::steady_clock::now();
    diagnostics.map_update_time_ms =
      elapsedMilliseconds(map_update_start, map_update_end);
  } else {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "Scan-to-map optimization rejected: %s",
      stats.status.c_str());
  }

  diagnostics.map_initialized = local_map_.isInitialized();
  diagnostics.map_points = local_map_.size();

  publishOdometry(msg->header, current_pose_, stats);

  if (publish_local_map_every_n_frames_ > 0 &&
      frame_count_ % static_cast<std::size_t>(publish_local_map_every_n_frames_) == 0) {
    publishLocalMap(msg->header);
  }

  if (publish_diagnostics_) {
    publishDiagnostics(msg->header, diagnostics);
  }

  const auto callback_end = std::chrono::steady_clock::now();

  RCLCPP_INFO_THROTTLE(
    get_logger(),
    *get_clock(),
    1000,
    "scan-to-map %s | points=%zu | map=%zu | corr=%zu | residual=%.4f | time=%.2f ms",
    stats.status.c_str(),
    filtered_scan->size(),
    local_map_.size(),
    stats.valid_correspondences,
    stats.mean_residual,
    elapsedMilliseconds(callback_start, callback_end));

  ++frame_count_;
}

CloudTPtr ScanToMapNode::filterScan(const CloudTConstPtr& cloud) const
{
  CloudTPtr range_filtered(new CloudT());

  if (!cloud || cloud->empty()) {
    return range_filtered;
  }

  const double min_range_sq = min_range_ * min_range_;
  const double max_range_sq = max_range_ * max_range_;

  range_filtered->points.reserve(cloud->points.size());

  for (const auto& point : cloud->points) {
    if (!pcl::isFinite(point)) {
      continue;
    }

    const double range_sq =
      static_cast<double>(point.x) * static_cast<double>(point.x) +
      static_cast<double>(point.y) * static_cast<double>(point.y) +
      static_cast<double>(point.z) * static_cast<double>(point.z);

    if (range_sq < min_range_sq || range_sq > max_range_sq) {
      continue;
    }

    range_filtered->points.push_back(point);
  }

  updateCloudLayout(range_filtered);

  CloudTPtr voxel_filtered(new CloudT());

  if (scan_voxel_leaf_size_ > 0.0 && !range_filtered->empty()) {
    pcl::VoxelGrid<PointT> voxel;
    voxel.setInputCloud(range_filtered);
    voxel.setLeafSize(
      static_cast<float>(scan_voxel_leaf_size_),
      static_cast<float>(scan_voxel_leaf_size_),
      static_cast<float>(scan_voxel_leaf_size_));
    voxel.filter(*voxel_filtered);
  } else {
    voxel_filtered = range_filtered;
  }

  updateCloudLayout(voxel_filtered);

  if (max_points_per_scan_ <= 0 ||
      voxel_filtered->size() <= static_cast<std::size_t>(max_points_per_scan_)) {
    return voxel_filtered;
  }

  CloudTPtr limited(new CloudT());
  limited->points.reserve(static_cast<std::size_t>(max_points_per_scan_));

  const double step =
    static_cast<double>(voxel_filtered->size()) / static_cast<double>(max_points_per_scan_);

  for (int i = 0; i < max_points_per_scan_; ++i) {
    const std::size_t index = std::min(
      static_cast<std::size_t>(std::floor(static_cast<double>(i) * step)),
      voxel_filtered->size() - 1);
    limited->points.push_back(voxel_filtered->points[index]);
  }

  updateCloudLayout(limited);
  return limited;
}

CloudTPtr ScanToMapNode::transformCloud(
  const CloudTConstPtr& cloud,
  const Eigen::Isometry3d& transform) const
{
  CloudTPtr transformed(new CloudT());

  if (!cloud || cloud->empty()) {
    return transformed;
  }

  transformed->points.reserve(cloud->points.size());

  for (const auto& point : cloud->points) {
    const Eigen::Vector3d p(
      static_cast<double>(point.x),
      static_cast<double>(point.y),
      static_cast<double>(point.z));

    const Eigen::Vector3d p_transformed = transform * p;

    PointT output = point;
    output.x = static_cast<float>(p_transformed.x());
    output.y = static_cast<float>(p_transformed.y());
    output.z = static_cast<float>(p_transformed.z());

    transformed->points.push_back(output);
  }

  updateCloudLayout(transformed);
  return transformed;
}

bool ScanToMapNode::lookupLidarToBaseTransform(
  const rclcpp::Time& stamp,
  Eigen::Isometry3d& T_lidar_base)
{
  ensureTfListener();

  try {
    const geometry_msgs::msg::TransformStamped transform =
      tf_buffer_->lookupTransform(
        lidar_frame_,
        base_frame_,
        stamp,
        rclcpp::Duration::from_seconds(0.05));

    T_lidar_base = tf2::transformToEigen(transform);
    return true;
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "Could not look up %s -> %s transform: %s",
      lidar_frame_.c_str(),
      base_frame_.c_str(),
      ex.what());
    return false;
  }
}

void ScanToMapNode::ensureTfListener()
{
  if (!tf_buffer_) {
    RCLCPP_INFO(get_logger(), "Creating TF buffer");
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  }

  if (!tf_listener_) {
    RCLCPP_INFO(get_logger(), "Creating TF listener");
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(
      *tf_buffer_,
      shared_from_this(),
      false);
  }
}

void ScanToMapNode::ensureTfBroadcaster()
{
  if (!tf_broadcaster_) {
    RCLCPP_INFO(get_logger(), "Creating TF broadcaster");
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(
      shared_from_this());
  }
}

void ScanToMapNode::publishOdometry(
  const std_msgs::msg::Header& header,
  const Eigen::Isometry3d& T_odom_lidar,
  const OptimizationStats& stats)
{
  Eigen::Isometry3d T_odom_child = T_odom_lidar;
  std::string child_frame = lidar_frame_;

  Eigen::Isometry3d T_lidar_base = Eigen::Isometry3d::Identity();

  RCLCPP_DEBUG(
    get_logger(),
    "Publishing odometry: attempting %s -> %s lookup",
    lidar_frame_.c_str(),
    base_frame_.c_str());

  if (lookupLidarToBaseTransform(rclcpp::Time(header.stamp), T_lidar_base)) {
    T_odom_child = T_odom_lidar * T_lidar_base;
    child_frame = base_frame_;
  }

  nav_msgs::msg::Odometry odom;
  odom.header.stamp = header.stamp;
  odom.header.frame_id = odom_frame_;
  odom.child_frame_id = child_frame;
  odom.pose.pose = toRosPose(T_odom_child);

  const double position_covariance = stats.success ? 0.05 : 0.25;
  const double orientation_covariance = stats.success ? 0.10 : 0.50;

  odom.pose.covariance[0] = position_covariance;
  odom.pose.covariance[7] = position_covariance;
  odom.pose.covariance[14] = position_covariance;
  odom.pose.covariance[21] = orientation_covariance;
  odom.pose.covariance[28] = orientation_covariance;
  odom.pose.covariance[35] = orientation_covariance;

  odom_pub_->publish(odom);

  if (publish_tf_) {
    if (child_frame == base_frame_) {
      publishTransform(header, T_odom_child, child_frame);
    } else {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Skipping TF broadcast because odometry child frame is %s, not %s",
        child_frame.c_str(),
        base_frame_.c_str());
    }
  }

  if (publish_path_) {
    publishPath(header, odom);
  }
}

void ScanToMapNode::publishPath(
  const std_msgs::msg::Header& header,
  const nav_msgs::msg::Odometry& odom)
{
  if (!path_pub_) {
    return;
  }

  geometry_msgs::msg::PoseStamped pose_stamped;
  pose_stamped.header.stamp = header.stamp;
  pose_stamped.header.frame_id = odom_frame_;
  pose_stamped.pose = odom.pose.pose;

  path_msg_.header.stamp = header.stamp;
  path_msg_.header.frame_id = odom_frame_;
  path_msg_.poses.push_back(pose_stamped);

  path_pub_->publish(path_msg_);
}

void ScanToMapNode::publishLocalMap(const std_msgs::msg::Header& header)
{
  if (!local_map_pub_ || !local_map_.isInitialized()) {
    return;
  }

  std_msgs::msg::Header map_header = header;
  map_header.frame_id = odom_frame_;

  local_map_pub_->publish(toRosCloud(*local_map_.cloud(), map_header));
}

void ScanToMapNode::publishDiagnostics(
  const std_msgs::msg::Header& header,
  const ScanToMapDiagnostics& diagnostics)
{
  if (!diagnostics_pub_) {
    return;
  }

  diagnostics_pub_->publish(
    makeDiagnosticArray(diagnostics, header.stamp, "custom_scan_to_map_odom"));
}

void ScanToMapNode::publishTransform(
  const std_msgs::msg::Header& header,
  const Eigen::Isometry3d& T_odom_child,
  const std::string& child_frame)
{
  ensureTfBroadcaster();

  const geometry_msgs::msg::Pose pose = toRosPose(T_odom_child);

  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = header.stamp;
  transform.header.frame_id = odom_frame_;
  transform.child_frame_id = child_frame;
  transform.transform.translation.x = pose.position.x;
  transform.transform.translation.y = pose.position.y;
  transform.transform.translation.z = pose.position.z;
  transform.transform.rotation = pose.orientation;

  tf_broadcaster_->sendTransform(transform);
}

PlaneFitterOptions ScanToMapNode::planeFitterOptionsFromParameters() const
{
  PlaneFitterOptions options;
  options.max_plane_error = max_plane_error_;
  options.min_plane_eigen_ratio = min_plane_eigen_ratio_;
  return options;
}

ScanToMapOptimizerOptions ScanToMapNode::optimizerOptionsFromParameters() const
{
  ScanToMapOptimizerOptions options;
  options.max_iterations = max_iterations_;
  options.k_neighbors = k_neighbors_;
  options.min_valid_correspondences =
    static_cast<std::size_t>(std::max(0, min_valid_correspondences_));
  options.max_neighbor_distance = max_neighbor_distance_;
  options.max_point_to_plane_residual = max_point_to_plane_residual_;
  options.convergence_translation_epsilon = convergence_translation_epsilon_;
  options.convergence_rotation_epsilon = convergence_rotation_epsilon_;
  options.max_pose_update_translation = max_pose_update_translation_;
  options.max_pose_update_rotation_deg = max_pose_update_rotation_deg_;
  options.plane_fitter = planeFitterOptionsFromParameters();
  return options;
}

}  // namespace custom_scan_to_map_odom

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<custom_scan_to_map_odom::ScanToMapNode>());
  rclcpp::shutdown();
  return 0;
}
