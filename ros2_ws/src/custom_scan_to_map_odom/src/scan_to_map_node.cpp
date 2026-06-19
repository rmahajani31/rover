#include "custom_scan_to_map_odom/scan_to_map_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

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

double degreesToRadians(double degrees)
{
  constexpr double kPi = 3.14159265358979323846;
  return degrees * kPi / 180.0;
}

double radiansToDegrees(double radians)
{
  constexpr double kPi = 3.14159265358979323846;
  return radians * 180.0 / kPi;
}

double yawFromTransform(const Eigen::Isometry3d& transform)
{
  return std::atan2(transform.linear()(1, 0), transform.linear()(0, 0));
}

// Nav2 consumes planar rover odometry, so strip roll, pitch, and z before publishing.
Eigen::Isometry3d makePlanarTransform(const Eigen::Isometry3d& transform)
{
  const double yaw = yawFromTransform(transform);

  Eigen::Isometry3d planar_transform = Eigen::Isometry3d::Identity();
  planar_transform.translation().x() = transform.translation().x();
  planar_transform.translation().y() = transform.translation().y();
  planar_transform.linear() =
    Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();

  return planar_transform;
}

}  // namespace

ScanToMapNode::ScanToMapNode(const rclcpp::NodeOptions& options)
: Node("custom_scan_to_map_odom", options)
{
  declareParameters();
  readParameters();

  local_map_config_ = localMapConfigFromParameters();
  local_map_manager_.configure(local_map_config_);

  optimizer_ = std::make_unique<ScanToMapOptimizer>(optimizerOptionsFromParameters());

  cloud_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  imu_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  tf_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  rclcpp::SubscriptionOptions cloud_subscription_options;
  cloud_subscription_options.callback_group = cloud_callback_group_;

  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    input_topic_,
    rclcpp::SensorDataQoS(),
    std::bind(&ScanToMapNode::cloudCallback, this, _1),
    cloud_subscription_options);

  if (use_imu_initial_guess_) {
    rclcpp::SubscriptionOptions imu_subscription_options;
    imu_subscription_options.callback_group = imu_callback_group_;

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&ScanToMapNode::imuCallback, this, _1),
      imu_subscription_options);

    RCLCPP_INFO(
      get_logger(),
      "IMU buffering enabled for scan-to-map initial guess: imu_topic=%s",
      imu_topic_.c_str());
  }

  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);
  if (publish_imu_predicted_odom_) {
    imu_predicted_odom_pub_ =
      create_publisher<nav_msgs::msg::Odometry>(imu_predicted_odom_topic_, 10);
  }
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

    if (tf_publish_rate_hz_ > 0.0) {
      const auto tf_period =
        std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / tf_publish_rate_hz_));

      tf_timer_ = create_wall_timer(
        tf_period,
        std::bind(&ScanToMapNode::publishLatestTransform, this),
        tf_callback_group_);

      RCLCPP_INFO(
        get_logger(),
        "Publishing %s -> %s TF at %.2f Hz from latest scan-to-map pose",
        odom_frame_.c_str(),
        base_frame_.c_str(),
        tf_publish_rate_hz_);
    } else {
      RCLCPP_WARN(get_logger(), "publish_tf is true but tf_publish_rate_hz is not positive");
    }
  }

  RCLCPP_INFO(get_logger(), "custom_scan_to_map_odom initialized");
}

void ScanToMapNode::declareParameters()
{
  declare_parameter<std::string>("input_topic", "/custom/points_preprocessed");
  declare_parameter<std::string>("imu_topic", "/livox/imu");
  declare_parameter<std::string>("odom_topic", "/custom/scan_to_map_odom");
  declare_parameter<std::string>("imu_predicted_odom_topic", "/custom/imu_predicted_odom");
  declare_parameter<std::string>("path_topic", "/custom/scan_to_map_path");
  declare_parameter<std::string>("local_map_topic", "/custom/local_map");
  declare_parameter<std::string>("diagnostics_topic", "/custom/scan_to_map_diagnostics");

  declare_parameter<std::string>("odom_frame", "odom");
  declare_parameter<std::string>("base_frame", "base_link");
  declare_parameter<std::string>("lidar_frame", "livox_frame");
  declare_parameter<bool>("publish_tf", false);
  declare_parameter<bool>("use_imu_initial_guess", true);
  declare_parameter<bool>("use_imu_rotation", true);
  declare_parameter<bool>("use_imu_translation", false);
  declare_parameter<bool>("publish_imu_prediction_debug", true);
  declare_parameter<bool>("publish_imu_predicted_odom", true);
  declare_parameter<bool>("constrain_to_planar", true);
  declare_parameter<bool>("stop_tf_on_tracking_degraded", true);
  declare_parameter<double>("tf_publish_rate_hz", 20.0);
  declare_parameter<int>("max_consecutive_tracking_failures", 3);

  declare_parameter<double>("scan_voxel_leaf_size", 0.20);
  declare_parameter<double>("min_range", 0.30);
  declare_parameter<double>("max_range", 30.0);
  declare_parameter<int>("max_points_per_scan", 3000);

  declare_parameter<int>("max_iterations", 5);
  declare_parameter<int>("min_valid_correspondences", 100);
  declare_parameter<double>("convergence_translation_epsilon", 0.001);
  declare_parameter<double>("convergence_rotation_epsilon", 0.001);
  declare_parameter<double>("max_pose_update_translation", 0.15);
  declare_parameter<double>("max_pose_update_rotation_deg", 5.0);

  declare_parameter<int>("k_neighbors", 5);
  declare_parameter<double>("max_neighbor_distance", 1.0);
  declare_parameter<double>("max_plane_error", 0.10);
  declare_parameter<double>("max_point_to_plane_residual", 0.50);
  declare_parameter<double>("min_plane_eigen_ratio", 5.0);

  declare_parameter<double>("max_imu_buffer_seconds", 5.0);
  declare_parameter<double>("max_allowed_imu_gap", 0.02);
  declare_parameter<double>("max_expected_yaw_change_deg_per_scan", 30.0);
  declare_parameter<std::vector<double>>("gravity", std::vector<double>{0.0, 0.0, -9.81});
  declare_parameter<std::vector<double>>("gyro_bias", std::vector<double>{0.0, 0.0, 0.0});
  declare_parameter<std::vector<double>>("accel_bias", std::vector<double>{0.0, 0.0, 0.0});
  declare_parameter<double>("imu_accel_scale", 9.80665);

  declare_parameter<double>("local_map.cube_size_x", 30.0);
  declare_parameter<double>("local_map.cube_size_y", 30.0);
  declare_parameter<double>("local_map.cube_size_z", 6.0);
  declare_parameter<double>("local_map.movement_threshold_xy", 5.0);
  declare_parameter<double>("local_map.movement_threshold_z", 2.0);
  declare_parameter<double>("local_map.voxel_leaf_size", 0.15);
  declare_parameter<bool>("local_map.publish_local_map", true);
  declare_parameter<double>("local_map.local_map_publish_period_sec", 1.0);

  declare_parameter<int>("max_path_poses", 2000);
  declare_parameter<bool>("publish_path", true);
  declare_parameter<bool>("publish_diagnostics", true);
}

void ScanToMapNode::readParameters()
{
  input_topic_ = get_parameter("input_topic").as_string();
  imu_topic_ = get_parameter("imu_topic").as_string();
  odom_topic_ = get_parameter("odom_topic").as_string();
  imu_predicted_odom_topic_ = get_parameter("imu_predicted_odom_topic").as_string();
  path_topic_ = get_parameter("path_topic").as_string();
  local_map_topic_ = get_parameter("local_map_topic").as_string();
  diagnostics_topic_ = get_parameter("diagnostics_topic").as_string();

  odom_frame_ = get_parameter("odom_frame").as_string();
  base_frame_ = get_parameter("base_frame").as_string();
  lidar_frame_ = get_parameter("lidar_frame").as_string();
  publish_tf_ = get_parameter("publish_tf").as_bool();
  use_imu_initial_guess_ = get_parameter("use_imu_initial_guess").as_bool();
  imu_options_.use_imu_rotation = get_parameter("use_imu_rotation").as_bool();
  imu_options_.use_imu_translation = get_parameter("use_imu_translation").as_bool();
  publish_imu_prediction_debug_ = get_parameter("publish_imu_prediction_debug").as_bool();
  publish_imu_predicted_odom_ = get_parameter("publish_imu_predicted_odom").as_bool();
  constrain_to_planar_ = get_parameter("constrain_to_planar").as_bool();
  stop_tf_on_tracking_degraded_ = get_parameter("stop_tf_on_tracking_degraded").as_bool();
  tf_publish_rate_hz_ = get_parameter("tf_publish_rate_hz").as_double();
  max_consecutive_tracking_failures_ =
    get_parameter("max_consecutive_tracking_failures").as_int();

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

  imu_options_.max_imu_buffer_seconds = get_parameter("max_imu_buffer_seconds").as_double();
  imu_options_.max_allowed_imu_gap = get_parameter("max_allowed_imu_gap").as_double();
  imu_options_.max_expected_yaw_change_deg_per_scan =
    get_parameter("max_expected_yaw_change_deg_per_scan").as_double();
  imu_options_.gravity =
    readVector3Parameter("gravity", Eigen::Vector3d(0.0, 0.0, -9.81));
  imu_options_.gyro_bias =
    readVector3Parameter("gyro_bias", Eigen::Vector3d::Zero());
  imu_options_.accel_bias =
    readVector3Parameter("accel_bias", Eigen::Vector3d::Zero());
  imu_accel_scale_ = get_parameter("imu_accel_scale").as_double();
  imu_propagator_.configure(imu_options_);

  max_path_poses_ = get_parameter("max_path_poses").as_int();
  publish_path_ = get_parameter("publish_path").as_bool();
  publish_diagnostics_ = get_parameter("publish_diagnostics").as_bool();
}

void ScanToMapNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  custom_imu_propagator::ImuSample sample;
  sample.stamp = msg->header.stamp;
  sample.gyro = Eigen::Vector3d(
    msg->angular_velocity.x,
    msg->angular_velocity.y,
    msg->angular_velocity.z);
  sample.accel = imu_accel_scale_ * Eigen::Vector3d(
    msg->linear_acceleration.x,
    msg->linear_acceleration.y,
    msg->linear_acceleration.z);

  // The propagator owns ordering and buffer trimming; the node only guards
  // concurrent access from the IMU and scan callbacks.
  std::size_t samples_received = 0;
  {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    imu_propagator_.addSample(sample);
    ++imu_samples_received_;
    samples_received = imu_samples_received_;
  }

  RCLCPP_DEBUG_THROTTLE(
    get_logger(),
    *get_clock(),
    2000,
    "Buffered IMU samples for scan-to-map: received=%zu gyro_norm=%.5f accel_norm=%.5f",
    samples_received,
    sample.gyro.norm(),
    sample.accel.norm());
}

Eigen::Vector3d ScanToMapNode::readVector3Parameter(
  const std::string& name,
  const Eigen::Vector3d& fallback) const
{
  const auto values = get_parameter(name).as_double_array();

  if (values.size() != 3) {
    RCLCPP_WARN(
      get_logger(),
      "Parameter %s must have exactly 3 values; using fallback [%.3f, %.3f, %.3f]",
      name.c_str(),
      fallback.x(),
      fallback.y(),
      fallback.z());
    return fallback;
  }

  return Eigen::Vector3d(values[0], values[1], values[2]);
}

Eigen::Isometry3d ScanToMapNode::initialGuessForScan(
  const std_msgs::msg::Header& header,
  const Eigen::Isometry3d& fallback_guess,
  ScanToMapDiagnostics& diagnostics)
{
  diagnostics.imu_initial_guess_enabled = use_imu_initial_guess_;

  if (!use_imu_initial_guess_ || !has_last_accepted_scan_stamp_) {
    diagnostics.imu_prediction_status = use_imu_initial_guess_ ?
      "no_last_accepted_scan_stamp" : "imu_initial_guess_disabled";
    return fallback_guess;
  }

  const rclcpp::Time current_scan_stamp(header.stamp);
  custom_imu_propagator::ImuPropagationResult imu_result;
  {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    imu_result =
      imu_propagator_.propagateBetween(last_accepted_scan_stamp_, current_scan_stamp);
  }

  if (publish_imu_prediction_debug_) {
    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      1000,
      "IMU initial guess: success=%s status=%s samples=%zu dt=%.4f yaw=%.3f deg",
      imu_result.success ? "true" : "false",
      imu_result.status.c_str(),
      imu_result.samples_used,
      imu_result.dt_total,
      imu_result.delta_yaw_deg);
  }

  diagnostics.imu_prediction_success = imu_result.success;
  diagnostics.imu_prediction_status = imu_result.status;
  diagnostics.imu_samples_used = imu_result.samples_used;
  diagnostics.imu_dt_total = imu_result.dt_total;
  diagnostics.imu_delta_roll_deg = imu_result.delta_roll_deg;
  diagnostics.imu_delta_pitch_deg = imu_result.delta_pitch_deg;
  diagnostics.imu_delta_yaw_deg = imu_result.delta_yaw_deg;

  if (!imu_result.success) {
    return fallback_guess;
  }

  // Phase 8 composes the IMU delta onto the previous accepted LiDAR pose. The
  // optimizer still owns the final odometry correction.
  const Eigen::Isometry3d imu_guess = fallback_guess * imu_result.delta_T;
  diagnostics.used_imu_guess = true;
  publishImuPredictedOdometry(header, imu_guess);
  return imu_guess;
}

void ScanToMapNode::updateLastAcceptedScanStamp(const rclcpp::Time& stamp)
{
  // Rejected scans are intentionally excluded so the next IMU interval starts
  // from the last pose that actually updated the map/odometry state.
  last_accepted_scan_stamp_ = stamp;
  has_last_accepted_scan_stamp_ = true;
}

void ScanToMapNode::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  const auto callback_start = std::chrono::steady_clock::now();

  ScanToMapDiagnostics diagnostics;
  diagnostics.map_initialized = local_map_manager_.isInitialized();
  diagnostics.imu_initial_guess_enabled = use_imu_initial_guess_;

  CloudTPtr raw_cloud = fromRosCloud(*msg);
  diagnostics.input_points = raw_cloud->size();

  CloudTPtr filtered_scan = filterScan(raw_cloud);
  diagnostics.downsampled_points = filtered_scan->size();

  if (filtered_scan->empty()) {
    diagnostics.optimization.status = "empty_filtered_scan";
    diagnostics.map_points = local_map_manager_.size();
    diagnostics.local_map = local_map_manager_.diagnostics();

    if (publish_diagnostics_) {
      publishDiagnostics(msg->header, diagnostics);
    }

    return;
  }

  if (!local_map_manager_.isInitialized()) {
    RCLCPP_INFO(
      get_logger(),
      "First scan callback: filtered_points=%zu",
      filtered_scan->size());

    // Bootstrap scan-to-map at identity; subsequent frames optimize against this seed map.
    current_pose_ = Eigen::Isometry3d::Identity();

    CloudTPtr first_scan_map = transformCloud(filtered_scan, current_pose_);
    RCLCPP_INFO(
      get_logger(),
      "First scan transformed: map_frame_points=%zu",
      first_scan_map->size());

    local_map_manager_.initialize(
      first_scan_map,
      current_pose_.translation());
    RCLCPP_INFO(
      get_logger(),
      "Local map initialized: points=%zu",
      local_map_manager_.size());

    diagnostics.map_initialized = true;
    diagnostics.map_points = local_map_manager_.size();
    diagnostics.map_update_time_ms =
      local_map_manager_.diagnostics().total_update_time_ms;
    diagnostics.local_map = local_map_manager_.diagnostics();
    diagnostics.optimization.success = true;
    diagnostics.optimization.status = "map_initialized";
    diagnostics.optimization.input_points = filtered_scan->size();
    consecutive_tracking_failures_ = 0;

    if (publishOdometry(msg->header, current_pose_, diagnostics.optimization)) {
      RCLCPP_INFO(get_logger(), "Initial odometry published");
    } else {
      RCLCPP_WARN(get_logger(), "Initial odometry skipped until lidar -> base transform is available");
    }

    publishLocalMap(msg->header);
    RCLCPP_INFO(get_logger(), "Initial local map published");

    if (publish_diagnostics_) {
      publishDiagnostics(msg->header, diagnostics);
    }

    RCLCPP_INFO(
      get_logger(),
      "Initialized local map with %zu points",
      local_map_manager_.size());

    updateLastAcceptedScanStamp(rclcpp::Time(msg->header.stamp));

    ++frame_count_;
    return;
  }

  // After the first frame, each scan is registered against the current local map.
  OptimizationStats stats;
  const Eigen::Isometry3d initial_guess =
    initialGuessForScan(msg->header, current_pose_, diagnostics);
  Eigen::Isometry3d optimized_pose = initial_guess;

  const auto optimization_start = std::chrono::steady_clock::now();
  const bool optimization_ok = optimizer_->optimize(
    filtered_scan,
    local_map_manager_.localMap(),
    initial_guess,
    optimized_pose,
    stats);
  const auto optimization_end = std::chrono::steady_clock::now();

  diagnostics.optimization_time_ms =
    elapsedMilliseconds(optimization_start, optimization_end);
  diagnostics.optimization = stats;

  if (optimization_ok) {
    if (constrain_to_planar_) {
      optimized_pose = makePlanarTransform(optimized_pose);
    }

    const Eigen::Isometry3d pose_delta = current_pose_.inverse() * optimized_pose;
    const double delta_translation = pose_delta.translation().norm();
    const double delta_rotation = std::abs(yawFromTransform(pose_delta));
    const double max_delta_rotation = degreesToRadians(max_pose_update_rotation_deg_);

    if (delta_translation > max_pose_update_translation_ ||
        delta_rotation > max_delta_rotation) {
      stats.success = false;
      stats.status = "accepted_pose_jump_too_large";
      diagnostics.optimization = stats;

      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Scan-to-map pose gate rejected jump: translation=%.3f m rotation=%.3f deg",
        delta_translation,
        delta_rotation * 180.0 / 3.14159265358979323846);
    } else {
      current_pose_ = optimized_pose;
      updateLastAcceptedScanStamp(rclcpp::Time(msg->header.stamp));

      const auto map_update_start = std::chrono::steady_clock::now();

      // Only accepted poses are fused into the map; rejected scans cannot corrupt it.
      CloudTPtr scan_map = transformCloud(filtered_scan, current_pose_);
      local_map_manager_.updateAfterOptimization(
        scan_map,
        current_pose_.translation());

      const auto map_update_end = std::chrono::steady_clock::now();
      diagnostics.map_update_time_ms =
        elapsedMilliseconds(map_update_start, map_update_end);
    }
  } else {
    if (stats.status == "pose_update_too_large") {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Scan-to-map optimization rejected: %s | update_translation=%.3f m "
        "update_rotation=%.3f deg | limits=%.3f m / %.3f deg",
        stats.status.c_str(),
        stats.final_update_translation_norm,
        radiansToDegrees(stats.final_update_rotation_norm),
        max_pose_update_translation_,
        max_pose_update_rotation_deg_);
    } else {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Scan-to-map optimization rejected: %s",
        stats.status.c_str());
    }
  }

  if (stats.success) {
    consecutive_tracking_failures_ = 0;
  } else {
    ++consecutive_tracking_failures_;

    // Nav2 should see degraded covariance once failures persist across several frames.
    if (max_consecutive_tracking_failures_ > 0 &&
        consecutive_tracking_failures_ >= max_consecutive_tracking_failures_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Scan-to-map tracking degraded after %d consecutive rejected frames; last status=%s",
        consecutive_tracking_failures_,
        stats.status.c_str());

      stats.status = "tracking_degraded";
    }
  }

  diagnostics.map_initialized = local_map_manager_.isInitialized();
  diagnostics.map_points = local_map_manager_.size();
  diagnostics.optimization = stats;
  diagnostics.local_map = local_map_manager_.diagnostics();

  publishOdometry(msg->header, current_pose_, stats);

  if (shouldPublishLocalMap(msg->header)) {
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
    local_map_manager_.size(),
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

  // Voxel filtering keeps registration cost bounded and reduces duplicate surfaces.
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

  // Keep a deterministic spread through the scan instead of taking only the first points.
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
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_->lookupTransform(
        lidar_frame_,
        base_frame_,
        stamp,
        rclcpp::Duration::from_seconds(0.05));
    } catch (const tf2::TransformException&) {
      // Static transforms may not have data at the scan timestamp, so fall back to latest.
      transform = tf_buffer_->lookupTransform(
        lidar_frame_,
        base_frame_,
        rclcpp::Time(0, 0, stamp.get_clock_type()),
        rclcpp::Duration::from_seconds(0.05));
    }
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
    tf_buffer_->setUsingDedicatedThread(true);
  }

  if (!tf_listener_) {
    RCLCPP_INFO(get_logger(), "Creating TF listener");
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(
      *tf_buffer_,
      shared_from_this(),
      true);
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

void ScanToMapNode::publishLatestTransform()
{
  Eigen::Isometry3d T_odom_child = Eigen::Isometry3d::Identity();
  std::string child_frame;

  {
    std::lock_guard<std::mutex> lock(latest_tf_mutex_);

    if (!has_latest_tf_) {
      return;
    }

    T_odom_child = latest_T_odom_child_;
    child_frame = latest_tf_child_frame_;
  }

  publishTransform(get_clock()->now(), T_odom_child, child_frame);
}

bool ScanToMapNode::publishOdometry(
  const std_msgs::msg::Header& header,
  const Eigen::Isometry3d& T_odom_lidar,
  const OptimizationStats& stats)
{
  Eigen::Isometry3d T_lidar_base = Eigen::Isometry3d::Identity();

  RCLCPP_DEBUG(
    get_logger(),
    "Publishing odometry: attempting %s -> %s lookup",
    lidar_frame_.c_str(),
    base_frame_.c_str());

  if (!lookupLidarToBaseTransform(rclcpp::Time(header.stamp), T_lidar_base)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "Skipping odometry publish until %s -> %s transform is available",
      lidar_frame_.c_str(),
      base_frame_.c_str());
    return false;
  }

  const std::string child_frame = base_frame_;
  const Eigen::Isometry3d T_odom_child = T_odom_lidar * T_lidar_base;
  const Eigen::Isometry3d T_nav_odom_child = makePlanarTransform(T_odom_child);
  const bool tracking_degraded = stats.status == "tracking_degraded";

  nav_msgs::msg::Odometry odom;
  odom.header.stamp = header.stamp;
  odom.header.frame_id = odom_frame_;
  odom.child_frame_id = child_frame;
  odom.pose.pose = toRosPose(T_nav_odom_child);

  double position_covariance = stats.success ? 0.05 : 0.25;
  double orientation_covariance = stats.success ? 0.10 : 0.50;

  if (tracking_degraded) {
    position_covariance = 4.0;
    orientation_covariance = 4.0;
  }

  odom.pose.covariance[0] = position_covariance;
  odom.pose.covariance[7] = position_covariance;
  odom.pose.covariance[14] = position_covariance;
  odom.pose.covariance[21] = orientation_covariance;
  odom.pose.covariance[28] = orientation_covariance;
  odom.pose.covariance[35] = orientation_covariance;

  if (stats.success && has_previous_odom_ && previous_odom_child_frame_ == child_frame) {
    const rclcpp::Time current_stamp(header.stamp);
    const double dt = (current_stamp - previous_odom_stamp_).seconds();

    if (dt > 1.0e-6 && dt < 5.0) {
      const Eigen::Isometry3d odom_delta = previous_odom_pose_.inverse() * T_nav_odom_child;

      // Publish measured planar velocity from accepted odometry deltas for Nav2 feedback.
      odom.twist.twist.linear.x = odom_delta.translation().x() / dt;
      odom.twist.twist.linear.y = odom_delta.translation().y() / dt;
      odom.twist.twist.angular.z = yawFromTransform(odom_delta) / dt;
    }
  }

  odom.twist.covariance[0] = stats.success ? 0.05 : position_covariance;
  odom.twist.covariance[7] = stats.success ? 0.05 : position_covariance;
  odom.twist.covariance[35] = stats.success ? 0.10 : orientation_covariance;

  odom_pub_->publish(odom);

  if (publish_tf_) {
    if (tracking_degraded && stop_tf_on_tracking_degraded_) {
      // Stop refreshing TF rather than broadcasting a stale pose as if tracking were healthy.
      std::lock_guard<std::mutex> lock(latest_tf_mutex_);
      has_latest_tf_ = false;
    } else {
      std::lock_guard<std::mutex> lock(latest_tf_mutex_);
      latest_T_odom_child_ = T_nav_odom_child;
      latest_tf_child_frame_ = child_frame;
      has_latest_tf_ = true;
    }
  }

  if (stats.success) {
    previous_odom_pose_ = T_nav_odom_child;
    previous_odom_stamp_ = rclcpp::Time(header.stamp);
    previous_odom_child_frame_ = child_frame;
    has_previous_odom_ = true;
  }

  if (publish_path_ && stats.success) {
    publishPath(header, odom);
  }

  return true;
}

void ScanToMapNode::publishImuPredictedOdometry(
  const std_msgs::msg::Header& header,
  const Eigen::Isometry3d& T_odom_lidar)
{
  if (!imu_predicted_odom_pub_) {
    return;
  }

  Eigen::Isometry3d T_lidar_base = Eigen::Isometry3d::Identity();
  if (!lookupLidarToBaseTransform(rclcpp::Time(header.stamp), T_lidar_base)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "Skipping IMU predicted odometry publish until %s -> %s transform is available",
      lidar_frame_.c_str(),
      base_frame_.c_str());
    return;
  }

  const Eigen::Isometry3d T_odom_child = T_odom_lidar * T_lidar_base;
  const Eigen::Isometry3d T_nav_odom_child = makePlanarTransform(T_odom_child);

  // Debug-only prediction: publish for inspection without updating TF, path, or
  // the previous odometry state consumed by Nav2.
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = header.stamp;
  odom.header.frame_id = odom_frame_;
  odom.child_frame_id = base_frame_;
  odom.pose.pose = toRosPose(T_nav_odom_child);

  odom.pose.covariance[0] = 0.25;
  odom.pose.covariance[7] = 0.25;
  odom.pose.covariance[14] = 0.25;
  odom.pose.covariance[21] = 0.50;
  odom.pose.covariance[28] = 0.50;
  odom.pose.covariance[35] = 0.50;

  imu_predicted_odom_pub_->publish(odom);
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

  // Keep the path useful in RViz without letting it grow forever during long tests.
  if (max_path_poses_ > 0 &&
      path_msg_.poses.size() > static_cast<std::size_t>(max_path_poses_)) {
    const auto excess =
      path_msg_.poses.size() - static_cast<std::size_t>(max_path_poses_);
    path_msg_.poses.erase(
      path_msg_.poses.begin(),
      path_msg_.poses.begin() + static_cast<std::ptrdiff_t>(excess));
  }

  path_pub_->publish(path_msg_);
}

void ScanToMapNode::publishLocalMap(const std_msgs::msg::Header& header)
{
  if (!local_map_config_.publish_local_map ||
      !local_map_pub_ ||
      !local_map_manager_.isInitialized()) {
    return;
  }

  std_msgs::msg::Header map_header = header;
  map_header.frame_id = odom_frame_;

  local_map_pub_->publish(toRosCloud(*local_map_manager_.cloud(), map_header));

  // Use incoming LiDAR time for throttling so bag playback and live runs behave alike.
  last_local_map_publish_stamp_ = rclcpp::Time(header.stamp);
  has_last_local_map_publish_stamp_ = true;
}

bool ScanToMapNode::shouldPublishLocalMap(const std_msgs::msg::Header& header) const
{
  if (!local_map_config_.publish_local_map ||
      !local_map_manager_.isInitialized()) {
    return false;
  }

  if (!has_last_local_map_publish_stamp_ ||
      local_map_config_.local_map_publish_period_sec <= 0.0) {
    return true;
  }

  const rclcpp::Time current_stamp(header.stamp);
  if (current_stamp < last_local_map_publish_stamp_) {
    return true;
  }

  return (current_stamp - last_local_map_publish_stamp_).seconds() >=
    local_map_config_.local_map_publish_period_sec;
}

void ScanToMapNode::publishDiagnostics(
  const std_msgs::msg::Header& header,
  const ScanToMapDiagnostics& diagnostics)
{
  if (!diagnostics_pub_) {
    return;
  }

  RCLCPP_INFO_ONCE(
    get_logger(),
    "Building first scan-to-map diagnostics message: status=%s",
    diagnostics.optimization.status.c_str());
  const auto diagnostic_msg =
    makeDiagnosticArray(diagnostics, header.stamp, "custom_scan_to_map_odom");

  RCLCPP_INFO_ONCE(
    get_logger(),
    "Built first scan-to-map diagnostics message: statuses=%zu subscribers=%zu",
    diagnostic_msg.status.size(),
    diagnostics_pub_->get_subscription_count());

  if (diagnostics_pub_->get_subscription_count() == 0U) {
    RCLCPP_INFO_ONCE(
      get_logger(),
      "Skipping scan-to-map diagnostics publish until a subscriber is present");
    return;
  }

  RCLCPP_INFO_ONCE(get_logger(), "Publishing first scan-to-map diagnostics message");
  diagnostics_pub_->publish(diagnostic_msg);
  RCLCPP_INFO_ONCE(get_logger(), "First scan-to-map diagnostics message published");
}

void ScanToMapNode::publishTransform(
  const rclcpp::Time& stamp,
  const Eigen::Isometry3d& T_odom_child,
  const std::string& child_frame)
{
  ensureTfBroadcaster();

  const geometry_msgs::msg::Pose pose = toRosPose(T_odom_child);

  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = stamp;
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
  options.constrain_to_planar = constrain_to_planar_;
  options.plane_fitter = planeFitterOptionsFromParameters();
  return options;
}

LocalMapConfig ScanToMapNode::localMapConfigFromParameters() const
{
  LocalMapConfig config;
  config.cube_size_x = get_parameter("local_map.cube_size_x").as_double();
  config.cube_size_y = get_parameter("local_map.cube_size_y").as_double();
  config.cube_size_z = get_parameter("local_map.cube_size_z").as_double();
  config.movement_threshold_xy =
    get_parameter("local_map.movement_threshold_xy").as_double();
  config.movement_threshold_z =
    get_parameter("local_map.movement_threshold_z").as_double();
  config.voxel_leaf_size = get_parameter("local_map.voxel_leaf_size").as_double();
  config.publish_local_map = get_parameter("local_map.publish_local_map").as_bool();
  config.local_map_publish_period_sec =
    get_parameter("local_map.local_map_publish_period_sec").as_double();
  return config;
}

}  // namespace custom_scan_to_map_odom

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<custom_scan_to_map_odom::ScanToMapNode>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
