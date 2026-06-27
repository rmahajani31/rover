#include "custom_lio_ekf/lio_ekf_node.hpp"

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

namespace custom_lio_ekf
{

namespace
{

void updateCloudLayout(const custom_scan_to_map_odom::CloudTPtr& cloud)
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

double yawFromTransform(const Eigen::Isometry3d& transform)
{
  return std::atan2(transform.linear()(1, 0), transform.linear()(0, 0));
}

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

LioEkfNode::LioEkfNode(const rclcpp::NodeOptions& options)
: Node("custom_lio_ekf", options)
{
  declareParameters();
  readParameters();

  state_.P = makeInitialCovariance(ekf_parameters_.initial_covariance);
  local_map_manager_.configure(local_map_config_);

  cloud_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  imu_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  tf_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  predicted_odom_callback_group_ =
    create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  rclcpp::SubscriptionOptions cloud_subscription_options;
  cloud_subscription_options.callback_group = cloud_callback_group_;

  auto cloud_qos = rclcpp::SensorDataQoS();
  cloud_qos.keep_last(1);

  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    input_topic_,
    cloud_qos,
    std::bind(&LioEkfNode::cloudCallback, this, _1),
    cloud_subscription_options);

  rclcpp::SubscriptionOptions imu_subscription_options;
  imu_subscription_options.callback_group = imu_callback_group_;

  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    imu_topic_,
    rclcpp::SensorDataQoS(),
    std::bind(&LioEkfNode::imuCallback, this, _1),
    imu_subscription_options);

  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);

  if (publish_path_) {
    path_pub_ = create_publisher<nav_msgs::msg::Path>(path_topic_, 10);
  }

  local_map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(local_map_topic_, 10);

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

    {
      std::lock_guard<std::mutex> lock(latest_tf_mutex_);
      latest_T_odom_child_ = Eigen::Isometry3d::Identity();
      latest_tf_child_frame_ = base_frame_;
      has_latest_tf_ = true;
    }
    RCLCPP_INFO(
      get_logger(),
      "Publishing bootstrap %s -> %s TF until the first EKF odometry update is accepted",
      odom_frame_.c_str(),
      base_frame_.c_str());

    if (tf_publish_rate_hz_ > 0.0) {
      const auto tf_period =
        std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / tf_publish_rate_hz_));

      tf_timer_ = create_wall_timer(
        tf_period,
        std::bind(&LioEkfNode::publishLatestTransform, this),
        tf_callback_group_);
    }
  }

  if (publish_predicted_odom_ && predicted_odom_rate_hz_ > 0.0) {
    const auto predicted_odom_period =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / predicted_odom_rate_hz_));

    predicted_odom_timer_ = create_wall_timer(
      predicted_odom_period,
      std::bind(&LioEkfNode::publishPredictedOdometry, this),
      predicted_odom_callback_group_);
  }

  RCLCPP_INFO(
    get_logger(),
    "custom_lio_ekf initialized | accel_translation_prediction=%s | "
    "predicted_odom=%s %.2f Hz | predicted_odom_accel_translation=%s",
    ekf_parameters_.use_accel_translation_prediction ? "true" : "false",
    publish_predicted_odom_ ? "true" : "false",
    predicted_odom_rate_hz_,
    predicted_odom_use_accel_translation_ ? "true" : "false");
}

void LioEkfNode::declareParameters()
{
  declare_parameter<std::string>("input_topic", "/custom/deskewed_points");
  declare_parameter<std::string>("imu_topic", "/livox/imu");
  declare_parameter<std::string>("odom_topic", "/custom/lio_ekf_odom");
  declare_parameter<std::string>("path_topic", "/custom/lio_ekf_path");
  declare_parameter<std::string>("local_map_topic", "/custom/lio_ekf_local_map");
  declare_parameter<std::string>("diagnostics_topic", "/custom/lio_ekf_diagnostics");

  declare_parameter<std::string>("odom_frame", "odom");
  declare_parameter<std::string>("base_frame", "base_link");
  declare_parameter<std::string>("imu_frame", "livox_imu");
  declare_parameter<std::string>("lidar_frame", "livox_frame");

  declare_parameter<bool>("publish_tf", false);
  declare_parameter<bool>("publish_path", true);
  declare_parameter<bool>("publish_diagnostics", true);
  declare_parameter<bool>("stop_tf_on_tracking_degraded", true);
  declare_parameter<bool>("publish_predicted_odom", false);
  declare_parameter<bool>("predicted_odom_use_accel_translation", false);

  declare_parameter<double>("tf_publish_rate_hz", 20.0);
  declare_parameter<double>("predicted_odom_rate_hz", 10.0);
  declare_parameter<double>("max_predicted_odom_interval_sec", 2.0);
  declare_parameter<int>("max_consecutive_tracking_failures", 3);
  declare_parameter<int>("max_path_poses", 2000);

  declare_parameter<double>("scan_voxel_leaf_size", 0.20);
  declare_parameter<double>("min_range", 0.30);
  declare_parameter<double>("max_range", 30.0);
  declare_parameter<int>("max_points_per_scan", 3000);

  declare_parameter<double>("max_imu_buffer_seconds", 5.0);
  declare_parameter<double>("imu_accel_scale", 9.80665);
  declare_parameter<bool>("imu_prediction.calibrate_initial_imu", true);
  declare_parameter<int>("imu_prediction.initial_calibration_min_samples", 20);
  declare_parameter<double>("imu_prediction.initial_calibration_window_sec", 1.0);

  declare_parameter<std::vector<double>>("initial_gyro_bias", std::vector<double>{0.0, 0.0, 0.0});
  declare_parameter<std::vector<double>>("initial_accel_bias", std::vector<double>{0.0, 0.0, 0.0});
  declare_parameter<std::vector<double>>("initial_gravity", std::vector<double>{0.0, 0.0, -9.81});

  declare_parameter<std::vector<double>>("lidar_imu_translation", std::vector<double>{0.0, 0.0, 0.0});
  declare_parameter<std::vector<double>>("lidar_imu_rpy", std::vector<double>{0.0, 0.0, 0.0});

  declare_parameter<double>("initial_covariance.theta", 0.05);
  declare_parameter<double>("initial_covariance.position", 0.10);
  declare_parameter<double>("initial_covariance.velocity", 0.10);
  declare_parameter<double>("initial_covariance.gyro_bias", 0.01);
  declare_parameter<double>("initial_covariance.accel_bias", 0.10);
  declare_parameter<double>("initial_covariance.gravity", 0.10);

  declare_parameter<double>("imu_noise.gyro_noise", 0.015);
  declare_parameter<double>("imu_noise.accel_noise", 0.20);
  declare_parameter<double>("imu_noise.gyro_bias_random_walk", 0.0001);
  declare_parameter<double>("imu_noise.accel_bias_random_walk", 0.001);
  declare_parameter<bool>("imu_prediction.use_accel_translation", false);

  declare_parameter<int>("lidar_update.max_iterations", 5);
  declare_parameter<int>("lidar_update.k_neighbors", 5);
  declare_parameter<int>("lidar_update.min_valid_residuals", 100);
  declare_parameter<double>("lidar_update.lidar_residual_stddev", 0.05);
  declare_parameter<double>("lidar_update.max_neighbor_distance", 1.0);
  declare_parameter<double>("lidar_update.max_plane_error", 0.10);
  declare_parameter<double>("lidar_update.max_point_to_plane_residual", 0.50);
  declare_parameter<double>("lidar_update.min_plane_eigen_ratio", 5.0);
  declare_parameter<double>("lidar_update.convergence_theta_norm", 0.001);
  declare_parameter<double>("lidar_update.convergence_position_norm", 0.001);
  declare_parameter<double>("lidar_update.max_correction_theta_norm", 0.10);
  declare_parameter<double>("lidar_update.max_correction_position_norm", 0.30);
  declare_parameter<bool>("lidar_update.use_huber_weight", true);
  declare_parameter<double>("lidar_update.huber_delta", 0.10);

  declare_parameter<double>("local_map.cube_size_x", 30.0);
  declare_parameter<double>("local_map.cube_size_y", 30.0);
  declare_parameter<double>("local_map.cube_size_z", 6.0);
  declare_parameter<double>("local_map.movement_threshold_xy", 5.0);
  declare_parameter<double>("local_map.movement_threshold_z", 2.0);
  declare_parameter<double>("local_map.voxel_leaf_size", 0.15);
  declare_parameter<bool>("local_map.publish_local_map", true);
  declare_parameter<double>("local_map.local_map_publish_period_sec", 1.0);
}

void LioEkfNode::readParameters()
{
  input_topic_ = get_parameter("input_topic").as_string();
  imu_topic_ = get_parameter("imu_topic").as_string();
  odom_topic_ = get_parameter("odom_topic").as_string();
  path_topic_ = get_parameter("path_topic").as_string();
  local_map_topic_ = get_parameter("local_map_topic").as_string();
  diagnostics_topic_ = get_parameter("diagnostics_topic").as_string();

  odom_frame_ = get_parameter("odom_frame").as_string();
  base_frame_ = get_parameter("base_frame").as_string();
  imu_frame_ = get_parameter("imu_frame").as_string();
  lidar_frame_ = get_parameter("lidar_frame").as_string();

  publish_tf_ = get_parameter("publish_tf").as_bool();
  publish_path_ = get_parameter("publish_path").as_bool();
  publish_diagnostics_ = get_parameter("publish_diagnostics").as_bool();
  stop_tf_on_tracking_degraded_ = get_parameter("stop_tf_on_tracking_degraded").as_bool();
  publish_predicted_odom_ = get_parameter("publish_predicted_odom").as_bool();
  predicted_odom_use_accel_translation_ =
    get_parameter("predicted_odom_use_accel_translation").as_bool();

  tf_publish_rate_hz_ = get_parameter("tf_publish_rate_hz").as_double();
  predicted_odom_rate_hz_ = get_parameter("predicted_odom_rate_hz").as_double();
  max_predicted_odom_interval_sec_ =
    get_parameter("max_predicted_odom_interval_sec").as_double();
  max_consecutive_tracking_failures_ = get_parameter("max_consecutive_tracking_failures").as_int();
  max_path_poses_ = get_parameter("max_path_poses").as_int();

  scan_voxel_leaf_size_ = get_parameter("scan_voxel_leaf_size").as_double();
  min_range_ = get_parameter("min_range").as_double();
  max_range_ = get_parameter("max_range").as_double();
  max_points_per_scan_ = get_parameter("max_points_per_scan").as_int();

  max_imu_buffer_seconds_ = get_parameter("max_imu_buffer_seconds").as_double();
  imu_accel_scale_ = get_parameter("imu_accel_scale").as_double();
  calibrate_initial_imu_ = get_parameter("imu_prediction.calibrate_initial_imu").as_bool();
  const int64_t initial_imu_calibration_min_samples =
    get_parameter("imu_prediction.initial_calibration_min_samples").as_int();
  initial_imu_calibration_min_samples_ =
    static_cast<int>(std::max<int64_t>(1, initial_imu_calibration_min_samples));
  initial_imu_calibration_window_sec_ =
    get_parameter("imu_prediction.initial_calibration_window_sec").as_double();

  state_.b_g = readVector3Parameter("initial_gyro_bias", Eigen::Vector3d::Zero());
  state_.b_a = readVector3Parameter("initial_accel_bias", Eigen::Vector3d::Zero());
  state_.g_W = readVector3Parameter("initial_gravity", Eigen::Vector3d(0.0, 0.0, -9.81));

  lidar_imu_extrinsics_.p_L_in_I =
    readVector3Parameter("lidar_imu_translation", Eigen::Vector3d::Zero());

  const Eigen::Vector3d rpy =
    readVector3Parameter("lidar_imu_rpy", Eigen::Vector3d::Zero());
  lidar_imu_extrinsics_.q_IL =
    Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX());

  ekf_parameters_ = parametersFromRosParameters();
  local_map_config_ = localMapConfigFromParameters();
}

void LioEkfNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
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

  std::lock_guard<std::mutex> lock(imu_mutex_);
  imu_buffer_.push_back(sample);
  ++imu_samples_received_;
  trimImuBuffer(sample.stamp);
}

void LioEkfNode::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  const auto callback_start = std::chrono::steady_clock::now();

  LioEkfDiagnostics diagnostics;
  diagnostics.map_initialized = local_map_manager_.isInitialized();

  auto raw_cloud = custom_scan_to_map_odom::fromRosCloud(*msg);
  auto filtered_scan = filterScan(raw_cloud);

  diagnostics.input_points = filtered_scan->size();
  diagnostics.map_points = local_map_manager_.size();

  if (filtered_scan->empty()) {
    diagnostics.lidar_update.status = "empty_filtered_scan";
    publishDiagnostics(msg->header, diagnostics);
    return;
  }

  if (!local_map_manager_.isInitialized()) {
    initializeMap(msg->header, filtered_scan, diagnostics);
    publishDiagnostics(msg->header, diagnostics);
    ++frame_count_;
    return;
  }

  const rclcpp::Time scan_stamp(msg->header.stamp);

  const auto prediction_start = std::chrono::steady_clock::now();
  EkfPredictionStats prediction_stats;
  bool prediction_ok = false;
  LidarUpdateStats update_stats;
  bool update_ok = false;

  prediction_ok = runPrediction(scan_stamp, prediction_stats);
  const auto prediction_end = std::chrono::steady_clock::now();

  diagnostics.prediction = prediction_stats;
  diagnostics.prediction_time_ms =
    elapsedMilliseconds(prediction_start, prediction_end);

  if (prediction_ok) {
    const auto update_start = std::chrono::steady_clock::now();
    update_ok = runLidarUpdate(filtered_scan, update_stats);
    const auto update_end = std::chrono::steady_clock::now();

    diagnostics.lidar_update = update_stats;
    diagnostics.lidar_update_time_ms =
      elapsedMilliseconds(update_start, update_end);
  } else {
    update_stats.status = "prediction_failed";
    diagnostics.lidar_update = update_stats;
  }

  if (update_ok) {
    consecutive_tracking_failures_ = 0;
    updateMap(filtered_scan, diagnostics);
    last_scan_stamp_ = scan_stamp;
    has_last_scan_stamp_ = true;
  } else {
    if (prediction_ok) {
      // The state has already been predicted to this scan time. Advance the IMU
      // interval even when LiDAR correction is rejected, otherwise the next scan
      // re-integrates the same IMU samples and quickly pushes the state away.
      last_scan_stamp_ = scan_stamp;
      has_last_scan_stamp_ = true;
    }

    ++consecutive_tracking_failures_;

    if (max_consecutive_tracking_failures_ > 0 &&
        consecutive_tracking_failures_ >= max_consecutive_tracking_failures_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "LIO EKF tracking degraded after %d consecutive failures; last status=%s | "
        "residuals=%zu | rms=%.4f | max=%.4f | dtheta=%.4f | dpos=%.4f",
        consecutive_tracking_failures_,
        diagnostics.lidar_update.status.c_str(),
        diagnostics.lidar_update.valid_residuals,
        diagnostics.lidar_update.rms_residual,
        diagnostics.lidar_update.max_abs_residual,
        diagnostics.lidar_update.final_delta_theta_norm,
        diagnostics.lidar_update.final_delta_position_norm);
      diagnostics.lidar_update.status = "tracking_degraded";
    }
  }

  if (update_ok || prediction_ok) {
    updatePredictionBaseState(scan_stamp);
  }

  diagnostics.map_initialized = local_map_manager_.isInitialized();
  diagnostics.map_points = local_map_manager_.size();

  publishOdometry(msg->header, diagnostics.lidar_update);
  publishPredictedOdometry();

  if (shouldPublishLocalMap(msg->header)) {
    publishLocalMap(msg->header);
  }

  publishDiagnostics(msg->header, diagnostics);

  const auto callback_end = std::chrono::steady_clock::now();

  RCLCPP_INFO_THROTTLE(
    get_logger(),
    *get_clock(),
    1000,
    "lio_ekf %s | points=%zu | map=%zu | residuals=%zu | rms=%.4f | "
    "dtheta=%.4f | dpos=%.4f | time=%.2f ms",
    diagnostics.lidar_update.status.c_str(),
    filtered_scan->size(),
    local_map_manager_.size(),
    diagnostics.lidar_update.valid_residuals,
    diagnostics.lidar_update.rms_residual,
    diagnostics.lidar_update.final_delta_theta_norm,
    diagnostics.lidar_update.final_delta_position_norm,
    elapsedMilliseconds(callback_start, callback_end));

  ++frame_count_;
}

custom_scan_to_map_odom::CloudTPtr LioEkfNode::filterScan(
  const custom_scan_to_map_odom::CloudTConstPtr& cloud) const
{
  custom_scan_to_map_odom::CloudTPtr range_filtered(
    new custom_scan_to_map_odom::CloudT());

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

  custom_scan_to_map_odom::CloudTPtr voxel_filtered(
    new custom_scan_to_map_odom::CloudT());

  if (scan_voxel_leaf_size_ > 0.0 && !range_filtered->empty()) {
    pcl::VoxelGrid<custom_scan_to_map_odom::PointT> voxel;
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

  custom_scan_to_map_odom::CloudTPtr limited(
    new custom_scan_to_map_odom::CloudT());
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

custom_scan_to_map_odom::CloudTPtr LioEkfNode::transformCloudToWorld(
  const custom_scan_to_map_odom::CloudTConstPtr& cloud,
  const EkfState& state) const
{
  custom_scan_to_map_odom::CloudTPtr transformed(
    new custom_scan_to_map_odom::CloudT());

  if (!cloud || cloud->empty()) {
    return transformed;
  }

  transformed->points.reserve(cloud->points.size());

  for (const auto& point : cloud->points) {
    const Eigen::Vector3d point_L(
      static_cast<double>(point.x),
      static_cast<double>(point.y),
      static_cast<double>(point.z));

    const Eigen::Vector3d point_I =
      transformLidarPointToImu(point_L, lidar_imu_extrinsics_);
    const Eigen::Vector3d point_W =
      transformImuPointToWorld(state, point_I);

    custom_scan_to_map_odom::PointT output = point;
    output.x = static_cast<float>(point_W.x());
    output.y = static_cast<float>(point_W.y());
    output.z = static_cast<float>(point_W.z());

    transformed->points.push_back(output);
  }

  updateCloudLayout(transformed);
  return transformed;
}

bool LioEkfNode::getImuSamplesForScan(
  const rclcpp::Time& scan_stamp,
  std::vector<custom_imu_propagator::ImuSample>& samples)
{
  samples.clear();

  if (!has_last_scan_stamp_) {
    return false;
  }

  std::lock_guard<std::mutex> lock(imu_mutex_);

  for (const auto& sample : imu_buffer_) {
    if (sample.stamp >= last_scan_stamp_ && sample.stamp <= scan_stamp) {
      samples.push_back(sample);
    }
  }

  return samples.size() >= 2;
}

void LioEkfNode::trimImuBuffer(const rclcpp::Time& newest_stamp)
{
  while (!imu_buffer_.empty() &&
         (newest_stamp - imu_buffer_.front().stamp).seconds() > max_imu_buffer_seconds_) {
    imu_buffer_.pop_front();
  }
}

bool LioEkfNode::calibrateInitialImuState(const rclcpp::Time& scan_stamp)
{
  if (!calibrate_initial_imu_ || initial_imu_calibrated_) {
    return true;
  }

  Eigen::Vector3d accel_sum = Eigen::Vector3d::Zero();
  Eigen::Vector3d gyro_sum = Eigen::Vector3d::Zero();
  int sample_count = 0;

  {
    std::lock_guard<std::mutex> lock(imu_mutex_);

    for (const auto& sample : imu_buffer_) {
      if (sample.stamp > scan_stamp) {
        continue;
      }

      if (initial_imu_calibration_window_sec_ > 0.0 &&
          (scan_stamp - sample.stamp).seconds() > initial_imu_calibration_window_sec_) {
        continue;
      }

      if (!sample.accel.allFinite() || !sample.gyro.allFinite()) {
        continue;
      }

      accel_sum += sample.accel;
      gyro_sum += sample.gyro;
      ++sample_count;
    }
  }

  if (sample_count < initial_imu_calibration_min_samples_) {
    RCLCPP_WARN(
      get_logger(),
      "Skipping initial IMU calibration: only %d samples available, need %d",
      sample_count,
      initial_imu_calibration_min_samples_);
    return false;
  }

  const Eigen::Vector3d accel_mean = accel_sum / static_cast<double>(sample_count);
  const Eigen::Vector3d gyro_mean = gyro_sum / static_cast<double>(sample_count);
  const double accel_norm = accel_mean.norm();
  const double gravity_norm = state_.g_W.norm();

  if (!std::isfinite(accel_norm) || accel_norm < 1.0 ||
      !std::isfinite(gravity_norm) || gravity_norm < 1.0) {
    RCLCPP_WARN(
      get_logger(),
      "Skipping initial IMU calibration: accel_norm=%.4f gravity_norm=%.4f",
      accel_norm,
      gravity_norm);
    return false;
  }

  const Eigen::Vector3d measured_specific_force = accel_mean.normalized();
  const Eigen::Vector3d expected_specific_force_world = (-state_.g_W).normalized();

  state_.q_WI =
    Eigen::Quaterniond::FromTwoVectors(
      measured_specific_force,
      expected_specific_force_world);
  state_.q_WI.normalize();

  const Eigen::Vector3d expected_specific_force_imu =
    state_.q_WI.inverse() * (-state_.g_W);

  state_.b_g = gyro_mean;
  state_.b_a = accel_mean - expected_specific_force_imu;
  state_.v_I_W = Eigen::Vector3d::Zero();
  initial_imu_calibrated_ = true;

  RCLCPP_INFO(
    get_logger(),
    "Initial IMU calibration: samples=%d accel_norm=%.4f gyro_bias=[%.5f %.5f %.5f] "
    "accel_bias=[%.5f %.5f %.5f]",
    sample_count,
    accel_norm,
    state_.b_g.x(),
    state_.b_g.y(),
    state_.b_g.z(),
    state_.b_a.x(),
    state_.b_a.y(),
    state_.b_a.z());

  return true;
}

bool LioEkfNode::initializeMap(
  const std_msgs::msg::Header& header,
  const custom_scan_to_map_odom::CloudTConstPtr& filtered_scan,
  LioEkfDiagnostics& diagnostics)
{
  const rclcpp::Time scan_stamp(header.stamp);

  if (!calibrateInitialImuState(scan_stamp)) {
    diagnostics.lidar_update.success = false;
    diagnostics.lidar_update.status = "waiting_for_initial_imu_calibration";
    diagnostics.lidar_update.input_points = filtered_scan->size();
    return false;
  }

  state_.P = makeInitialCovariance(ekf_parameters_.initial_covariance);

  auto scan_world = transformCloudToWorld(filtered_scan, state_);
  local_map_manager_.initialize(scan_world, state_.p_I_W);

  last_scan_stamp_ = scan_stamp;
  has_last_scan_stamp_ = true;
  updatePredictionBaseState(scan_stamp);

  diagnostics.map_initialized = true;
  diagnostics.map_points = local_map_manager_.size();
  diagnostics.lidar_update.success = true;
  diagnostics.lidar_update.status = "map_initialized";
  diagnostics.lidar_update.input_points = filtered_scan->size();

  publishOdometry(header, diagnostics.lidar_update);
  publishLocalMap(header);

  RCLCPP_INFO(
    get_logger(),
    "Initialized LIO EKF local map with %zu points",
    local_map_manager_.size());

  return true;
}

bool LioEkfNode::runPrediction(
  const rclcpp::Time& scan_stamp,
  EkfPredictionStats& prediction_stats)
{
  std::vector<custom_imu_propagator::ImuSample> samples;

  if (!getImuSamplesForScan(scan_stamp, samples)) {
    prediction_stats = EkfPredictionStats{};
    prediction_stats.status = "missing_imu_interval";
    return false;
  }

  return predictStateAndCovariance(
    state_,
    ekf_parameters_,
    samples,
    prediction_stats);
}

bool LioEkfNode::runLidarUpdate(
  const custom_scan_to_map_odom::CloudTConstPtr& filtered_scan,
  LidarUpdateStats& update_stats)
{
  return applyIteratedLidarUpdate(
    state_,
    filtered_scan,
    lidar_imu_extrinsics_,
    local_map_manager_.localMap(),
    ekf_parameters_.lidar_update,
    update_stats);
}

void LioEkfNode::updateMap(
  const custom_scan_to_map_odom::CloudTConstPtr& filtered_scan,
  LioEkfDiagnostics& diagnostics)
{
  const auto map_update_start = std::chrono::steady_clock::now();

  auto scan_world = transformCloudToWorld(filtered_scan, state_);
  local_map_manager_.updateAfterOptimization(scan_world, state_.p_I_W);

  const auto map_update_end = std::chrono::steady_clock::now();
  diagnostics.map_update_time_ms =
    elapsedMilliseconds(map_update_start, map_update_end);
}

bool LioEkfNode::lookupImuToBaseTransform(
  const rclcpp::Time& stamp,
  Eigen::Isometry3d& T_imu_base)
{
  ensureTfListener();

  try {
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_->lookupTransform(
        imu_frame_,
        base_frame_,
        stamp,
        rclcpp::Duration::from_seconds(0.05));
    } catch (const tf2::TransformException&) {
      transform = tf_buffer_->lookupTransform(
        imu_frame_,
        base_frame_,
        rclcpp::Time(0, 0, stamp.get_clock_type()),
        rclcpp::Duration::from_seconds(0.05));
    }

    T_imu_base = tf2::transformToEigen(transform);
    return true;
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "Could not look up %s -> %s transform: %s",
      imu_frame_.c_str(),
      base_frame_.c_str(),
      ex.what());
    return false;
  }
}

void LioEkfNode::ensureTfListener()
{
  if (!tf_buffer_) {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_buffer_->setUsingDedicatedThread(true);
  }

  if (!tf_listener_) {
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(
      *tf_buffer_,
      shared_from_this(),
      true);
  }
}

void LioEkfNode::ensureTfBroadcaster()
{
  if (!tf_broadcaster_) {
    tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(shared_from_this());
  }
}

void LioEkfNode::publishLatestTransform()
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

void LioEkfNode::publishPredictedOdometry()
{
  if (!publish_predicted_odom_) {
    return;
  }

  EkfState predicted_state;
  rclcpp::Time base_stamp(0, 0, RCL_ROS_TIME);

  if (!getPredictionBaseState(predicted_state, base_stamp)) {
    ++predicted_odom_no_base_count_;
    logPredictedOdometryStats();
    return;
  }

  std::vector<custom_imu_propagator::ImuSample> samples;

  {
    std::lock_guard<std::mutex> imu_lock(imu_mutex_);

    if (imu_buffer_.size() < 2) {
      ++predicted_odom_not_enough_imu_count_;
      logPredictedOdometryStats();
      return;
    }

    for (const auto& sample : imu_buffer_) {
      if (sample.stamp >= base_stamp) {
        samples.push_back(sample);
      }
    }
  }

  if (samples.size() < 2) {
    ++predicted_odom_not_enough_imu_count_;
    logPredictedOdometryStats();
    return;
  }

  const rclcpp::Time predicted_stamp = samples.back().stamp;
  const double prediction_interval = (predicted_stamp - base_stamp).seconds();
  const rclcpp::Time now = get_clock()->now();

  last_predicted_interval_sec_ = prediction_interval;
  last_predicted_base_stamp_sec_ = base_stamp.seconds();
  last_predicted_imu_stamp_sec_ = predicted_stamp.seconds();
  last_predicted_now_sec_ = now.seconds();
  last_predicted_now_minus_base_sec_ = (now - base_stamp).seconds();
  last_predicted_now_minus_imu_sec_ = (now - predicted_stamp).seconds();

  if (prediction_interval <= 0.0 ||
      (max_predicted_odom_interval_sec_ > 0.0 &&
       prediction_interval > max_predicted_odom_interval_sec_)) {
    ++predicted_odom_stale_interval_count_;
    logPredictedOdometryStats();
    return;
  }

  ImuPredictionStats prediction_stats;
  if (!predictNominalState(
      predicted_state,
      samples,
      prediction_stats,
      predicted_odom_use_accel_translation_)) {
    ++predicted_odom_prediction_failed_count_;
    logPredictedOdometryStats();
    return;
  }

  std_msgs::msg::Header header;
  header.stamp = predicted_stamp;
  header.frame_id = odom_frame_;

  LidarUpdateStats update_stats;
  update_stats.success = true;
  update_stats.status = "imu_predicted";

  if (publishOdometryForState(header, update_stats, predicted_state, false)) {
    ++predicted_odom_success_count_;
  } else {
    ++predicted_odom_publish_rejected_count_;
  }

  logPredictedOdometryStats();
}

void LioEkfNode::logPredictedOdometryStats()
{
  RCLCPP_INFO_THROTTLE(
    get_logger(),
    *get_clock(),
    2000,
    "predicted_odom stats | success=%zu | no_base=%zu | not_enough_imu=%zu | "
    "stale_interval=%zu | prediction_failed=%zu | publish_rejected=%zu | "
    "base_updates=%zu | interval=%.3f | base_stamp=%.3f | imu_stamp=%.3f | "
    "now=%.3f | now-base=%.3f | now-imu=%.3f | "
    "last_base_update_stamp=%.3f | last_base_update_now=%.3f | "
    "last_base_update_age=%.3f",
    predicted_odom_success_count_,
    predicted_odom_no_base_count_,
    predicted_odom_not_enough_imu_count_,
    predicted_odom_stale_interval_count_,
    predicted_odom_prediction_failed_count_,
    predicted_odom_publish_rejected_count_,
    prediction_base_update_count_,
    last_predicted_interval_sec_,
    last_predicted_base_stamp_sec_,
    last_predicted_imu_stamp_sec_,
    last_predicted_now_sec_,
    last_predicted_now_minus_base_sec_,
    last_predicted_now_minus_imu_sec_,
    last_base_update_stamp_sec_,
    last_base_update_now_sec_,
    last_base_update_now_minus_stamp_sec_);
}

void LioEkfNode::updatePredictionBaseState(const rclcpp::Time& stamp)
{
  std::lock_guard<std::mutex> lock(prediction_base_mutex_);
  prediction_base_state_ = state_;
  prediction_base_stamp_ = stamp;
  has_prediction_base_state_ = true;
  ++prediction_base_update_count_;

  const rclcpp::Time now = get_clock()->now();
  last_base_update_stamp_sec_ = stamp.seconds();
  last_base_update_now_sec_ = now.seconds();
  last_base_update_now_minus_stamp_sec_ = (now - stamp).seconds();
}

bool LioEkfNode::getPredictionBaseState(EkfState& state, rclcpp::Time& stamp)
{
  std::lock_guard<std::mutex> lock(prediction_base_mutex_);

  if (!has_prediction_base_state_) {
    return false;
  }

  state = prediction_base_state_;
  stamp = prediction_base_stamp_;
  return true;
}

bool LioEkfNode::publishOdometry(
  const std_msgs::msg::Header& header,
  const LidarUpdateStats& update_stats)
{
  return publishOdometryForState(header, update_stats, state_, publish_path_);
}

bool LioEkfNode::publishOdometryForState(
  const std_msgs::msg::Header& header,
  const LidarUpdateStats& update_stats,
  const EkfState& state,
  bool publish_path)
{
  Eigen::Isometry3d T_imu_base = Eigen::Isometry3d::Identity();

  if (!lookupImuToBaseTransform(rclcpp::Time(header.stamp), T_imu_base)) {
    return false;
  }

  const Eigen::Isometry3d T_odom_imu = imuPoseInWorld(state);
  const Eigen::Isometry3d T_odom_base = makePlanarTransform(T_odom_imu * T_imu_base);
  const bool tracking_degraded = update_stats.status == "tracking_degraded";
  const bool imu_predicted = update_stats.status == "imu_predicted";

  nav_msgs::msg::Odometry odom;
  odom.header.stamp = header.stamp;
  odom.header.frame_id = odom_frame_;
  odom.child_frame_id = base_frame_;
  odom.pose.pose = custom_scan_to_map_odom::toRosPose(T_odom_base);

  double position_covariance = update_stats.success ? 0.05 : 0.25;
  double orientation_covariance = update_stats.success ? 0.10 : 0.50;

  if (imu_predicted) {
    position_covariance = 0.15;
    orientation_covariance = 0.20;
  }

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

  const rclcpp::Time current_stamp(header.stamp);

  {
    std::lock_guard<std::mutex> odom_lock(odom_publish_mutex_);

    if (has_previous_odom_ && current_stamp <= previous_odom_stamp_) {
      return false;
    }

    if (update_stats.success && has_previous_odom_ && previous_odom_child_frame_ == base_frame_) {
      const double dt = (current_stamp - previous_odom_stamp_).seconds();

      if (dt > 1.0e-6 && dt < 5.0) {
        const Eigen::Isometry3d odom_delta =
          previous_odom_pose_.inverse() * T_odom_base;

        odom.twist.twist.linear.x = odom_delta.translation().x() / dt;
        odom.twist.twist.linear.y = odom_delta.translation().y() / dt;
        odom.twist.twist.angular.z = yawFromTransform(odom_delta) / dt;
      }
    }

    odom.twist.covariance[0] = update_stats.success ? 0.05 : position_covariance;
    odom.twist.covariance[7] = update_stats.success ? 0.05 : position_covariance;
    odom.twist.covariance[35] = update_stats.success ? 0.10 : orientation_covariance;

    odom_pub_->publish(odom);

    if (publish_tf_) {
      if (tracking_degraded && stop_tf_on_tracking_degraded_) {
        std::lock_guard<std::mutex> lock(latest_tf_mutex_);
        has_latest_tf_ = false;
      } else {
        std::lock_guard<std::mutex> lock(latest_tf_mutex_);
        latest_T_odom_child_ = T_odom_base;
        latest_tf_child_frame_ = base_frame_;
        has_latest_tf_ = true;
      }
    }

    if (!tracking_degraded) {
      previous_odom_pose_ = T_odom_base;
      previous_odom_stamp_ = current_stamp;
      previous_odom_child_frame_ = base_frame_;
      has_previous_odom_ = true;
    }
  }

  if (publish_path && update_stats.success && !imu_predicted) {
    publishPath(header, odom);
  }

  return true;
}

void LioEkfNode::publishPath(
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

void LioEkfNode::publishLocalMap(const std_msgs::msg::Header& header)
{
  if (!local_map_config_.publish_local_map ||
      !local_map_pub_ ||
      !local_map_manager_.isInitialized()) {
    return;
  }

  std_msgs::msg::Header map_header = header;
  map_header.frame_id = odom_frame_;

  local_map_pub_->publish(
    custom_scan_to_map_odom::toRosCloud(*local_map_manager_.cloud(), map_header));

  last_local_map_publish_stamp_ = rclcpp::Time(header.stamp);
  has_last_local_map_publish_stamp_ = true;
}

bool LioEkfNode::shouldPublishLocalMap(const std_msgs::msg::Header& header) const
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

void LioEkfNode::publishDiagnostics(
  const std_msgs::msg::Header& header,
  const LioEkfDiagnostics& diagnostics)
{
  if (!publish_diagnostics_ || !diagnostics_pub_) {
    return;
  }

  auto diagnostic_msg =
    makeDiagnosticArray(diagnostics, header.stamp, "custom_lio_ekf");
  diagnostics_pub_->publish(diagnostic_msg);
}

void LioEkfNode::publishTransform(
  const rclcpp::Time& stamp,
  const Eigen::Isometry3d& T_odom_child,
  const std::string& child_frame)
{
  ensureTfBroadcaster();

  const geometry_msgs::msg::Pose pose =
    custom_scan_to_map_odom::toRosPose(T_odom_child);

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

Eigen::Vector3d LioEkfNode::readVector3Parameter(
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

EkfParameters LioEkfNode::parametersFromRosParameters() const
{
  EkfParameters parameters;

  parameters.initial_covariance.theta =
    get_parameter("initial_covariance.theta").as_double();
  parameters.initial_covariance.position =
    get_parameter("initial_covariance.position").as_double();
  parameters.initial_covariance.velocity =
    get_parameter("initial_covariance.velocity").as_double();
  parameters.initial_covariance.gyro_bias =
    get_parameter("initial_covariance.gyro_bias").as_double();
  parameters.initial_covariance.accel_bias =
    get_parameter("initial_covariance.accel_bias").as_double();
  parameters.initial_covariance.gravity =
    get_parameter("initial_covariance.gravity").as_double();

  parameters.imu_noise.gyro_noise =
    get_parameter("imu_noise.gyro_noise").as_double();
  parameters.imu_noise.accel_noise =
    get_parameter("imu_noise.accel_noise").as_double();
  parameters.imu_noise.gyro_bias_random_walk =
    get_parameter("imu_noise.gyro_bias_random_walk").as_double();
  parameters.imu_noise.accel_bias_random_walk =
    get_parameter("imu_noise.accel_bias_random_walk").as_double();
  parameters.use_accel_translation_prediction =
    get_parameter("imu_prediction.use_accel_translation").as_bool();

  parameters.lidar_update.max_iterations =
    get_parameter("lidar_update.max_iterations").as_int();
  parameters.lidar_update.k_neighbors =
    get_parameter("lidar_update.k_neighbors").as_int();
  const int64_t min_valid_residuals =
    get_parameter("lidar_update.min_valid_residuals").as_int();
  parameters.lidar_update.min_valid_residuals =
    static_cast<std::size_t>(std::max<int64_t>(0, min_valid_residuals));
  parameters.lidar_update.lidar_residual_stddev =
    get_parameter("lidar_update.lidar_residual_stddev").as_double();
  parameters.lidar_update.max_neighbor_distance =
    get_parameter("lidar_update.max_neighbor_distance").as_double();
  parameters.lidar_update.max_plane_error =
    get_parameter("lidar_update.max_plane_error").as_double();
  parameters.lidar_update.max_point_to_plane_residual =
    get_parameter("lidar_update.max_point_to_plane_residual").as_double();
  parameters.lidar_update.min_plane_eigen_ratio =
    get_parameter("lidar_update.min_plane_eigen_ratio").as_double();
  parameters.lidar_update.convergence_theta_norm =
    get_parameter("lidar_update.convergence_theta_norm").as_double();
  parameters.lidar_update.convergence_position_norm =
    get_parameter("lidar_update.convergence_position_norm").as_double();
  parameters.lidar_update.max_correction_theta_norm =
    get_parameter("lidar_update.max_correction_theta_norm").as_double();
  parameters.lidar_update.max_correction_position_norm =
    get_parameter("lidar_update.max_correction_position_norm").as_double();
  parameters.lidar_update.use_huber_weight =
    get_parameter("lidar_update.use_huber_weight").as_bool();
  parameters.lidar_update.huber_delta =
    get_parameter("lidar_update.huber_delta").as_double();

  return parameters;
}

custom_scan_to_map_odom::LocalMapConfig LioEkfNode::localMapConfigFromParameters() const
{
  custom_scan_to_map_odom::LocalMapConfig config;
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

}  // namespace custom_lio_ekf

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<custom_lio_ekf::LioEkfNode>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
