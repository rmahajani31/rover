#include "custom_lidar_deskew/lidar_deskew_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <utility>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "custom_lidar_deskew/imu_integrator.hpp"

namespace custom_lidar_deskew
{
namespace
{

std::size_t pointCount(const sensor_msgs::msg::PointCloud2& cloud)
{
  return static_cast<std::size_t>(cloud.width) * static_cast<std::size_t>(cloud.height);
}

diagnostic_msgs::msg::KeyValue makeStatusValue(
  const std::string& key,
  const std::string& value)
{
  diagnostic_msgs::msg::KeyValue key_value;
  key_value.key = key;
  key_value.value = value;
  return key_value;
}

}  // namespace

LidarDeskewNode::LidarDeskewNode()
: Node("custom_lidar_deskew")
{
  declareParameters();
  loadParameters();

  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    imu_topic_,
    rclcpp::SensorDataQoS(),
    std::bind(&LidarDeskewNode::imuCallback, this, std::placeholders::_1));

  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    lidar_topic_,
    rclcpp::SensorDataQoS(),
    std::bind(&LidarDeskewNode::cloudCallback, this, std::placeholders::_1));

  cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    output_topic_,
    rclcpp::SensorDataQoS());

  diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    diagnostics_topic_,
    10);

  RCLCPP_INFO(
    get_logger(),
    "custom_lidar_deskew started: lidar=%s imu=%s output=%s",
    lidar_topic_.c_str(),
    imu_topic_.c_str(),
    output_topic_.c_str());
}

void LidarDeskewNode::declareParameters()
{
  declare_parameter<std::string>("lidar_topic", "/livox/lidar");
  declare_parameter<std::string>("imu_topic", "/livox/imu");
  declare_parameter<std::string>("output_topic", "/custom/deskewed_points");
  declare_parameter<std::string>("diagnostics_topic", "/custom/deskew/diagnostics");

  declare_parameter<bool>("deskew_to_scan_end", true);
  declare_parameter<bool>("use_rotation_deskew", true);
  declare_parameter<bool>("use_translation_deskew", false);

  declare_parameter<double>("gyro_bias_x", 0.0);
  declare_parameter<double>("gyro_bias_y", 0.0);
  declare_parameter<double>("gyro_bias_z", 0.0);

  declare_parameter<double>("accel_bias_x", 0.0);
  declare_parameter<double>("accel_bias_y", 0.0);
  declare_parameter<double>("accel_bias_z", 0.0);

  declare_parameter<double>("gravity_x", 0.0);
  declare_parameter<double>("gravity_y", 0.0);
  declare_parameter<double>("gravity_z", -9.81);

  declare_parameter<int>("max_imu_buffer_size", 5000);
  declare_parameter<double>("imu_time_margin_sec", 0.02);
  declare_parameter<double>("max_allowed_scan_duration_sec", 0.2);
  declare_parameter<int>("min_required_imu_samples", 2);

  declare_parameter<std::string>("point_time_field", "offset_time");
  declare_parameter<std::string>("point_time_unit", "nanoseconds");
  declare_parameter<double>("fallback_scan_duration_sec", 0.1);
  declare_parameter<bool>("allow_fallback_without_point_time", false);

  declare_parameter<bool>("publish_raw_if_deskew_fails", true);
  declare_parameter<bool>("preserve_input_header_stamp", true);
  declare_parameter<std::string>("output_frame_id", "");

  declare_parameter<bool>("publish_diagnostics", true);
  declare_parameter<bool>("print_timing_debug", false);
}

void LidarDeskewNode::loadParameters()
{
  lidar_topic_ = get_parameter("lidar_topic").as_string();
  imu_topic_ = get_parameter("imu_topic").as_string();
  output_topic_ = get_parameter("output_topic").as_string();
  diagnostics_topic_ = get_parameter("diagnostics_topic").as_string();

  deskew_to_scan_end_ = get_parameter("deskew_to_scan_end").as_bool();
  use_rotation_deskew_ = get_parameter("use_rotation_deskew").as_bool();
  use_translation_deskew_ = get_parameter("use_translation_deskew").as_bool();

  gyro_bias_ = Eigen::Vector3d(
    get_parameter("gyro_bias_x").as_double(),
    get_parameter("gyro_bias_y").as_double(),
    get_parameter("gyro_bias_z").as_double());

  accel_bias_ = Eigen::Vector3d(
    get_parameter("accel_bias_x").as_double(),
    get_parameter("accel_bias_y").as_double(),
    get_parameter("accel_bias_z").as_double());

  gravity_ = Eigen::Vector3d(
    get_parameter("gravity_x").as_double(),
    get_parameter("gravity_y").as_double(),
    get_parameter("gravity_z").as_double());

  max_imu_buffer_size_ =
    static_cast<std::size_t>(
    std::max<std::int64_t>(1, get_parameter("max_imu_buffer_size").as_int()));
  imu_time_margin_sec_ = get_parameter("imu_time_margin_sec").as_double();
  max_allowed_scan_duration_sec_ = get_parameter("max_allowed_scan_duration_sec").as_double();
  min_required_imu_samples_ = get_parameter("min_required_imu_samples").as_int();

  point_time_field_name_ = get_parameter("point_time_field").as_string();
  point_time_unit_name_ = get_parameter("point_time_unit").as_string();

  const auto parsed_unit = parsePointTimeUnit(point_time_unit_name_);
  if (!parsed_unit.has_value()) {
    RCLCPP_WARN(
      get_logger(),
      "Invalid point_time_unit '%s'; using nanoseconds",
      point_time_unit_name_.c_str());
    point_time_unit_ = PointTimeUnit::Nanoseconds;
  } else {
    point_time_unit_ = parsed_unit.value();
  }

  fallback_scan_duration_sec_ = get_parameter("fallback_scan_duration_sec").as_double();
  allow_fallback_without_point_time_ =
    get_parameter("allow_fallback_without_point_time").as_bool();

  publish_raw_if_deskew_fails_ = get_parameter("publish_raw_if_deskew_fails").as_bool();
  preserve_input_header_stamp_ = get_parameter("preserve_input_header_stamp").as_bool();
  output_frame_id_ = get_parameter("output_frame_id").as_string();

  publish_diagnostics_ = get_parameter("publish_diagnostics").as_bool();
  print_timing_debug_ = get_parameter("print_timing_debug").as_bool();

  if (!deskew_to_scan_end_) {
    RCLCPP_WARN(
      get_logger(),
      "deskew_to_scan_end=false is not implemented yet; using scan-end deskew.");
    deskew_to_scan_end_ = true;
  }

  if (use_translation_deskew_) {
    RCLCPP_WARN(
      get_logger(),
      "use_translation_deskew=true requested, but translation deskew is not implemented yet.");
  }
}

void LidarDeskewNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  ImuSample sample;
  sample.t = stampToSec(msg->header.stamp);
  sample.gyro = Eigen::Vector3d(
    msg->angular_velocity.x,
    msg->angular_velocity.y,
    msg->angular_velocity.z);
  sample.accel = Eigen::Vector3d(
    msg->linear_acceleration.x,
    msg->linear_acceleration.y,
    msg->linear_acceleration.z);

  if (!std::isfinite(sample.t) || !sample.gyro.allFinite() || !sample.accel.allFinite()) {
    return;
  }

  std::lock_guard<std::mutex> lock(imu_mutex_);

  if (!imu_buffer_.empty() && sample.t < imu_buffer_.back().t) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "Received out-of-order IMU sample; clearing IMU buffer.");
    imu_buffer_.clear();
  }

  imu_buffer_.push_back(sample);

  while (imu_buffer_.size() > max_imu_buffer_size_) {
    imu_buffer_.pop_front();
  }
}

void LidarDeskewNode::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  const auto processing_start = std::chrono::steady_clock::now();

  DeskewStats stats;
  stats.raw_point_count = static_cast<int>(pointCount(*msg));

  auto publish_raw_and_diagnostics = [&]() {
    const auto processing_end = std::chrono::steady_clock::now();
    stats.processing_time_ms =
      std::chrono::duration<double, std::milli>(processing_end - processing_start).count();
    publishRawCloud(*msg, stats);
    publishDiagnostics(stats, msg->header.stamp);
  };

  const double scan_start = stampToSec(msg->header.stamp);
  const auto point_time_summary =
    summarizePointTimes(*msg, point_time_field_name_, point_time_unit_);

  stats.has_point_time = point_time_summary.has_point_time;
  stats.min_point_rel_time_sec = point_time_summary.min_relative_time_sec;
  stats.max_point_rel_time_sec = point_time_summary.max_relative_time_sec;

  double scan_duration = point_time_summary.max_relative_time_sec;

  if (!point_time_summary.has_point_time) {
    if (!allow_fallback_without_point_time_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Point time field '%s' is missing or unreadable; publishing raw cloud.",
        point_time_field_name_.c_str());
      publish_raw_and_diagnostics();
      return;
    }

    scan_duration = fallback_scan_duration_sec_;
  }

  stats.scan_duration_sec = scan_duration;

  if (scan_duration <= 0.0 || scan_duration > max_allowed_scan_duration_sec_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "Invalid scan duration %.6f sec; publishing raw cloud.",
      scan_duration);
    publish_raw_and_diagnostics();
    return;
  }

  if (!use_rotation_deskew_) {
    stats.deskew_success = true;
    publish_raw_and_diagnostics();
    return;
  }

  const double scan_end = scan_start + scan_duration;
  const auto imu_samples = getImuSamples(scan_start, scan_end);

  stats.imu_samples_used = static_cast<int>(imu_samples.size());
  stats.max_angular_velocity_rad_s = maxAngularVelocity(imu_samples, gyro_bias_);
  stats.imu_coverage_ok = hasImuCoverage(imu_samples, scan_start, scan_end);
  if (!stats.imu_coverage_ok) {
    stats.missing_imu_count =
      std::max(0, min_required_imu_samples_ - static_cast<int>(imu_samples.size()));
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "Insufficient IMU coverage for scan [%.6f, %.6f]; samples=%zu.",
      scan_start,
      scan_end,
      imu_samples.size());
    publish_raw_and_diagnostics();
    return;
  }

  const auto* point_time_field = findPointField(*msg, point_time_field_name_);
  if (point_time_field == nullptr) {
    publish_raw_and_diagnostics();
    return;
  }

  auto output = deskewCloudRotationOnly(
    *msg,
    imu_samples,
    scan_start,
    scan_end,
    *point_time_field,
    stats);

  const auto processing_end = std::chrono::steady_clock::now();
  stats.processing_time_ms =
    std::chrono::duration<double, std::milli>(processing_end - processing_start).count();

  cloud_pub_->publish(std::move(output));
  publishDiagnostics(stats, msg->header.stamp);

  if (print_timing_debug_) {
    RCLCPP_INFO(
      get_logger(),
      "Deskew duration=%.6f sec imu=%d points=%d time=%.3f ms success=%s",
      stats.scan_duration_sec,
      stats.imu_samples_used,
      stats.deskewed_point_count,
      stats.processing_time_ms,
      stats.deskew_success ? "true" : "false");
  }
}

std::vector<ImuSample> LidarDeskewNode::getImuSamples(
  double scan_start,
  double scan_end) const
{
  std::vector<ImuSample> result;

  const double t0 = scan_start - imu_time_margin_sec_;
  const double t1 = scan_end + imu_time_margin_sec_;

  std::lock_guard<std::mutex> lock(imu_mutex_);

  for (const auto& sample : imu_buffer_) {
    if (sample.t >= t0 && sample.t <= t1) {
      result.push_back(sample);
    }
  }

  return result;
}

bool LidarDeskewNode::hasImuCoverage(
  const std::vector<ImuSample>& imu_samples,
  double scan_start,
  double scan_end) const
{
  if (imu_samples.size() < static_cast<std::size_t>(std::max(1, min_required_imu_samples_))) {
    return false;
  }

  return imu_samples.front().t <= scan_start && imu_samples.back().t >= scan_end;
}

sensor_msgs::msg::PointCloud2 LidarDeskewNode::deskewCloudRotationOnly(
  const sensor_msgs::msg::PointCloud2& cloud,
  const std::vector<ImuSample>& imu_samples,
  double scan_start,
  double scan_end,
  const sensor_msgs::msg::PointField& point_time_field,
  DeskewStats& stats) const
{
  auto output = cloud;

  if (!output_frame_id_.empty()) {
    output.header.frame_id = output_frame_id_;
  }

  if (!preserve_input_header_stamp_) {
    output.header.stamp = now();
  }

  const auto* x_field = findPointField(output, "x");
  const auto* y_field = findPointField(output, "y");
  const auto* z_field = findPointField(output, "z");
  if (x_field == nullptr || y_field == nullptr || z_field == nullptr) {
    stats.deskew_success = false;
    return output;
  }

  const auto rotations = integrateRotation(imu_samples, scan_start, scan_end, gyro_bias_);
  if (rotations.size() < 2) {
    stats.deskew_success = false;
    return output;
  }

  const Eigen::Quaterniond q_end = interpolateRotation(rotations, scan_end);

  sensor_msgs::PointCloud2Iterator<float> x_it(output, "x");
  sensor_msgs::PointCloud2Iterator<float> y_it(output, "y");
  sensor_msgs::PointCloud2Iterator<float> z_it(output, "z");

  const std::size_t total_points = pointCount(cloud);
  std::size_t deskewed_count = 0;

  for (std::size_t i = 0; i < total_points; ++i, ++x_it, ++y_it, ++z_it) {
    const auto rel_time =
      readPointRelativeTimeSec(cloud, i, point_time_field, point_time_unit_);
    if (!rel_time.has_value()) {
      continue;
    }

    const double point_time = scan_start + rel_time.value();
    const Eigen::Quaterniond q_point = interpolateRotation(rotations, point_time);
    const Eigen::Vector3d p_raw(
      static_cast<double>(*x_it),
      static_cast<double>(*y_it),
      static_cast<double>(*z_it));

    if (!p_raw.allFinite()) {
      continue;
    }

    const Eigen::Vector3d p_new = q_end.inverse() * (q_point * p_raw);

    *x_it = static_cast<float>(p_new.x());
    *y_it = static_cast<float>(p_new.y());
    *z_it = static_cast<float>(p_new.z());

    ++deskewed_count;
  }

  stats.deskewed_point_count = static_cast<int>(deskewed_count);
  stats.deskew_success = deskewed_count > 0;
  return output;
}

void LidarDeskewNode::publishRawCloud(
  const sensor_msgs::msg::PointCloud2& cloud,
  DeskewStats& stats) const
{
  if (!publish_raw_if_deskew_fails_) {
    return;
  }

  auto output = cloud;

  if (!output_frame_id_.empty()) {
    output.header.frame_id = output_frame_id_;
  }

  if (!preserve_input_header_stamp_) {
    output.header.stamp = now();
  }

  stats.deskewed_point_count = stats.raw_point_count;
  cloud_pub_->publish(std::move(output));
}

void LidarDeskewNode::publishDiagnostics(
  const DeskewStats& stats,
  const builtin_interfaces::msg::Time& stamp) const
{
  if (!publish_diagnostics_) {
    return;
  }

  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "custom_lidar_deskew";
  status.hardware_id = "lidar_deskew";

  if (stats.deskew_success) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = "deskew_success";
  } else {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = "deskew_failed_or_raw_cloud_published";
  }

  status.values.push_back(
    makeStatusValue("raw_point_count", std::to_string(stats.raw_point_count)));
  status.values.push_back(
    makeStatusValue("deskewed_point_count", std::to_string(stats.deskewed_point_count)));
  status.values.push_back(
    makeStatusValue("imu_samples_used", std::to_string(stats.imu_samples_used)));
  status.values.push_back(
    makeStatusValue("missing_imu_count", std::to_string(stats.missing_imu_count)));
  status.values.push_back(
    makeStatusValue("has_point_time", stats.has_point_time ? "true" : "false"));
  status.values.push_back(
    makeStatusValue("imu_coverage_ok", stats.imu_coverage_ok ? "true" : "false"));
  status.values.push_back(
    makeStatusValue("deskew_success", stats.deskew_success ? "true" : "false"));
  status.values.push_back(
    makeStatusValue("scan_duration_sec", std::to_string(stats.scan_duration_sec)));
  status.values.push_back(
    makeStatusValue("min_point_rel_time_sec", std::to_string(stats.min_point_rel_time_sec)));
  status.values.push_back(
    makeStatusValue("max_point_rel_time_sec", std::to_string(stats.max_point_rel_time_sec)));
  status.values.push_back(
    makeStatusValue("processing_time_ms", std::to_string(stats.processing_time_ms)));
  status.values.push_back(
    makeStatusValue(
      "max_angular_velocity_rad_s",
      std::to_string(stats.max_angular_velocity_rad_s)));

  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = stamp;
  array.status.push_back(std::move(status));

  diagnostics_pub_->publish(std::move(array));
}

double LidarDeskewNode::stampToSec(const builtin_interfaces::msg::Time& stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

}  // namespace custom_lidar_deskew

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<custom_lidar_deskew::LidarDeskewNode>());
  rclcpp::shutdown();
  return 0;
}
