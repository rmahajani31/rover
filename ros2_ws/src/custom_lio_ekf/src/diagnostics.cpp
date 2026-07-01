#include "custom_lio_ekf/diagnostics.hpp"

#include <cstdint>
#include <string>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>

namespace custom_lio_ekf
{

namespace
{

diagnostic_msgs::msg::KeyValue makeKeyValue(
  const std::string& key,
  const std::string& value)
{
  diagnostic_msgs::msg::KeyValue key_value;
  key_value.key = key;
  key_value.value = value;
  return key_value;
}

void appendBackendProfileValues(
  std::vector<diagnostic_msgs::msg::KeyValue>& values,
  const std::string& prefix,
  const custom_ikd_tree_backend::BackendProfileSnapshot& profile)
{
  values.push_back(makeKeyValue(
    prefix + "_status",
    profile.status));
  values.push_back(makeKeyValue(
    prefix + "_map_size",
    std::to_string(profile.map_size)));
  values.push_back(makeKeyValue(
    prefix + "_active_size",
    std::to_string(profile.active_size)));
  values.push_back(makeKeyValue(
    prefix + "_invalid_node_count",
    std::to_string(profile.invalid_node_count)));
  values.push_back(makeKeyValue(
    prefix + "_invalid_ratio",
    std::to_string(profile.invalid_ratio)));
  values.push_back(makeKeyValue(
    prefix + "_knn_queries",
    std::to_string(profile.knn_query_count)));
  values.push_back(makeKeyValue(
    prefix + "_inserted_points",
    std::to_string(profile.inserted_point_count)));
  values.push_back(makeKeyValue(
    prefix + "_deleted_points",
    std::to_string(profile.deleted_point_count)));
  values.push_back(makeKeyValue(
    prefix + "_rejected_by_voxel",
    std::to_string(profile.rejected_by_voxel_count)));
  values.push_back(makeKeyValue(
    prefix + "_voxel_replacements",
    std::to_string(profile.voxel_replacement_count)));
  values.push_back(makeKeyValue(
    prefix + "_rebuild_count",
    std::to_string(profile.rebuild_count)));
  values.push_back(makeKeyValue(
    prefix + "_knn_time_ms",
    std::to_string(profile.knn_time_ms)));
  values.push_back(makeKeyValue(
    prefix + "_insert_time_ms",
    std::to_string(profile.insert_time_ms)));
  values.push_back(makeKeyValue(
    prefix + "_delete_time_ms",
    std::to_string(profile.delete_time_ms)));
  values.push_back(makeKeyValue(
    prefix + "_downsample_time_ms",
    std::to_string(profile.downsample_time_ms)));
  values.push_back(makeKeyValue(
    prefix + "_rebuild_time_ms",
    std::to_string(profile.rebuild_time_ms)));
  values.push_back(makeKeyValue(
    prefix + "_total_time_ms",
    std::to_string(profile.total_backend_time_ms)));
}

std::uint8_t diagnosticLevel(const LioEkfDiagnostics& diagnostics)
{
  if (diagnostics.lidar_update.success &&
      diagnostics.map_initialized &&
      diagnostics.tf_lookup_success &&
      diagnostics.odom_publish_success) {
    return diagnostic_msgs::msg::DiagnosticStatus::OK;
  }

  const std::string& status = diagnostics.lidar_update.status;

  // Startup and sparse-data states are actionable but not necessarily fatal.
  if (!diagnostics.map_initialized ||
      status == "not_started" ||
      status == "empty_filtered_scan" ||
      status == "waiting_for_initial_imu_calibration" ||
      status == "missing_imu_interval" ||
      status == "prediction_failed" ||
      (diagnostics.lidar_update.success && !diagnostics.tf_lookup_success) ||
      (diagnostics.lidar_update.success && !diagnostics.odom_publish_success) ||
      status == "too_few_valid_residuals") {
    return diagnostic_msgs::msg::DiagnosticStatus::WARN;
  }

  return diagnostic_msgs::msg::DiagnosticStatus::ERROR;
}

std::string diagnosticMessage(const LioEkfDiagnostics& diagnostics)
{
  if (!diagnostics.map_initialized) {
    if (!diagnostics.lidar_update.status.empty() &&
        diagnostics.lidar_update.status != "not_started") {
      return diagnostics.lidar_update.status;
    }
    return "local_map_not_initialized";
  }

  if (diagnostics.lidar_update.success && !diagnostics.tf_lookup_success) {
    return "tf_lookup_failed";
  }

  if (diagnostics.lidar_update.success && !diagnostics.odom_publish_success) {
    return "odom_publish_failed";
  }

  if (!diagnostics.prediction.success &&
      !diagnostics.prediction.status.empty() &&
      diagnostics.prediction.status != "not_started") {
    return diagnostics.prediction.status;
  }

  return diagnostics.lidar_update.status;
}

}  // namespace

diagnostic_msgs::msg::DiagnosticArray makeDiagnosticArray(
  const LioEkfDiagnostics& diagnostics,
  const builtin_interfaces::msg::Time& stamp,
  const std::string& name)
{
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = stamp;

  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = name;
  status.hardware_id = "custom_lio_ekf";
  status.level = diagnosticLevel(diagnostics);
  status.message = diagnosticMessage(diagnostics);

  status.values.push_back(makeKeyValue(
    "frame_count",
    std::to_string(diagnostics.frame_count)));
  status.values.push_back(makeKeyValue(
    "map_initialized",
    diagnostics.map_initialized ? "true" : "false"));
  status.values.push_back(makeKeyValue(
    "input_points",
    std::to_string(diagnostics.input_points)));
  status.values.push_back(makeKeyValue(
    "map_points",
    std::to_string(diagnostics.map_points)));
  status.values.push_back(makeKeyValue(
    "local_map_points_before_update",
    std::to_string(diagnostics.local_map_points_before_update)));
  status.values.push_back(makeKeyValue(
    "local_map_points_after_update",
    std::to_string(diagnostics.local_map_points_after_update)));
  status.values.push_back(makeKeyValue(
    "imu_samples_buffered",
    std::to_string(diagnostics.imu_samples_buffered)));
  status.values.push_back(makeKeyValue(
    "imu_samples_received",
    std::to_string(diagnostics.imu_samples_received)));
  status.values.push_back(makeKeyValue(
    "consecutive_tracking_failures",
    std::to_string(diagnostics.consecutive_tracking_failures)));
  status.values.push_back(makeKeyValue(
    "tf_lookup_success",
    diagnostics.tf_lookup_success ? "true" : "false"));
  status.values.push_back(makeKeyValue(
    "odom_publish_success",
    diagnostics.odom_publish_success ? "true" : "false"));
  status.values.push_back(makeKeyValue(
    "prediction_success",
    diagnostics.prediction.success ? "true" : "false"));
  status.values.push_back(makeKeyValue(
    "prediction_status",
    diagnostics.prediction.status));
  status.values.push_back(makeKeyValue(
    "imu_intervals_integrated",
    std::to_string(diagnostics.prediction.intervals_integrated)));
  status.values.push_back(makeKeyValue(
    "imu_dt_total",
    std::to_string(diagnostics.prediction.dt_total)));
  status.values.push_back(makeKeyValue(
    "lidar_update_success",
    diagnostics.lidar_update.success ? "true" : "false"));
  status.values.push_back(makeKeyValue(
    "lidar_update_status",
    diagnostics.lidar_update.status));
  status.values.push_back(makeKeyValue(
    "lidar_iterations",
    std::to_string(diagnostics.lidar_update.iterations)));
  status.values.push_back(makeKeyValue(
    "valid_residuals",
    std::to_string(diagnostics.lidar_update.valid_residuals)));
  status.values.push_back(makeKeyValue(
    "mean_abs_residual",
    std::to_string(diagnostics.lidar_update.mean_abs_residual)));
  status.values.push_back(makeKeyValue(
    "rms_residual",
    std::to_string(diagnostics.lidar_update.rms_residual)));
  status.values.push_back(makeKeyValue(
    "max_abs_residual",
    std::to_string(diagnostics.lidar_update.max_abs_residual)));
  status.values.push_back(makeKeyValue(
    "delta_theta_norm",
    std::to_string(diagnostics.lidar_update.final_delta_theta_norm)));
  status.values.push_back(makeKeyValue(
    "delta_position_norm",
    std::to_string(diagnostics.lidar_update.final_delta_position_norm)));
  status.values.push_back(makeKeyValue(
    "prediction_time_ms",
    std::to_string(diagnostics.prediction_time_ms)));
  status.values.push_back(makeKeyValue(
    "lidar_update_time_ms",
    std::to_string(diagnostics.lidar_update_time_ms)));
  status.values.push_back(makeKeyValue(
    "map_update_time_ms",
    std::to_string(diagnostics.map_update_time_ms)));
  status.values.push_back(makeKeyValue(
    "map_backend_type",
    diagnostics.map_backend_type));
  appendBackendProfileValues(
    status.values,
    "map_backend_latest",
    diagnostics.map_backend_profile);
  appendBackendProfileValues(
    status.values,
    "map_backend_lidar",
    diagnostics.map_backend_lidar_profile);
  appendBackendProfileValues(
    status.values,
    "map_backend_update",
    diagnostics.map_backend_update_profile);

  array.status.push_back(status);
  return array;
}

}  // namespace custom_lio_ekf
