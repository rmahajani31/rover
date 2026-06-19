#include "custom_scan_to_map_odom/diagnostics.hpp"

#include <cstdint>
#include <string>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>

namespace custom_scan_to_map_odom
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

diagnostic_msgs::msg::KeyValue makeKeyValue(
  const std::string& key,
  std::size_t value)
{
  return makeKeyValue(key, std::to_string(value));
}

diagnostic_msgs::msg::KeyValue makeKeyValue(
  const std::string& key,
  int value)
{
  return makeKeyValue(key, std::to_string(value));
}

diagnostic_msgs::msg::KeyValue makeKeyValue(
  const std::string& key,
  double value)
{
  return makeKeyValue(key, std::to_string(value));
}

diagnostic_msgs::msg::KeyValue makeKeyValue(
  const std::string& key,
  bool value)
{
  return makeKeyValue(key, value ? "true" : "false");
}

std::uint8_t diagnosticLevel(const ScanToMapDiagnostics& diagnostics)
{
  if (diagnostics.optimization.success) {
    return diagnostic_msgs::msg::DiagnosticStatus::OK;
  }

  const std::string& status = diagnostics.optimization.status;

  // Startup and sparse-data states are actionable but not necessarily fatal.
  if (status == "not_started" ||
      status == "empty_scan" ||
      status == "map_not_ready" ||
      status == "not_enough_correspondences") {
    return diagnostic_msgs::msg::DiagnosticStatus::WARN;
  }

  return diagnostic_msgs::msg::DiagnosticStatus::ERROR;
}

std::string diagnosticMessage(const ScanToMapDiagnostics& diagnostics)
{
  if (!diagnostics.map_initialized) {
    return "map_not_initialized";
  }

  return diagnostics.optimization.status;
}

}  // namespace

diagnostic_msgs::msg::DiagnosticArray makeDiagnosticArray(
  const ScanToMapDiagnostics& diagnostics,
  const builtin_interfaces::msg::Time& stamp,
  const std::string& name)
{
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = stamp;

  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = name;
  status.hardware_id = "custom_scan_to_map_odom";
  status.level = diagnosticLevel(diagnostics);
  status.message = diagnosticMessage(diagnostics);

  // Keep diagnostics flat so ros2 topic echo and rqt_robot_monitor remain easy to scan.
  status.values.push_back(makeKeyValue("map_initialized", diagnostics.map_initialized));
  status.values.push_back(makeKeyValue("input_points", diagnostics.input_points));
  status.values.push_back(makeKeyValue("downsampled_points", diagnostics.downsampled_points));
  status.values.push_back(makeKeyValue("map_points", diagnostics.map_points));

  status.values.push_back(makeKeyValue(
    "optimization_success",
    diagnostics.optimization.success));
  status.values.push_back(makeKeyValue(
    "optimization_status",
    diagnostics.optimization.status));
  status.values.push_back(makeKeyValue(
    "valid_correspondences",
    diagnostics.optimization.valid_correspondences));
  status.values.push_back(makeKeyValue(
    "valid_planes",
    diagnostics.optimization.valid_planes));
  status.values.push_back(makeKeyValue(
    "mean_residual",
    diagnostics.optimization.mean_residual));
  status.values.push_back(makeKeyValue(
    "max_residual",
    diagnostics.optimization.max_residual));
  status.values.push_back(makeKeyValue(
    "final_update_translation_norm",
    diagnostics.optimization.final_update_translation_norm));
  status.values.push_back(makeKeyValue(
    "final_update_rotation_norm",
    diagnostics.optimization.final_update_rotation_norm));
  status.values.push_back(makeKeyValue(
    "iterations",
    diagnostics.optimization.iterations));

  status.values.push_back(makeKeyValue(
    "imu_initial_guess_enabled",
    diagnostics.imu_initial_guess_enabled));
  status.values.push_back(makeKeyValue(
    "used_imu_guess",
    diagnostics.used_imu_guess));
  status.values.push_back(makeKeyValue(
    "imu_prediction_success",
    diagnostics.imu_prediction_success));
  status.values.push_back(makeKeyValue(
    "imu_prediction_status",
    diagnostics.imu_prediction_status));
  status.values.push_back(makeKeyValue(
    "imu_samples_used",
    diagnostics.imu_samples_used));
  status.values.push_back(makeKeyValue(
    "imu_dt_total",
    diagnostics.imu_dt_total));
  status.values.push_back(makeKeyValue(
    "imu_delta_roll_deg",
    diagnostics.imu_delta_roll_deg));
  status.values.push_back(makeKeyValue(
    "imu_delta_pitch_deg",
    diagnostics.imu_delta_pitch_deg));
  status.values.push_back(makeKeyValue(
    "imu_delta_yaw_deg",
    diagnostics.imu_delta_yaw_deg));

  status.values.push_back(makeKeyValue(
    "optimization_time_ms",
    diagnostics.optimization_time_ms));
  status.values.push_back(makeKeyValue(
    "map_update_time_ms",
    diagnostics.map_update_time_ms));

  array.status.push_back(status);

  diagnostic_msgs::msg::DiagnosticStatus local_map_status;
  local_map_status.name = name + "/local_map_manager";
  local_map_status.hardware_id = "local_map";
  local_map_status.level = diagnostics.map_initialized ?
    diagnostic_msgs::msg::DiagnosticStatus::OK :
    diagnostic_msgs::msg::DiagnosticStatus::WARN;
  local_map_status.message = diagnostics.map_initialized ?
    "local_map_manager_running" : "map_not_initialized";

  const auto& local_map = diagnostics.local_map;
  local_map_status.values.push_back(makeKeyValue(
    "map_size_before_update",
    local_map.map_size_before_update));
  local_map_status.values.push_back(makeKeyValue(
    "map_size_after_crop",
    local_map.map_size_after_crop));
  local_map_status.values.push_back(makeKeyValue(
    "map_size_after_insert",
    local_map.map_size_after_insert));
  local_map_status.values.push_back(makeKeyValue(
    "map_size_after_downsample",
    local_map.map_size_after_downsample));
  local_map_status.values.push_back(makeKeyValue(
    "inserted_points",
    local_map.inserted_points));
  local_map_status.values.push_back(makeKeyValue(
    "removed_points_outside_cube",
    local_map.removed_points_outside_cube));
  local_map_status.values.push_back(makeKeyValue(
    "cube_shifted",
    local_map.cube_shifted));
  local_map_status.values.push_back(makeKeyValue(
    "cube_center_x",
    local_map.cube_center_x));
  local_map_status.values.push_back(makeKeyValue(
    "cube_center_y",
    local_map.cube_center_y));
  local_map_status.values.push_back(makeKeyValue(
    "cube_center_z",
    local_map.cube_center_z));
  local_map_status.values.push_back(makeKeyValue(
    "crop_time_ms",
    local_map.crop_time_ms));
  local_map_status.values.push_back(makeKeyValue(
    "insert_time_ms",
    local_map.insert_time_ms));
  local_map_status.values.push_back(makeKeyValue(
    "downsample_time_ms",
    local_map.downsample_time_ms));
  local_map_status.values.push_back(makeKeyValue(
    "kdtree_rebuild_time_ms",
    local_map.kdtree_rebuild_time_ms));
  local_map_status.values.push_back(makeKeyValue(
    "total_update_time_ms",
    local_map.total_update_time_ms));

  array.status.push_back(local_map_status);
  return array;
}

}  // namespace custom_scan_to_map_odom
