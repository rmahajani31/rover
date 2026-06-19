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

  status.values.push_back(makeKeyValue(
    "map_initialized",
    diagnostics.map_initialized ? "true" : "false"));
  status.values.push_back(makeKeyValue(
    "input_points",
    std::to_string(diagnostics.input_points)));
  status.values.push_back(makeKeyValue(
    "downsampled_points",
    std::to_string(diagnostics.downsampled_points)));
  status.values.push_back(makeKeyValue(
    "map_points",
    std::to_string(diagnostics.map_points)));
  status.values.push_back(makeKeyValue(
    "optimization_status",
    diagnostics.optimization.status));
  status.values.push_back(makeKeyValue(
    "optimization_success",
    diagnostics.optimization.success ? "true" : "false"));
  status.values.push_back(makeKeyValue(
    "valid_correspondences",
    std::to_string(diagnostics.optimization.valid_correspondences)));
  status.values.push_back(makeKeyValue(
    "valid_planes",
    std::to_string(diagnostics.optimization.valid_planes)));
  status.values.push_back(makeKeyValue(
    "mean_residual",
    std::to_string(diagnostics.optimization.mean_residual)));
  status.values.push_back(makeKeyValue(
    "max_residual",
    std::to_string(diagnostics.optimization.max_residual)));
  status.values.push_back(makeKeyValue(
    "final_update_translation_norm",
    std::to_string(diagnostics.optimization.final_update_translation_norm)));
  status.values.push_back(makeKeyValue(
    "final_update_rotation_norm",
    std::to_string(diagnostics.optimization.final_update_rotation_norm)));
  status.values.push_back(makeKeyValue(
    "iterations",
    std::to_string(diagnostics.optimization.iterations)));
  status.values.push_back(makeKeyValue(
    "imu_initial_guess_enabled",
    diagnostics.imu_initial_guess_enabled ? "true" : "false"));
  status.values.push_back(makeKeyValue(
    "used_imu_guess",
    diagnostics.used_imu_guess ? "true" : "false"));
  status.values.push_back(makeKeyValue(
    "imu_prediction_success",
    diagnostics.imu_prediction_success ? "true" : "false"));
  status.values.push_back(makeKeyValue(
    "imu_prediction_status",
    diagnostics.imu_prediction_status));
  status.values.push_back(makeKeyValue(
    "imu_samples_used",
    std::to_string(diagnostics.imu_samples_used)));
  status.values.push_back(makeKeyValue(
    "imu_dt_total",
    std::to_string(diagnostics.imu_dt_total)));
  status.values.push_back(makeKeyValue(
    "imu_delta_roll_deg",
    std::to_string(diagnostics.imu_delta_roll_deg)));
  status.values.push_back(makeKeyValue(
    "imu_delta_pitch_deg",
    std::to_string(diagnostics.imu_delta_pitch_deg)));
  status.values.push_back(makeKeyValue(
    "imu_delta_yaw_deg",
    std::to_string(diagnostics.imu_delta_yaw_deg)));

  array.status.push_back(status);
  return array;
}

}  // namespace custom_scan_to_map_odom
