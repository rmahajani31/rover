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
    "optimization_time_ms",
    diagnostics.optimization_time_ms));
  status.values.push_back(makeKeyValue(
    "map_update_time_ms",
    diagnostics.map_update_time_ms));

  array.status.push_back(status);
  return array;
}

}  // namespace custom_scan_to_map_odom
