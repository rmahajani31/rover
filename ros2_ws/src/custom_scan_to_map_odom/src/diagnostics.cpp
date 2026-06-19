#include "custom_scan_to_map_odom/diagnostics.hpp"

#include <cstdint>
#include <string>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>

namespace custom_scan_to_map_odom
{

namespace
{

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

  array.status.push_back(status);
  return array;
}

}  // namespace custom_scan_to_map_odom
