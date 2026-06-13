#pragma once

#include <cstddef>
#include <string>

#include <builtin_interfaces/msg/time.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>

#include "custom_scan_to_map_odom/scan_to_map_optimizer.hpp"

namespace custom_scan_to_map_odom
{

struct ScanToMapDiagnostics
{
  bool map_initialized = false;

  std::size_t input_points = 0;
  std::size_t downsampled_points = 0;
  std::size_t map_points = 0;

  double optimization_time_ms = 0.0;
  double map_update_time_ms = 0.0;

  OptimizationStats optimization;
};

diagnostic_msgs::msg::DiagnosticArray makeDiagnosticArray(
  const ScanToMapDiagnostics& diagnostics,
  const builtin_interfaces::msg::Time& stamp,
  const std::string& name = "custom_scan_to_map_odom");

}  // namespace custom_scan_to_map_odom
