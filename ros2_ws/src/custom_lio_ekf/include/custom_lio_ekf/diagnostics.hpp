#pragma once

#include <cstddef>
#include <string>

#include <builtin_interfaces/msg/time.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>

#include "custom_lio_ekf/covariance_propagation.hpp"
#include "custom_lio_ekf/iterated_lidar_update.hpp"

namespace custom_lio_ekf
{

struct LioEkfDiagnostics
{
  bool map_initialized = false;

  std::size_t input_points = 0;
  std::size_t map_points = 0;

  double prediction_time_ms = 0.0;
  double lidar_update_time_ms = 0.0;
  double map_update_time_ms = 0.0;

  EkfPredictionStats prediction;
  LidarUpdateStats lidar_update;
};

diagnostic_msgs::msg::DiagnosticArray makeDiagnosticArray(
  const LioEkfDiagnostics& diagnostics,
  const builtin_interfaces::msg::Time& stamp,
  const std::string& name = "custom_lio_ekf");

}  // namespace custom_lio_ekf
