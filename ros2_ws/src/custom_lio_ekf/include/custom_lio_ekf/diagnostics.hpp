#pragma once

#include <cstddef>
#include <string>

#include <builtin_interfaces/msg/time.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>

#include "custom_lio_ekf/covariance_propagation.hpp"
#include "custom_lio_ekf/iterated_lidar_update.hpp"
#include "custom_ikd_tree_backend/backend_profiler.hpp"

namespace custom_lio_ekf
{

struct LioEkfDiagnostics
{
  bool map_initialized = false;

  std::size_t frame_count = 0;
  std::size_t input_points = 0;
  std::size_t map_points = 0;
  std::size_t local_map_points_before_update = 0;
  std::size_t local_map_points_after_update = 0;
  std::size_t imu_samples_buffered = 0;
  std::size_t imu_samples_received = 0;
  int consecutive_tracking_failures = 0;

  bool tf_lookup_success = false;
  bool odom_publish_success = false;

  double cloud_filter_time_ms = 0.0;
  double prediction_time_ms = 0.0;
  double lidar_update_time_ms = 0.0;
  double map_update_time_ms = 0.0;
  double total_callback_time_ms = 0.0;

  std::string map_backend_type;
  custom_ikd_tree_backend::BackendProfileSnapshot map_backend_profile;
  custom_ikd_tree_backend::BackendProfileSnapshot map_backend_lidar_profile;
  custom_ikd_tree_backend::BackendProfileSnapshot map_backend_update_profile;

  EkfPredictionStats prediction;
  LidarUpdateStats lidar_update;
};

diagnostic_msgs::msg::DiagnosticArray makeDiagnosticArray(
  const LioEkfDiagnostics& diagnostics,
  const builtin_interfaces::msg::Time& stamp,
  const std::string& name = "custom_lio_ekf");

}  // namespace custom_lio_ekf
