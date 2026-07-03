#pragma once

#include <cstddef>
#include <string>

#include "custom_ikd_tree_backend/map_backend_interface.hpp"
#include "custom_scan_to_map_odom/ros_conversions.hpp"
#include "custom_lio_ekf/ekf_parameters.hpp"
#include "custom_lio_ekf/ekf_state.hpp"
#include "custom_lio_ekf/lidar_residual.hpp"

namespace custom_lio_ekf
{

struct LidarUpdateStats
{
  bool success = false;
  std::string status = "not_started";

  int iterations = 0;
  std::size_t input_points = 0;
  std::size_t valid_residuals = 0;

  double mean_abs_residual = 0.0;
  double rms_residual = 0.0;
  double max_abs_residual = 0.0;

  double final_delta_theta_norm = 0.0;
  double final_delta_position_norm = 0.0;
};

struct LidarNormalEquations
{
  bool success = false;
  std::string status = "not_started";

  Matrix18d information = Matrix18d::Zero();
  Vector18d rhs = Vector18d::Zero();

  LidarUpdateStats stats;
};

bool buildLidarNormalEquations(
  const custom_scan_to_map_odom::CloudTConstPtr& scan_lidar_frame,
  const EkfState& linearization_state,
  const Matrix18d& prior_information,
  const Vector18d& prior_error,
  bool build_rhs,
  const LidarImuExtrinsics& extrinsics,
  const custom_ikd_tree_backend::MapBackendInterface& map_backend,
  const custom_scan_to_map_odom::PlaneFitter& plane_fitter,
  const LidarUpdateOptions& options,
  LidarNormalEquations& equations);

bool solveLidarCorrection(
  const Matrix18d& information,
  const Vector18d& rhs,
  Vector18d& delta_x);

bool applyIteratedLidarUpdate(
  EkfState& state,
  const custom_scan_to_map_odom::CloudTConstPtr& scan_lidar_frame,
  const LidarImuExtrinsics& extrinsics,
  const custom_ikd_tree_backend::MapBackendInterface& map_backend,
  const LidarUpdateOptions& options,
  LidarUpdateStats& stats);

}  // namespace custom_lio_ekf
