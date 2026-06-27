#include "custom_lio_ekf/iterated_lidar_update.hpp"

#include <algorithm>
#include <cmath>

#include <Eigen/Cholesky>

namespace custom_lio_ekf
{

namespace
{

Eigen::Vector3d pointToEigen(const custom_scan_to_map_odom::PointT& point)
{
  return Eigen::Vector3d(
    static_cast<double>(point.x),
    static_cast<double>(point.y),
    static_cast<double>(point.z));
}

bool invertPositiveDefiniteMatrix(
  const Matrix18d& matrix,
  Matrix18d& inverse)
{
  Eigen::LDLT<Matrix18d> ldlt(matrix);

  if (ldlt.info() != Eigen::Success) {
    return false;
  }

  inverse = ldlt.solve(Matrix18d::Identity());
  return inverse.allFinite();
}

void updateResidualStats(
  LidarUpdateStats& stats,
  double residual,
  double& sum_abs_residual,
  double& sum_squared_residual)
{
  const double abs_residual = std::abs(residual);

  sum_abs_residual += abs_residual;
  sum_squared_residual += residual * residual;
  stats.max_abs_residual = std::max(stats.max_abs_residual, abs_residual);
}

void finalizeResidualStats(
  LidarUpdateStats& stats,
  double sum_abs_residual,
  double sum_squared_residual)
{
  if (stats.valid_residuals == 0) {
    return;
  }

  const double count = static_cast<double>(stats.valid_residuals);
  stats.mean_abs_residual = sum_abs_residual / count;
  stats.rms_residual = std::sqrt(sum_squared_residual / count);
}

}  // namespace

bool buildLidarNormalEquations(
  const custom_scan_to_map_odom::CloudTConstPtr& scan_lidar_frame,
  const EkfState& linearization_state,
  const Matrix18d& prior_information,
  const LidarImuExtrinsics& extrinsics,
  const custom_scan_to_map_odom::LocalMap& local_map,
  const custom_scan_to_map_odom::PlaneFitter& plane_fitter,
  const LidarUpdateOptions& options,
  LidarNormalEquations& equations)
{
  equations = LidarNormalEquations{};
  equations.information = prior_information;
  equations.rhs = Vector18d::Zero();

  if (!scan_lidar_frame || scan_lidar_frame->empty()) {
    equations.status = "empty_scan";
    equations.stats.status = equations.status;
    return false;
  }

  double sum_abs_residual = 0.0;
  double sum_squared_residual = 0.0;

  equations.stats.input_points = scan_lidar_frame->size();

  for (const auto& point : scan_lidar_frame->points) {
    LidarResidual residual;
    const bool valid = buildPointToPlaneResidual(
      pointToEigen(point),
      linearization_state,
      extrinsics,
      local_map,
      plane_fitter,
      options,
      residual);

    if (!valid) {
      continue;
    }

    equations.information +=
      residual.information_weight * residual.H.transpose() * residual.H;
    equations.rhs -=
      residual.information_weight * residual.H.transpose() * residual.residual;

    ++equations.stats.valid_residuals;
    updateResidualStats(
      equations.stats,
      residual.residual,
      sum_abs_residual,
      sum_squared_residual);
  }

  finalizeResidualStats(equations.stats, sum_abs_residual, sum_squared_residual);

  if (equations.stats.valid_residuals < options.min_valid_residuals) {
    equations.status = "too_few_valid_residuals";
    equations.stats.status = equations.status;
    return false;
  }

  if (!equations.information.allFinite() || !equations.rhs.allFinite()) {
    equations.status = "nonfinite_normal_equations";
    equations.stats.status = equations.status;
    return false;
  }

  equations.success = true;
  equations.status = "success";
  equations.stats.success = true;
  equations.stats.status = "success";
  return true;
}

bool solveLidarCorrection(
  const Matrix18d& information,
  const Vector18d& rhs,
  Vector18d& delta_x)
{
  Eigen::LDLT<Matrix18d> ldlt(information);

  if (ldlt.info() != Eigen::Success) {
    delta_x = Vector18d::Zero();
    return false;
  }

  delta_x = ldlt.solve(rhs);
  return delta_x.allFinite();
}

bool applyIteratedLidarUpdate(
  EkfState& state,
  const custom_scan_to_map_odom::CloudTConstPtr& scan_lidar_frame,
  const LidarImuExtrinsics& extrinsics,
  const custom_scan_to_map_odom::LocalMap& local_map,
  const LidarUpdateOptions& options,
  LidarUpdateStats& stats)
{
  stats = LidarUpdateStats{};

  if (!isFinite(state)) {
    stats.status = "nonfinite_input_state";
    return false;
  }

  Matrix18d prior_information;
  if (!invertPositiveDefiniteMatrix(state.P, prior_information)) {
    stats.status = "invalid_prior_covariance";
    return false;
  }

  custom_scan_to_map_odom::PlaneFitter plane_fitter(
    planeFitterOptionsFromLidarOptions(options));

  EkfState iter_state = state;
  LidarNormalEquations final_equations;

  for (int iter = 0; iter < options.max_iterations; ++iter) {
    LidarNormalEquations equations;
    if (!buildLidarNormalEquations(
        scan_lidar_frame,
        iter_state,
        prior_information,
        extrinsics,
        local_map,
        plane_fitter,
        options,
        equations)) {
      stats = equations.stats;
      stats.iterations = iter + 1;
      return false;
    }

    Vector18d delta_x = Vector18d::Zero();
    if (!solveLidarCorrection(equations.information, equations.rhs, delta_x)) {
      stats = equations.stats;
      stats.iterations = iter + 1;
      stats.success = false;
      stats.status = "linear_solve_failed";
      return false;
    }

    const double delta_theta_norm =
      delta_x.segment<3>(kThetaOffset).norm();
    const double delta_position_norm =
      delta_x.segment<3>(kPositionOffset).norm();

    if (delta_theta_norm > options.max_correction_theta_norm ||
        delta_position_norm > options.max_correction_position_norm) {
      stats = equations.stats;
      stats.iterations = iter + 1;
      stats.success = false;
      stats.status = "correction_too_large";
      stats.final_delta_theta_norm = delta_theta_norm;
      stats.final_delta_position_norm = delta_position_norm;
      return false;
    }

    injectError(iter_state, delta_x);

    stats = equations.stats;
    stats.iterations = iter + 1;
    stats.final_delta_theta_norm = delta_theta_norm;
    stats.final_delta_position_norm = delta_position_norm;

    final_equations = equations;

    if (delta_theta_norm < options.convergence_theta_norm &&
        delta_position_norm < options.convergence_position_norm) {
      break;
    }
  }

  LidarNormalEquations covariance_equations;
  if (buildLidarNormalEquations(
      scan_lidar_frame,
      iter_state,
      prior_information,
      extrinsics,
      local_map,
      plane_fitter,
      options,
      covariance_equations)) {
    final_equations = covariance_equations;
    stats.valid_residuals = covariance_equations.stats.valid_residuals;
    stats.mean_abs_residual = covariance_equations.stats.mean_abs_residual;
    stats.rms_residual = covariance_equations.stats.rms_residual;
    stats.max_abs_residual = covariance_equations.stats.max_abs_residual;
  }

  Matrix18d posterior_covariance;
  if (!invertPositiveDefiniteMatrix(final_equations.information, posterior_covariance)) {
    stats.success = false;
    stats.status = "posterior_covariance_update_failed";
    return false;
  }

  iter_state.P = posterior_covariance;
  symmetrizeCovariance(iter_state);

  if (!isFinite(iter_state)) {
    stats.success = false;
    stats.status = "nonfinite_updated_state";
    return false;
  }

  state = iter_state;

  stats.success = true;
  stats.status = "success";
  return true;
}

}  // namespace custom_lio_ekf
