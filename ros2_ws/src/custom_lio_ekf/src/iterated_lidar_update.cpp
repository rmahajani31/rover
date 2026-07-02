#include "custom_lio_ekf/iterated_lidar_update.hpp"

#include <algorithm>
#include <cmath>

#include <Eigen/Cholesky>

namespace custom_lio_ekf
{

namespace
{

constexpr double kMinPriorCovarianceDiagonal = 1.0e-12;

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
  // Covariance and information matrices should stay symmetric positive definite.
  const Matrix18d symmetric_matrix = 0.5 * (matrix + matrix.transpose());
  Eigen::LDLT<Matrix18d> ldlt(symmetric_matrix);

  if (ldlt.info() != Eigen::Success || !ldlt.isPositive()) {
    return false;
  }

  inverse = ldlt.solve(Matrix18d::Identity());
  inverse = 0.5 * (inverse + inverse.transpose());
  return inverse.allFinite();
}

bool regularizePriorCovariance(EkfState& prior_state)
{
  symmetrizeCovariance(prior_state);

  if (!prior_state.P.allFinite()) {
    return false;
  }

  for (int i = 0; i < kErrorStateDim; ++i) {
    if (!std::isfinite(prior_state.P(i, i))) {
      return false;
    }

    prior_state.P(i, i) =
      std::max(prior_state.P(i, i), kMinPriorCovarianceDiagonal);
  }

  return prior_state.P.allFinite();
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
  const Vector18d& prior_error,
  bool build_rhs,
  const LidarImuExtrinsics& extrinsics,
  const custom_ikd_tree_backend::MapBackendInterface& map_backend,
  const custom_scan_to_map_odom::PlaneFitter& plane_fitter,
  const LidarUpdateOptions& options,
  LidarNormalEquations& equations)
{
  equations = LidarNormalEquations{};
  // Start with the EKF prior centered at x_minus:
  // rhs = -P_minus^-1 * (x_iter boxminus x_minus).
  equations.information = prior_information;
  equations.rhs =
    build_rhs ? -prior_information * prior_error : Vector18d::Zero();

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
      map_backend,
      plane_fitter,
      options,
      residual);

    if (!valid) {
      continue;
    }

    if (!std::isfinite(residual.residual) ||
        !std::isfinite(residual.information_weight) ||
        residual.information_weight <= 0.0 ||
        !residual.H.allFinite()) {
      continue;
    }

    // Accumulate weighted Gauss-Newton normal equations for H * delta = -r.
    equations.information +=
      residual.information_weight * residual.H.transpose() * residual.H;

    if (build_rhs) {
      equations.rhs -=
        residual.information_weight * residual.H.transpose() * residual.residual;
    }

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
  const custom_ikd_tree_backend::MapBackendInterface& map_backend,
  const LidarUpdateOptions& options,
  LidarUpdateStats& stats)
{
  stats = LidarUpdateStats{};

  if (!isFinite(state)) {
    stats.status = "nonfinite_input_state";
    return false;
  }

  // Work from a local prior copy so failed LiDAR updates cannot mutate state.
  EkfState prior_state = state;
  if (!regularizePriorCovariance(prior_state)) {
    stats.status = "invalid_prior_covariance";
    return false;
  }

  Matrix18d prior_information;
  if (!invertPositiveDefiniteMatrix(prior_state.P, prior_information)) {
    stats.status = "prior_information_update_failed";
    return false;
  }

  custom_scan_to_map_odom::PlaneFitter plane_fitter(
    planeFitterOptionsFromLidarOptions(options));

  // Iterate on a copy so a rejected update cannot corrupt the live EKF state.
  EkfState iter_state = prior_state;
  LidarNormalEquations final_equations;

  for (int iter = 0; iter < options.max_iterations; ++iter) {
    const Vector18d prior_error =
      errorStateDifference(iter_state, prior_state);

    LidarNormalEquations equations;
    if (!buildLidarNormalEquations(
        scan_lidar_frame,
        iter_state,
        prior_information,
        prior_error,
        true,
        extrinsics,
        map_backend,
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

  const Vector18d prior_error =
    errorStateDifference(iter_state, prior_state);

  const int completed_iterations = stats.iterations;
  LidarNormalEquations covariance_equations;
  if (!buildLidarNormalEquations(
      scan_lidar_frame,
      iter_state,
      prior_information,
      prior_error,
      false,
      extrinsics,
      map_backend,
      plane_fitter,
      options,
      covariance_equations)) {
    stats = covariance_equations.stats;
    stats.iterations = completed_iterations;
    stats.success = false;
    return false;
  }

  final_equations = covariance_equations;
  stats.valid_residuals = covariance_equations.stats.valid_residuals;
  stats.mean_abs_residual = covariance_equations.stats.mean_abs_residual;
  stats.rms_residual = covariance_equations.stats.rms_residual;
  stats.max_abs_residual = covariance_equations.stats.max_abs_residual;

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

  // Commit state and covariance only after the whole iterated update passes.
  state = iter_state;

  stats.success = true;
  stats.status = "success";
  return true;
}

}  // namespace custom_lio_ekf
