#include "custom_lio_ekf/ekf_parameters.hpp"

#include <cmath>

namespace custom_lio_ekf
{

namespace
{

double square(double value)
{
  return value * value;
}

bool isPositiveFinite(double value)
{
  return std::isfinite(value) && value > 0.0;
}

void setDiagonalBlock(
  Matrix18d& matrix,
  int offset,
  double variance)
{
  matrix.block<3, 3>(offset, offset) =
    variance * Eigen::Matrix3d::Identity();
}

}  // namespace

Matrix18d makeInitialCovariance(const InitialCovarianceStdDevs& stddevs)
{
  Matrix18d P = Matrix18d::Zero();

  // Parameters are standard deviations; the EKF stores variances on P.
  setDiagonalBlock(P, kThetaOffset, square(stddevs.theta));
  setDiagonalBlock(P, kPositionOffset, square(stddevs.position));
  setDiagonalBlock(P, kVelocityOffset, square(stddevs.velocity));
  setDiagonalBlock(P, kGyroBiasOffset, square(stddevs.gyro_bias));
  setDiagonalBlock(P, kAccelBiasOffset, square(stddevs.accel_bias));
  setDiagonalBlock(P, kGravityOffset, square(stddevs.gravity));

  return P;
}

Matrix12d makeContinuousImuNoiseCovariance(const ImuNoiseStdDevs& stddevs)
{
  Matrix12d Q_c = Matrix12d::Zero();

  // Continuous noise order: gyro, accel, gyro-bias random walk, accel-bias random walk.
  Q_c.block<3, 3>(0, 0) =
    square(stddevs.gyro_noise) * Eigen::Matrix3d::Identity();

  Q_c.block<3, 3>(3, 3) =
    square(stddevs.accel_noise) * Eigen::Matrix3d::Identity();

  Q_c.block<3, 3>(6, 6) =
    square(stddevs.gyro_bias_random_walk) * Eigen::Matrix3d::Identity();

  Q_c.block<3, 3>(9, 9) =
    square(stddevs.accel_bias_random_walk) * Eigen::Matrix3d::Identity();

  return Q_c;
}

double lidarResidualVariance(const LidarUpdateOptions& options)
{
  return square(options.lidar_residual_stddev);
}

double robustResidualWeight(
  double residual,
  const LidarUpdateOptions& options)
{
  if (!options.use_huber_weight) {
    return 1.0;
  }

  // Huber weighting keeps large point-to-plane residuals from dominating.
  const double abs_residual = std::abs(residual);

  if (abs_residual <= options.huber_delta) {
    return 1.0;
  }

  return options.huber_delta / abs_residual;
}

bool hasValidNoiseParameters(const EkfParameters& parameters)
{
  const auto& initial = parameters.initial_covariance;
  const auto& imu = parameters.imu_noise;
  const auto& lidar = parameters.lidar_update;

  return isPositiveFinite(initial.theta) &&
         isPositiveFinite(initial.position) &&
         isPositiveFinite(initial.velocity) &&
         isPositiveFinite(initial.gyro_bias) &&
         isPositiveFinite(initial.accel_bias) &&
         isPositiveFinite(initial.gravity) &&
         isPositiveFinite(imu.gyro_noise) &&
         isPositiveFinite(imu.accel_noise) &&
         isPositiveFinite(imu.gyro_bias_random_walk) &&
         isPositiveFinite(imu.accel_bias_random_walk) &&
         isPositiveFinite(lidar.lidar_residual_stddev) &&
         isPositiveFinite(lidar.max_neighbor_distance) &&
         isPositiveFinite(lidar.max_plane_error) &&
         isPositiveFinite(lidar.max_point_to_plane_residual) &&
         isPositiveFinite(lidar.min_plane_eigen_ratio) &&
         isPositiveFinite(lidar.convergence_theta_norm) &&
         isPositiveFinite(lidar.convergence_position_norm) &&
         isPositiveFinite(lidar.max_correction_theta_norm) &&
         isPositiveFinite(lidar.max_correction_position_norm) &&
         isPositiveFinite(lidar.huber_delta) &&
         lidar.max_iterations > 0 &&
         lidar.k_neighbors >= 3 &&
         lidar.min_valid_residuals > 0;
}

}  // namespace custom_lio_ekf
