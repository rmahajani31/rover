#pragma once

#include <cstddef>

#include <Eigen/Core>

#include "custom_lio_ekf/ekf_state.hpp"

namespace custom_lio_ekf
{

constexpr int kImuNoiseDim = 12;

using Matrix12d = Eigen::Matrix<double, kImuNoiseDim, kImuNoiseDim>;

struct InitialCovarianceStdDevs
{
  double theta = 0.05;
  double position = 0.10;
  double velocity = 0.10;
  double gyro_bias = 0.01;
  double accel_bias = 0.10;
  double gravity = 0.10;
};

struct ImuNoiseStdDevs
{
  double gyro_noise = 0.015;
  double accel_noise = 0.20;
  double gyro_bias_random_walk = 0.0001;
  double accel_bias_random_walk = 0.001;
};

struct LidarUpdateOptions
{
  int max_iterations = 5;
  int k_neighbors = 5;

  std::size_t min_valid_residuals = 100;

  double lidar_residual_stddev = 0.05;

  double max_neighbor_distance = 1.0;
  double max_plane_error = 0.10;
  double max_point_to_plane_residual = 0.50;
  double min_plane_eigen_ratio = 5.0;

  double convergence_theta_norm = 0.001;
  double convergence_position_norm = 0.001;

  double max_correction_theta_norm = 0.10;
  double max_correction_position_norm = 0.30;

  bool use_huber_weight = true;
  double huber_delta = 0.10;
};

struct EkfParameters
{
  InitialCovarianceStdDevs initial_covariance;
  ImuNoiseStdDevs imu_noise;
  LidarUpdateOptions lidar_update;

  // Early rover tests use rotation-only IMU prediction. Full accelerometer
  // translation prediction needs calibrated accel bias, gravity alignment, and
  // verified IMU units; otherwise a stationary rover can accumulate fake motion.
  bool use_accel_translation_prediction = false;
};

Matrix18d makeInitialCovariance(const InitialCovarianceStdDevs& stddevs);

Matrix12d makeContinuousImuNoiseCovariance(const ImuNoiseStdDevs& stddevs);

double lidarResidualVariance(const LidarUpdateOptions& options);

double robustResidualWeight(
  double residual,
  const LidarUpdateOptions& options);

bool hasValidNoiseParameters(const EkfParameters& parameters);

}  // namespace custom_lio_ekf
