#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "custom_imu_propagator/imu_sample.hpp"
#include "custom_lio_ekf/ekf_parameters.hpp"
#include "custom_lio_ekf/ekf_state.hpp"
#include "custom_lio_ekf/imu_prediction.hpp"

namespace custom_lio_ekf
{

using Matrix18x12d = Eigen::Matrix<double, kErrorStateDim, kImuNoiseDim>;

struct EkfPredictionStats
{
  bool success = false;
  std::string status = "not_started";

  std::size_t intervals_integrated = 0;
  double dt_total = 0.0;
};

Matrix18d buildContinuousErrorDynamics(
  const EkfState& state,
  const Eigen::Vector3d& gyro_unbiased,
  const Eigen::Vector3d& accel_unbiased,
  bool use_accel_translation = false);

Matrix18x12d buildContinuousNoiseInputMatrix(
  const EkfState& state,
  bool use_accel_translation = false);

Matrix18d discretizeErrorDynamics(
  const Matrix18d& F_c,
  double dt);

Matrix18d discretizeProcessNoise(
  const Matrix18x12d& G_c,
  const Matrix12d& Q_c,
  double dt);

void propagateCovariance(
  EkfState& state,
  const ImuNoiseStdDevs& imu_noise,
  const Eigen::Vector3d& gyro_unbiased,
  const Eigen::Vector3d& accel_unbiased,
  double dt,
  bool use_accel_translation = false);

void propagateStateAndCovariance(
  EkfState& state,
  const ImuNoiseStdDevs& imu_noise,
  const custom_imu_propagator::ImuSample& sample,
  double dt,
  bool use_accel_translation = false);

bool predictStateAndCovariance(
  EkfState& state,
  const EkfParameters& parameters,
  const std::vector<custom_imu_propagator::ImuSample>& samples,
  EkfPredictionStats& stats);

}  // namespace custom_lio_ekf
