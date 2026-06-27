#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "custom_imu_propagator/imu_sample.hpp"
#include "custom_lio_ekf/ekf_state.hpp"

namespace custom_lio_ekf
{

struct ImuPredictionStats
{
  bool success = false;
  std::string status = "not_started";

  std::size_t intervals_integrated = 0;
  double dt_total = 0.0;
};

Eigen::Vector3d biasCorrectedGyro(
  const custom_imu_propagator::ImuSample& sample,
  const EkfState& state);

Eigen::Vector3d biasCorrectedAccel(
  const custom_imu_propagator::ImuSample& sample,
  const EkfState& state);

Eigen::Vector3d worldAcceleration(
  const EkfState& state,
  const Eigen::Vector3d& accel_unbiased);

void propagateNominalState(
  EkfState& state,
  const Eigen::Vector3d& gyro_unbiased,
  const Eigen::Vector3d& accel_unbiased,
  double dt,
  bool use_accel_translation = false);

bool predictNominalState(
  EkfState& state,
  const std::vector<custom_imu_propagator::ImuSample>& samples,
  ImuPredictionStats& stats,
  bool use_accel_translation = false);

}  // namespace custom_lio_ekf
