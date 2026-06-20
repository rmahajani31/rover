#pragma once

#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "custom_lidar_deskew/deskew_types.hpp"

namespace custom_lidar_deskew
{

Eigen::Quaterniond expQuaternion(const Eigen::Vector3d& theta);

// imu_samples must be sorted by increasing timestamp and should cover the scan interval.
std::vector<RotationSample> integrateRotation(
  const std::vector<ImuSample>& imu_samples,
  double scan_start,
  double scan_end,
  const Eigen::Vector3d& gyro_bias);

// Returns the scan-relative rotation at time t using spherical interpolation.
Eigen::Quaterniond interpolateRotation(
  const std::vector<RotationSample>& samples,
  double t);

double maxAngularVelocity(
  const std::vector<ImuSample>& imu_samples,
  const Eigen::Vector3d& gyro_bias);

}  // namespace custom_lidar_deskew
