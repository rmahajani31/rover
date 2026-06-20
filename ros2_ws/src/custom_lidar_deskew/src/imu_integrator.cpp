#include "custom_lidar_deskew/imu_integrator.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace custom_lidar_deskew
{
namespace
{

bool isFiniteTime(double t)
{
  return std::isfinite(t);
}

double clamp(double value, double lower, double upper)
{
  return std::max(lower, std::min(value, upper));
}

}  // namespace

Eigen::Quaterniond expQuaternion(const Eigen::Vector3d& theta)
{
  const double angle = theta.norm();

  if (angle < 1e-8) {
    // Small-angle approximation keeps the quaternion well behaved near zero rotation.
    return Eigen::Quaterniond(
      1.0,
      0.5 * theta.x(),
      0.5 * theta.y(),
      0.5 * theta.z()).normalized();
  }

  const Eigen::Vector3d axis = theta / angle;
  return Eigen::Quaterniond(Eigen::AngleAxisd(angle, axis)).normalized();
}

std::vector<RotationSample> integrateRotation(
  const std::vector<ImuSample>& imu_samples,
  double scan_start,
  double scan_end,
  const Eigen::Vector3d& gyro_bias)
{
  std::vector<RotationSample> rotations;

  if (!isFiniteTime(scan_start) || !isFiniteTime(scan_end) || scan_end < scan_start) {
    return rotations;
  }

  Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
  rotations.push_back({scan_start, q});

  if (scan_end == scan_start) {
    return rotations;
  }

  for (std::size_t i = 0; i + 1 < imu_samples.size(); ++i) {
    const double t0 = imu_samples[i].t;
    const double t1 = imu_samples[i + 1].t;

    if (!isFiniteTime(t0) || !isFiniteTime(t1) || t1 <= t0) {
      continue;
    }

    const double seg_start = std::max(t0, scan_start);
    const double seg_end = std::min(t1, scan_end);

    if (seg_end <= seg_start) {
      continue;
    }

    const double dt = seg_end - seg_start;
    const Eigen::Vector3d omega = imu_samples[i].gyro - gyro_bias;

    if (!omega.allFinite()) {
      continue;
    }

    // Integrate the gyro as a piecewise-constant angular velocity over this segment.
    const Eigen::Quaterniond dq = expQuaternion(omega * dt);
    q = q * dq;
    q.normalize();

    rotations.push_back({seg_end, q});

    if (seg_end >= scan_end) {
      break;
    }
  }

  if (rotations.back().t < scan_end) {
    rotations.push_back({scan_end, q});
  }

  return rotations;
}

Eigen::Quaterniond interpolateRotation(
  const std::vector<RotationSample>& samples,
  double t)
{
  if (samples.empty()) {
    return Eigen::Quaterniond::Identity();
  }

  if (!isFiniteTime(t)) {
    return samples.front().q.normalized();
  }

  if (t <= samples.front().t) {
    return samples.front().q.normalized();
  }

  if (t >= samples.back().t) {
    return samples.back().q.normalized();
  }

  const auto upper = std::lower_bound(
    samples.begin(),
    samples.end(),
    t,
    [](const RotationSample& sample, double value) {
      return sample.t < value;
    });

  if (upper == samples.begin()) {
    return upper->q.normalized();
  }

  if (upper == samples.end()) {
    return samples.back().q.normalized();
  }

  const auto lower = upper - 1;
  const double interval = upper->t - lower->t;

  if (interval <= 0.0 || !std::isfinite(interval)) {
    return lower->q.normalized();
  }

  const double alpha = clamp((t - lower->t) / interval, 0.0, 1.0);
  // Slerp preserves constant angular-rate interpolation between adjacent IMU samples.
  return lower->q.slerp(alpha, upper->q).normalized();
}

double maxAngularVelocity(
  const std::vector<ImuSample>& imu_samples,
  const Eigen::Vector3d& gyro_bias)
{
  double max_norm = 0.0;

  for (const auto& sample : imu_samples) {
    const Eigen::Vector3d omega = sample.gyro - gyro_bias;
    if (!omega.allFinite()) {
      continue;
    }

    max_norm = std::max(max_norm, omega.norm());
  }

  return max_norm;
}

}  // namespace custom_lidar_deskew
