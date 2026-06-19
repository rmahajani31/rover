#include "custom_imu_propagator/imu_propagator.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace custom_imu_propagator
{

namespace
{

constexpr double kPi = 3.14159265358979323846;

double radiansToDegrees(double radians)
{
  return radians * 180.0 / kPi;
}

bool isFiniteQuaternion(const Eigen::Quaterniond& q)
{
  return std::isfinite(q.w()) &&
         std::isfinite(q.x()) &&
         std::isfinite(q.y()) &&
         std::isfinite(q.z());
}

Eigen::Vector3d quaternionToRpy(const Eigen::Quaterniond& q)
{
  const Eigen::Matrix3d r = q.normalized().toRotationMatrix();

  const double roll = std::atan2(r(2, 1), r(2, 2));
  const double pitch = std::asin(std::clamp(-r(2, 0), -1.0, 1.0));
  const double yaw = std::atan2(r(1, 0), r(0, 0));

  return Eigen::Vector3d(roll, pitch, yaw);
}

}  // namespace

ImuPropagator::ImuPropagator(const ImuPropagatorOptions& options)
: options_(options)
{
}

void ImuPropagator::configure(const ImuPropagatorOptions& options)
{
  options_ = options;
}

const ImuPropagatorOptions& ImuPropagator::options() const
{
  return options_;
}

void ImuPropagator::addSample(const ImuSample& sample)
{
  if (!imu_buffer_.empty() && sample.stamp <= imu_buffer_.back().stamp) {
    return;
  }

  imu_buffer_.push_back(sample);
  trimBuffer(sample.stamp);
}

void ImuPropagator::clear()
{
  imu_buffer_.clear();
}

bool ImuPropagator::getSamplesInInterval(
  const rclcpp::Time& start,
  const rclcpp::Time& end,
  std::vector<ImuSample>& samples) const
{
  samples.clear();

  if (end <= start || imu_buffer_.empty()) {
    return false;
  }

  // Phase 8 keeps extraction simple: no boundary interpolation, so the buffer
  // must fully cover the requested LiDAR-to-LiDAR interval.
  if (imu_buffer_.front().stamp > start || imu_buffer_.back().stamp < end) {
    return false;
  }

  for (const auto& sample : imu_buffer_) {
    if (sample.stamp >= start && sample.stamp <= end) {
      samples.push_back(sample);
    }
  }

  return samples.size() >= 2;
}

ImuPropagationResult ImuPropagator::propagateBetween(
  const rclcpp::Time& start,
  const rclcpp::Time& end) const
{
  ImuPropagationResult result;

  if (!options_.use_imu_rotation) {
    result.success = true;
    result.status = "imu_rotation_disabled";
    return result;
  }

  // Translation is deliberately gated off until gravity/bias handling is ready.
  if (options_.use_imu_translation) {
    result.status = "imu_translation_not_implemented";
    return result;
  }

  std::vector<ImuSample> samples;
  if (!getSamplesInInterval(start, end, samples)) {
    result.status = "imu_interval_unavailable";
    return result;
  }

  return propagateRotationOnly(samples);
}

Eigen::Quaterniond ImuPropagator::deltaQuaternion(
  const Eigen::Vector3d& omega,
  double dt)
{
  const Eigen::Vector3d theta = omega * dt;
  const double angle = theta.norm();

  if (angle < 1.0e-8) {
    // First-order quaternion approximation avoids a poorly conditioned axis for
    // near-zero angular increments.
    Eigen::Quaterniond dq(
      1.0,
      0.5 * theta.x(),
      0.5 * theta.y(),
      0.5 * theta.z());
    dq.normalize();
    return dq;
  }

  const Eigen::Vector3d axis = theta / angle;
  Eigen::Quaterniond dq(Eigen::AngleAxisd(angle, axis));
  dq.normalize();
  return dq;
}

void ImuPropagator::trimBuffer(const rclcpp::Time& newest_time)
{
  while (!imu_buffer_.empty()) {
    const double age = (newest_time - imu_buffer_.front().stamp).seconds();

    if (age <= options_.max_imu_buffer_seconds) {
      break;
    }

    imu_buffer_.pop_front();
  }
}

ImuPropagationResult ImuPropagator::propagateRotationOnly(
  const std::vector<ImuSample>& samples) const
{
  ImuPropagationResult result;

  if (samples.size() < 2) {
    result.status = "not_enough_imu_samples";
    return result;
  }

  Eigen::Quaterniond delta_q = Eigen::Quaterniond::Identity();
  double dt_total = 0.0;

  for (std::size_t i = 0; i + 1 < samples.size(); ++i) {
    const double dt = (samples[i + 1].stamp - samples[i].stamp).seconds();

    if (dt <= 0.0) {
      result.status = "non_positive_imu_dt";
      return result;
    }

    if (dt > options_.max_allowed_imu_gap) {
      result.status = "imu_gap_too_large";
      return result;
    }

    const Eigen::Vector3d omega = samples[i].gyro - options_.gyro_bias;
    const Eigen::Quaterniond dq = deltaQuaternion(omega, dt);

    // Right-multiply body-frame increments so the delta maps from the previous
    // LiDAR pose into the current scan's predicted orientation.
    delta_q = delta_q * dq;
    delta_q.normalize();

    if (!isFiniteQuaternion(delta_q)) {
      result.status = "non_finite_delta_quaternion";
      return result;
    }

    dt_total += dt;
  }

  const Eigen::Vector3d rpy = quaternionToRpy(delta_q);
  const double delta_yaw_deg = radiansToDegrees(rpy.z());

  if (std::abs(delta_yaw_deg) > options_.max_expected_yaw_change_deg_per_scan) {
    result.status = "delta_yaw_too_large";
    return result;
  }

  result.success = true;
  result.status = "rotation_only_propagated";
  result.delta_q = delta_q;
  result.delta_T = Eigen::Isometry3d::Identity();
  result.delta_T.linear() = delta_q.toRotationMatrix();
  result.samples_used = samples.size();
  result.dt_total = dt_total;
  result.delta_roll_deg = radiansToDegrees(rpy.x());
  result.delta_pitch_deg = radiansToDegrees(rpy.y());
  result.delta_yaw_deg = delta_yaw_deg;

  return result;
}

}  // namespace custom_imu_propagator
