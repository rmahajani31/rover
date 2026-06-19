#pragma once

#include <cstddef>
#include <deque>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>

#include "custom_imu_propagator/imu_sample.hpp"

namespace custom_imu_propagator
{

// Runtime limits and feature gates for lightweight scan-to-scan IMU deltas.
struct ImuPropagatorOptions
{
  double max_imu_buffer_seconds = 5.0;
  double max_allowed_imu_gap = 0.02;
  double max_expected_yaw_change_deg_per_scan = 30.0;

  bool use_imu_rotation = true;
  bool use_imu_translation = false;

  Eigen::Vector3d gyro_bias = Eigen::Vector3d::Zero();
  Eigen::Vector3d accel_bias = Eigen::Vector3d::Zero();
  Eigen::Vector3d gravity = Eigen::Vector3d(0.0, 0.0, -9.81);
};

// Result of integrating buffered IMU samples over one LiDAR scan interval.
struct ImuPropagationResult
{
  bool success = false;
  // Machine-readable status used by logs and diagnostics.
  std::string status;

  // Phase 8 fills delta_q and delta_T rotation. delta_p is reserved for the
  // optional translation path.
  Eigen::Quaterniond delta_q = Eigen::Quaterniond::Identity();
  Eigen::Vector3d delta_p = Eigen::Vector3d::Zero();
  Eigen::Isometry3d delta_T = Eigen::Isometry3d::Identity();

  std::size_t samples_used = 0;
  double dt_total = 0.0;
  double delta_roll_deg = 0.0;
  double delta_pitch_deg = 0.0;
  double delta_yaw_deg = 0.0;
};

class ImuPropagator
{
public:
  explicit ImuPropagator(const ImuPropagatorOptions& options = ImuPropagatorOptions());

  void configure(const ImuPropagatorOptions& options);
  const ImuPropagatorOptions& options() const;

  void addSample(const ImuSample& sample);
  void clear();

  // Returns samples covering [start, end]. The buffer must already contain data
  // at or before start and at or after end; otherwise callers should fall back.
  bool getSamplesInInterval(
    const rclcpp::Time& start,
    const rclcpp::Time& end,
    std::vector<ImuSample>& samples) const;

  // Integrates IMU motion between consecutive accepted LiDAR scan stamps.
  ImuPropagationResult propagateBetween(
    const rclcpp::Time& start,
    const rclcpp::Time& end) const;

  // SO(3) exponential map for a constant angular velocity over dt.
  static Eigen::Quaterniond deltaQuaternion(
    const Eigen::Vector3d& omega,
    double dt);

private:
  void trimBuffer(const rclcpp::Time& newest_time);

  ImuPropagationResult propagateRotationOnly(
    const std::vector<ImuSample>& samples) const;

  ImuPropagatorOptions options_;
  std::deque<ImuSample> imu_buffer_;
};

}  // namespace custom_imu_propagator
