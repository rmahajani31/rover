#pragma once

#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>

namespace custom_imu_propagator
{

// One timestamped IMU reading after unit normalization.
struct ImuSample
{
  rclcpp::Time stamp;
  // Angular velocity in rad/s, expressed in the IMU/body frame.
  Eigen::Vector3d gyro = Eigen::Vector3d::Zero();
  // Linear acceleration in m/s^2, expressed in the IMU/body frame.
  Eigen::Vector3d accel = Eigen::Vector3d::Zero();
};

}  // namespace custom_imu_propagator
