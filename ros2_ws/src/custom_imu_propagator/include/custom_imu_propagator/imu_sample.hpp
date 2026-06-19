#pragma once

#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>

namespace custom_imu_propagator
{

struct ImuSample
{
  rclcpp::Time stamp;
  Eigen::Vector3d gyro = Eigen::Vector3d::Zero();
  Eigen::Vector3d accel = Eigen::Vector3d::Zero();
};

}  // namespace custom_imu_propagator
