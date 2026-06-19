#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>

namespace custom_imu_propagator
{

struct ImuState
{
  Eigen::Quaterniond q_WI = Eigen::Quaterniond::Identity();
  Eigen::Vector3d p_WI = Eigen::Vector3d::Zero();
  Eigen::Vector3d v_WI = Eigen::Vector3d::Zero();

  Eigen::Vector3d b_g = Eigen::Vector3d::Zero();
  Eigen::Vector3d b_a = Eigen::Vector3d::Zero();

  Eigen::Vector3d g_W = Eigen::Vector3d(0.0, 0.0, -9.81);

  rclcpp::Time stamp;
};

}  // namespace custom_imu_propagator
