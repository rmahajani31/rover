#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <geometry_msgs/msg/pose.hpp>

namespace custom_scan_to_map_odom
{

// Pose increments are ordered as [rx, ry, rz, tx, ty, tz].
using Vector6d = Eigen::Matrix<double, 6, 1>;

Eigen::Matrix3d skew(const Eigen::Vector3d& v);

Eigen::Matrix3d expSO3(const Eigen::Vector3d& w);

Eigen::Isometry3d expSE3(const Vector6d& dx);

// Applies a world-frame perturbation, matching the optimizer Jacobian convention.
Eigen::Isometry3d leftUpdateSE3(
  const Vector6d& dx,
  const Eigen::Isometry3d& T);

geometry_msgs::msg::Pose toRosPose(const Eigen::Isometry3d& T);

}  // namespace custom_scan_to_map_odom
