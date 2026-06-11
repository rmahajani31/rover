#pragma once

#include <Eigen/Dense>

#include <geometry_msgs/msg/quaternion.hpp>

namespace custom_icp_odom
{

geometry_msgs::msg::Quaternion rotationMatrixToQuaternion(
  const Eigen::Matrix3d & rotation);

double yawFromRotationMatrix(const Eigen::Matrix3d & rotation);

}  // namespace custom_icp_odom
