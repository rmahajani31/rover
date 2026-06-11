#include "custom_icp_odom/icp_utils.hpp"

#include <cmath>

namespace custom_icp_odom
{

geometry_msgs::msg::Quaternion rotationMatrixToQuaternion(
  const Eigen::Matrix3d & rotation)
{
  Eigen::Quaterniond quaternion(rotation);
  quaternion.normalize();

  geometry_msgs::msg::Quaternion msg;
  msg.x = quaternion.x();
  msg.y = quaternion.y();
  msg.z = quaternion.z();
  msg.w = quaternion.w();

  return msg;
}

double yawFromRotationMatrix(const Eigen::Matrix3d & rotation)
{
  return std::atan2(rotation(1, 0), rotation(0, 0));
}

}  // namespace custom_icp_odom
