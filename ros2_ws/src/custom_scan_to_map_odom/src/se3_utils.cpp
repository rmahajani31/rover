#include "custom_scan_to_map_odom/se3_utils.hpp"

#include <cmath>

namespace custom_scan_to_map_odom
{

Eigen::Matrix3d skew(const Eigen::Vector3d& v)
{
  Eigen::Matrix3d S;
  S << 0.0, -v.z(), v.y(),
       v.z(), 0.0, -v.x(),
       -v.y(), v.x(), 0.0;
  return S;
}

Eigen::Matrix3d expSO3(const Eigen::Vector3d& w)
{
  const double theta = w.norm();
  const Eigen::Matrix3d W = skew(w);
  const Eigen::Matrix3d W2 = W * W;

  if (theta < 1.0e-10) {
    return Eigen::Matrix3d::Identity() + W + 0.5 * W2;
  }

  const double theta2 = theta * theta;
  const double A = std::sin(theta) / theta;
  const double B = (1.0 - std::cos(theta)) / theta2;

  return Eigen::Matrix3d::Identity() + A * W + B * W2;
}

Eigen::Isometry3d expSE3(const Vector6d& dx)
{
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();

  const Eigen::Vector3d dtheta = dx.head<3>();
  const Eigen::Vector3d dt = dx.tail<3>();

  T.linear() = expSO3(dtheta);
  T.translation() = dt;

  return T;
}

Eigen::Isometry3d leftUpdateSE3(
  const Vector6d& dx,
  const Eigen::Isometry3d& T)
{
  return expSE3(dx) * T;
}

geometry_msgs::msg::Pose toRosPose(const Eigen::Isometry3d& T)
{
  geometry_msgs::msg::Pose pose;

  pose.position.x = T.translation().x();
  pose.position.y = T.translation().y();
  pose.position.z = T.translation().z();

  Eigen::Quaterniond q(T.rotation());
  q.normalize();

  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();

  return pose;
}

}  // namespace custom_scan_to_map_odom
