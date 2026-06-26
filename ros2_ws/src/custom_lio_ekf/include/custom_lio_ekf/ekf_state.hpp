#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace custom_lio_ekf
{

constexpr int kErrorStateDim = 18;

constexpr int kThetaOffset = 0;
constexpr int kPositionOffset = 3;
constexpr int kVelocityOffset = 6;
constexpr int kGyroBiasOffset = 9;
constexpr int kAccelBiasOffset = 12;
constexpr int kGravityOffset = 15;

using Vector18d = Eigen::Matrix<double, kErrorStateDim, 1>;
using Matrix18d = Eigen::Matrix<double, kErrorStateDim, kErrorStateDim>;

// Nominal EKF state from the Phase 10 guide:
//
// x = (R_WI, p_I_W, v_I_W, b_g, b_a, g_W)
//
// The covariance P is over the 18D error state:
//
// delta_x = [
//   delta_theta,
//   delta_p,
//   delta_v,
//   delta_b_g,
//   delta_b_a,
//   delta_g
// ]
struct EkfState
{
  Eigen::Quaterniond q_WI = Eigen::Quaterniond::Identity();
  Eigen::Vector3d p_I_W = Eigen::Vector3d::Zero();
  Eigen::Vector3d v_I_W = Eigen::Vector3d::Zero();

  Eigen::Vector3d b_g = Eigen::Vector3d::Zero();
  Eigen::Vector3d b_a = Eigen::Vector3d::Zero();
  Eigen::Vector3d g_W = Eigen::Vector3d(0.0, 0.0, -9.81);

  Matrix18d P = Matrix18d::Zero();
};

Vector18d zeroErrorState();

Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d& v);

Eigen::Quaterniond so3Exp(const Eigen::Vector3d& delta_theta);

Eigen::Isometry3d imuPoseInWorld(const EkfState& state);

void normalizeRotation(EkfState& state);

void injectError(EkfState& state, const Vector18d& delta_x);

void symmetrizeCovariance(EkfState& state);

bool isFinite(const EkfState& state);

}  // namespace custom_lio_ekf
