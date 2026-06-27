#include "custom_lio_ekf/ekf_state.hpp"

#include <cmath>

namespace custom_lio_ekf
{

namespace
{

bool vectorIsFinite(const Eigen::Vector3d& v)
{
  return v.allFinite();
}

bool quaternionIsFinite(const Eigen::Quaterniond& q)
{
  return std::isfinite(q.w()) &&
         std::isfinite(q.x()) &&
         std::isfinite(q.y()) &&
         std::isfinite(q.z());
}

}  // namespace

Vector18d zeroErrorState()
{
  return Vector18d::Zero();
}

Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d& v)
{
  Eigen::Matrix3d skew;
  skew << 0.0, -v.z(), v.y(),
          v.z(), 0.0, -v.x(),
          -v.y(), v.x(), 0.0;
  return skew;
}

Eigen::Quaterniond so3Exp(const Eigen::Vector3d& delta_theta)
{
  const double theta = delta_theta.norm();

  if (theta < 1.0e-12) {
    // First-order SO(3) exponential for tiny corrections.
    Eigen::Quaterniond q(
      1.0,
      0.5 * delta_theta.x(),
      0.5 * delta_theta.y(),
      0.5 * delta_theta.z());
    q.normalize();
    return q;
  }

  const Eigen::Vector3d axis = delta_theta / theta;
  return Eigen::Quaterniond(Eigen::AngleAxisd(theta, axis));
}

Eigen::Isometry3d imuPoseInWorld(const EkfState& state)
{
  Eigen::Isometry3d T_WI = Eigen::Isometry3d::Identity();
  T_WI.linear() = state.q_WI.normalized().toRotationMatrix();
  T_WI.translation() = state.p_I_W;
  return T_WI;
}

void normalizeRotation(EkfState& state)
{
  state.q_WI.normalize();
}

void injectError(EkfState& state, const Vector18d& delta_x)
{
  const Eigen::Vector3d delta_theta =
    delta_x.segment<3>(kThetaOffset);

  // Right-multiply the small attitude error: R_WI <- R_WI * Exp(delta_theta).
  state.q_WI = state.q_WI * so3Exp(delta_theta);
  state.q_WI.normalize();

  state.p_I_W += delta_x.segment<3>(kPositionOffset);
  state.v_I_W += delta_x.segment<3>(kVelocityOffset);
  state.b_g += delta_x.segment<3>(kGyroBiasOffset);
  state.b_a += delta_x.segment<3>(kAccelBiasOffset);
  state.g_W += delta_x.segment<3>(kGravityOffset);
}

void symmetrizeCovariance(EkfState& state)
{
  // Numeric propagation/solves can introduce tiny asymmetric roundoff.
  state.P = 0.5 * (state.P + state.P.transpose());
}

bool isFinite(const EkfState& state)
{
  return quaternionIsFinite(state.q_WI) &&
         vectorIsFinite(state.p_I_W) &&
         vectorIsFinite(state.v_I_W) &&
         vectorIsFinite(state.b_g) &&
         vectorIsFinite(state.b_a) &&
         vectorIsFinite(state.g_W) &&
         state.P.allFinite();
}

}  // namespace custom_lio_ekf
