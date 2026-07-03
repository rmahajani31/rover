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

Eigen::Vector3d so3Log(const Eigen::Quaterniond& q)
{
  Eigen::Quaterniond normalized = q.normalized();

  if (normalized.w() < 0.0) {
    normalized.coeffs() *= -1.0;
  }

  const Eigen::Vector3d vector_part(
    normalized.x(),
    normalized.y(),
    normalized.z());
  const double vector_norm = vector_part.norm();

  if (vector_norm < 1.0e-12) {
    return 2.0 * vector_part;
  }

  const double angle = 2.0 * std::atan2(vector_norm, normalized.w());
  return angle * vector_part / vector_norm;
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

Vector18d errorStateDifference(
  const EkfState& state,
  const EkfState& reference)
{
  Vector18d eta = Vector18d::Zero();

  Eigen::Quaterniond delta_q =
    reference.q_WI.normalized().conjugate() * state.q_WI.normalized();
  delta_q.normalize();

  if (delta_q.w() < 0.0) {
    delta_q.coeffs() *= -1.0;
  }

  eta.segment<3>(kThetaOffset) = so3Log(delta_q);
  eta.segment<3>(kPositionOffset) = state.p_I_W - reference.p_I_W;
  eta.segment<3>(kVelocityOffset) = state.v_I_W - reference.v_I_W;
  eta.segment<3>(kGyroBiasOffset) = state.b_g - reference.b_g;
  eta.segment<3>(kAccelBiasOffset) = state.b_a - reference.b_a;
  eta.segment<3>(kGravityOffset) = state.g_W - reference.g_W;

  return eta;
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
