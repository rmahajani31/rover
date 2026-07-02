#include "custom_lio_ekf/ekf_state.hpp"

#include <gtest/gtest.h>

namespace custom_lio_ekf
{
namespace
{

void expectVectorNear(
  const Eigen::Vector3d& actual,
  const Eigen::Vector3d& expected,
  double tolerance)
{
  EXPECT_NEAR(actual.x(), expected.x(), tolerance);
  EXPECT_NEAR(actual.y(), expected.y(), tolerance);
  EXPECT_NEAR(actual.z(), expected.z(), tolerance);
}

TEST(EkfState, So3LogInvertsSo3ExpForSmallRotation)
{
  const Eigen::Vector3d delta_theta(0.01, -0.02, 0.03);

  expectVectorNear(
    so3Log(so3Exp(delta_theta)),
    delta_theta,
    1.0e-12);
}

TEST(EkfState, So3LogUsesShortestQuaternionRepresentation)
{
  const Eigen::Vector3d delta_theta(0.02, 0.01, -0.03);
  Eigen::Quaterniond q = so3Exp(delta_theta);
  q.coeffs() *= -1.0;

  expectVectorNear(
    so3Log(q),
    delta_theta,
    1.0e-12);
}

TEST(EkfState, ErrorStateDifferenceRecoversInjectedError)
{
  EkfState reference;
  reference.q_WI =
    Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(-0.1, Eigen::Vector3d::UnitY());
  reference.p_I_W = Eigen::Vector3d(1.0, 2.0, 3.0);
  reference.v_I_W = Eigen::Vector3d(0.5, -0.25, 0.1);
  reference.b_g = Eigen::Vector3d(0.01, 0.02, -0.03);
  reference.b_a = Eigen::Vector3d(-0.1, 0.2, 0.3);
  reference.g_W = Eigen::Vector3d(0.0, 0.0, -9.8);

  Vector18d injected = Vector18d::Zero();
  injected.segment<3>(kThetaOffset) = Eigen::Vector3d(0.01, -0.015, 0.02);
  injected.segment<3>(kPositionOffset) = Eigen::Vector3d(0.1, -0.2, 0.3);
  injected.segment<3>(kVelocityOffset) = Eigen::Vector3d(-0.4, 0.5, -0.6);
  injected.segment<3>(kGyroBiasOffset) = Eigen::Vector3d(0.001, -0.002, 0.003);
  injected.segment<3>(kAccelBiasOffset) = Eigen::Vector3d(-0.01, 0.02, -0.03);
  injected.segment<3>(kGravityOffset) = Eigen::Vector3d(0.04, -0.05, 0.06);

  EkfState state = reference;
  injectError(state, injected);

  const Vector18d difference = errorStateDifference(state, reference);

  for (int i = 0; i < kErrorStateDim; ++i) {
    EXPECT_NEAR(difference(i), injected(i), 1.0e-12);
  }
}

}  // namespace
}  // namespace custom_lio_ekf
