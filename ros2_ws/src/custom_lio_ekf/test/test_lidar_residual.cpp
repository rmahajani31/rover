#include "custom_lio_ekf/lidar_residual.hpp"

#include <gtest/gtest.h>

namespace custom_lio_ekf
{
namespace
{

TEST(LidarResidual, PointToPlaneJacobianMatchesRightPerturbationFiniteDifference)
{
  EkfState state;
  state.q_WI =
    Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(-0.2, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitX());
  state.p_I_W = Eigen::Vector3d(0.4, -0.2, 0.7);

  const Eigen::Vector3d point_I(1.2, -0.3, 0.5);
  const Eigen::Vector3d normal_W =
    Eigen::Vector3d(0.2, -0.4, 0.9).normalized();
  const Eigen::Vector3d plane_centroid_W(0.1, 0.2, -0.1);

  const Matrix1x18d H =
    pointToPlaneJacobian(state, point_I, normal_W);

  const double eps = 1.0e-7;

  for (int axis = 0; axis < 3; ++axis) {
    Vector18d delta = Vector18d::Zero();
    delta(kThetaOffset + axis) = eps;

    EkfState plus = state;
    injectError(plus, delta);

    delta(kThetaOffset + axis) = -eps;
    EkfState minus = state;
    injectError(minus, delta);

    const double residual_plus = pointToPlaneResidual(
      transformImuPointToWorld(plus, point_I),
      plane_centroid_W,
      normal_W);
    const double residual_minus = pointToPlaneResidual(
      transformImuPointToWorld(minus, point_I),
      plane_centroid_W,
      normal_W);

    const double numerical_derivative =
      (residual_plus - residual_minus) / (2.0 * eps);

    EXPECT_NEAR(
      H(0, kThetaOffset + axis),
      numerical_derivative,
      1.0e-6);
  }

  EXPECT_NEAR(H(0, kPositionOffset + 0), normal_W.x(), 1.0e-12);
  EXPECT_NEAR(H(0, kPositionOffset + 1), normal_W.y(), 1.0e-12);
  EXPECT_NEAR(H(0, kPositionOffset + 2), normal_W.z(), 1.0e-12);
}

}  // namespace
}  // namespace custom_lio_ekf
