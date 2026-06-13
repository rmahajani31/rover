#include "custom_scan_to_map_odom/se3_utils.hpp"

#include <cmath>

#include <gtest/gtest.h>

namespace custom_scan_to_map_odom
{
namespace
{

constexpr double kTolerance = 1.0e-9;

TEST(Se3UtilsTest, SkewMatchesCrossProduct)
{
  const Eigen::Vector3d a(1.0, -2.0, 3.0);
  const Eigen::Vector3d b(4.0, 5.0, -6.0);

  const Eigen::Vector3d matrix_cross = skew(a) * b;
  const Eigen::Vector3d direct_cross = a.cross(b);

  EXPECT_NEAR((matrix_cross - direct_cross).norm(), 0.0, kTolerance);
}

TEST(Se3UtilsTest, ExpSo3ZeroIsIdentity)
{
  const Eigen::Matrix3d R = expSO3(Eigen::Vector3d::Zero());

  EXPECT_NEAR((R - Eigen::Matrix3d::Identity()).norm(), 0.0, kTolerance);
}

TEST(Se3UtilsTest, ExpSo3YawRotatesXAxisToYAxis)
{
  constexpr double kHalfPi = 1.57079632679489661923;
  const Eigen::Matrix3d R = expSO3(Eigen::Vector3d(0.0, 0.0, kHalfPi));
  const Eigen::Vector3d rotated = R * Eigen::Vector3d::UnitX();

  EXPECT_NEAR(rotated.x(), 0.0, 1.0e-9);
  EXPECT_NEAR(rotated.y(), 1.0, 1.0e-9);
  EXPECT_NEAR(rotated.z(), 0.0, 1.0e-9);
}

TEST(Se3UtilsTest, ExpSe3UsesRotationAndTranslationParts)
{
  Vector6d dx = Vector6d::Zero();
  dx.tail<3>() = Eigen::Vector3d(1.0, 2.0, 3.0);

  const Eigen::Isometry3d T = expSE3(dx);

  EXPECT_NEAR((T.rotation() - Eigen::Matrix3d::Identity()).norm(), 0.0, kTolerance);
  EXPECT_NEAR((T.translation() - Eigen::Vector3d(1.0, 2.0, 3.0)).norm(), 0.0, kTolerance);
}

TEST(Se3UtilsTest, LeftUpdatePremultipliesPose)
{
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.translation() = Eigen::Vector3d(1.0, 0.0, 0.0);

  Vector6d dx = Vector6d::Zero();
  dx.tail<3>() = Eigen::Vector3d(0.5, 0.0, 0.0);

  const Eigen::Isometry3d updated = leftUpdateSE3(dx, T);

  EXPECT_NEAR(updated.translation().x(), 1.5, kTolerance);
  EXPECT_NEAR(updated.translation().y(), 0.0, kTolerance);
  EXPECT_NEAR(updated.translation().z(), 0.0, kTolerance);
}

TEST(Se3UtilsTest, ToRosPoseCopiesTransform)
{
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.translation() = Eigen::Vector3d(1.0, 2.0, 3.0);

  const auto pose = toRosPose(T);

  EXPECT_DOUBLE_EQ(pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(pose.position.y, 2.0);
  EXPECT_DOUBLE_EQ(pose.position.z, 3.0);
  EXPECT_NEAR(pose.orientation.x, 0.0, kTolerance);
  EXPECT_NEAR(pose.orientation.y, 0.0, kTolerance);
  EXPECT_NEAR(pose.orientation.z, 0.0, kTolerance);
  EXPECT_NEAR(pose.orientation.w, 1.0, kTolerance);
}

}  // namespace
}  // namespace custom_scan_to_map_odom
