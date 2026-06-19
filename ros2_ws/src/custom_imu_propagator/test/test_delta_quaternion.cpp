#include <cmath>

#include <gtest/gtest.h>

#include "custom_imu_propagator/imu_propagator.hpp"

namespace custom_imu_propagator
{
namespace
{

double yawFromQuaternion(const Eigen::Quaterniond& q)
{
  const Eigen::Matrix3d rotation = q.normalized().toRotationMatrix();
  return std::atan2(rotation(1, 0), rotation(0, 0));
}

}  // namespace

TEST(DeltaQuaternionTest, IntegratesYawRateOverOneSecond)
{
  const Eigen::Quaterniond q =
    ImuPropagator::deltaQuaternion(Eigen::Vector3d(0.0, 0.0, 1.0), 1.0);

  EXPECT_NEAR(1.0, q.norm(), 1.0e-12);
  EXPECT_NEAR(1.0, yawFromQuaternion(q), 1.0e-12);
}

TEST(DeltaQuaternionTest, IntegratesNegativeYawRate)
{
  const Eigen::Quaterniond q =
    ImuPropagator::deltaQuaternion(Eigen::Vector3d(0.0, 0.0, -0.5), 2.0);

  EXPECT_NEAR(1.0, q.norm(), 1.0e-12);
  EXPECT_NEAR(-1.0, yawFromQuaternion(q), 1.0e-12);
}

TEST(DeltaQuaternionTest, HandlesTinyRotation)
{
  const Eigen::Vector3d omega(1.0e-10, -2.0e-10, 3.0e-10);
  const Eigen::Quaterniond q = ImuPropagator::deltaQuaternion(omega, 0.01);

  EXPECT_NEAR(1.0, q.norm(), 1.0e-12);
  EXPECT_TRUE(std::isfinite(q.w()));
  EXPECT_TRUE(std::isfinite(q.x()));
  EXPECT_TRUE(std::isfinite(q.y()));
  EXPECT_TRUE(std::isfinite(q.z()));
}

}  // namespace custom_imu_propagator
