#include <cmath>
#include <cstdint>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "custom_imu_propagator/imu_propagator.hpp"
#include "custom_imu_propagator/imu_sample.hpp"

namespace custom_imu_propagator
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

rclcpp::Time timeFromSeconds(double seconds)
{
  return rclcpp::Time(
    static_cast<std::int64_t>(std::llround(seconds * 1.0e9)),
    RCL_ROS_TIME);
}

double yawFromTransform(const Eigen::Isometry3d& transform)
{
  return std::atan2(transform.linear()(1, 0), transform.linear()(0, 0));
}

ImuSample makeSample(double stamp_sec, const Eigen::Vector3d& gyro)
{
  ImuSample sample;
  sample.stamp = timeFromSeconds(stamp_sec);
  sample.gyro = gyro;
  return sample;
}

}  // namespace

TEST(RotationIntegrationTest, IntegratesKnownYawRateAcrossScanInterval)
{
  ImuPropagatorOptions options;
  options.max_allowed_imu_gap = 0.02;
  options.max_expected_yaw_change_deg_per_scan = 30.0;

  ImuPropagator propagator(options);

  for (int i = 0; i <= 20; ++i) {
    propagator.addSample(makeSample(
      static_cast<double>(i) * 0.005,
      Eigen::Vector3d(0.0, 0.0, 0.5)));
  }

  const ImuPropagationResult result = propagator.propagateBetween(
    timeFromSeconds(0.0),
    timeFromSeconds(0.1));

  ASSERT_TRUE(result.success) << result.status;
  EXPECT_EQ("rotation_only_propagated", result.status);
  EXPECT_EQ(21U, result.samples_used);
  EXPECT_NEAR(0.1, result.dt_total, 1.0e-12);
  EXPECT_NEAR(0.05, yawFromTransform(result.delta_T), 1.0e-12);
  EXPECT_NEAR(0.05 * 180.0 / kPi, result.delta_yaw_deg, 1.0e-9);
  EXPECT_NEAR(0.0, result.delta_T.translation().norm(), 1.0e-12);
}

TEST(RotationIntegrationTest, AppliesGyroBias)
{
  ImuPropagatorOptions options;
  options.gyro_bias = Eigen::Vector3d(0.0, 0.0, 0.1);
  options.max_allowed_imu_gap = 0.02;

  ImuPropagator propagator(options);

  for (int i = 0; i <= 10; ++i) {
    propagator.addSample(makeSample(
      static_cast<double>(i) * 0.01,
      Eigen::Vector3d(0.0, 0.0, 0.5)));
  }

  const ImuPropagationResult result = propagator.propagateBetween(
    timeFromSeconds(0.0),
    timeFromSeconds(0.1));

  ASSERT_TRUE(result.success) << result.status;
  EXPECT_NEAR(0.04, yawFromTransform(result.delta_T), 1.0e-12);
}

TEST(RotationIntegrationTest, RejectsLargeImuGap)
{
  ImuPropagatorOptions options;
  options.max_allowed_imu_gap = 0.02;

  ImuPropagator propagator(options);
  propagator.addSample(makeSample(0.00, Eigen::Vector3d(0.0, 0.0, 0.5)));
  propagator.addSample(makeSample(0.03, Eigen::Vector3d(0.0, 0.0, 0.5)));

  const ImuPropagationResult result = propagator.propagateBetween(
    timeFromSeconds(0.0),
    timeFromSeconds(0.03));

  EXPECT_FALSE(result.success);
  EXPECT_EQ("imu_gap_too_large", result.status);
}

TEST(RotationIntegrationTest, RejectsExcessiveYawJump)
{
  ImuPropagatorOptions options;
  options.max_allowed_imu_gap = 0.02;
  options.max_expected_yaw_change_deg_per_scan = 5.0;

  ImuPropagator propagator(options);

  for (int i = 0; i <= 10; ++i) {
    propagator.addSample(makeSample(
      static_cast<double>(i) * 0.01,
      Eigen::Vector3d(0.0, 0.0, 2.0)));
  }

  const ImuPropagationResult result = propagator.propagateBetween(
    timeFromSeconds(0.0),
    timeFromSeconds(0.1));

  EXPECT_FALSE(result.success);
  EXPECT_EQ("delta_yaw_too_large", result.status);
}

TEST(RotationIntegrationTest, FailsExplicitlyWhenTranslationIsEnabled)
{
  ImuPropagatorOptions options;
  options.use_imu_translation = true;

  ImuPropagator propagator(options);
  propagator.addSample(makeSample(0.00, Eigen::Vector3d::Zero()));
  propagator.addSample(makeSample(0.01, Eigen::Vector3d::Zero()));

  const ImuPropagationResult result = propagator.propagateBetween(
    timeFromSeconds(0.0),
    timeFromSeconds(0.01));

  EXPECT_FALSE(result.success);
  EXPECT_EQ("imu_translation_not_implemented", result.status);
}

}  // namespace custom_imu_propagator
