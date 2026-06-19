#include <cmath>
#include <cstdint>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "custom_imu_propagator/imu_propagator.hpp"
#include "custom_imu_propagator/imu_sample.hpp"

namespace custom_imu_propagator
{
namespace
{

rclcpp::Time timeFromSeconds(double seconds)
{
  return rclcpp::Time(
    static_cast<std::int64_t>(std::llround(seconds * 1.0e9)),
    RCL_ROS_TIME);
}

ImuSample makeSample(double stamp_sec)
{
  ImuSample sample;
  sample.stamp = timeFromSeconds(stamp_sec);
  return sample;
}

}  // namespace

TEST(ImuIntervalExtractionTest, ReturnsSamplesInsideCoveredInterval)
{
  ImuPropagator propagator;

  for (int i = 0; i <= 10; ++i) {
    propagator.addSample(makeSample(static_cast<double>(i) * 0.01));
  }

  std::vector<ImuSample> samples;
  const bool ok = propagator.getSamplesInInterval(
    timeFromSeconds(0.02),
    timeFromSeconds(0.06),
    samples);

  ASSERT_TRUE(ok);
  ASSERT_EQ(5U, samples.size());
  EXPECT_EQ(timeFromSeconds(0.02), samples.front().stamp);
  EXPECT_EQ(timeFromSeconds(0.06), samples.back().stamp);
}

TEST(ImuIntervalExtractionTest, RejectsIntervalBeforeBufferStart)
{
  ImuPropagator propagator;

  for (int i = 2; i <= 10; ++i) {
    propagator.addSample(makeSample(static_cast<double>(i) * 0.01));
  }

  std::vector<ImuSample> samples;
  const bool ok = propagator.getSamplesInInterval(
    timeFromSeconds(0.01),
    timeFromSeconds(0.06),
    samples);

  EXPECT_FALSE(ok);
  EXPECT_TRUE(samples.empty());
}

TEST(ImuIntervalExtractionTest, RejectsIntervalAfterBufferEnd)
{
  ImuPropagator propagator;

  for (int i = 0; i <= 6; ++i) {
    propagator.addSample(makeSample(static_cast<double>(i) * 0.01));
  }

  std::vector<ImuSample> samples;
  const bool ok = propagator.getSamplesInInterval(
    timeFromSeconds(0.02),
    timeFromSeconds(0.07),
    samples);

  EXPECT_FALSE(ok);
  EXPECT_TRUE(samples.empty());
}

TEST(ImuIntervalExtractionTest, RejectsOutOfOrderSamples)
{
  ImuPropagator propagator;

  propagator.addSample(makeSample(0.00));
  propagator.addSample(makeSample(0.02));
  propagator.addSample(makeSample(0.01));
  propagator.addSample(makeSample(0.03));

  std::vector<ImuSample> samples;
  const bool ok = propagator.getSamplesInInterval(
    timeFromSeconds(0.00),
    timeFromSeconds(0.03),
    samples);

  ASSERT_TRUE(ok);
  ASSERT_EQ(3U, samples.size());
  EXPECT_EQ(timeFromSeconds(0.00), samples[0].stamp);
  EXPECT_EQ(timeFromSeconds(0.02), samples[1].stamp);
  EXPECT_EQ(timeFromSeconds(0.03), samples[2].stamp);
}

}  // namespace custom_imu_propagator
