#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "rover_odometry/odometry_node.hpp"

namespace rover_odometry
{
    namespace
    {
        class FakeOdometryDevice : public OdometryDevice
        {
            public:
                explicit FakeOdometryDevice(std::vector<float> reads)
                : reads_(std::move(reads))
                {
                }

                float readF32(std::uint8_t) override
                {
                    if (throw_on_read_) {
                        throw std::runtime_error("read failed");
                    }

                    if (read_index_ >= reads_.size()) {
                        throw std::runtime_error("out of fake readings");
                    }

                    return reads_[read_index_++];
                }

                void resetPosAndImu() override
                {
                }

                void setEncoderDirections(bool, bool) override
                {
                }

                void setPodOffsetsMm(float, float) override
                {
                }

                bool throw_on_read_{false};
                std::size_t read_index_{0};

            private:
                std::vector<float> reads_;
        };

        class RclcppFixture : public ::testing::Test
        {
            protected:
                static void SetUpTestSuite()
                {
                    if (!rclcpp::ok()) {
                        int argc = 0;
                        char ** argv = nullptr;
                        rclcpp::init(argc, argv);
                    }
                }

                static void TearDownTestSuite()
                {
                    if (rclcpp::ok()) {
                        rclcpp::shutdown();
                    }
                }
        };
    }

    TEST(ParseEndianTest, AcceptsKnownValues)
    {
        EXPECT_EQ(parseEndian("little"), PinpointI2C::Endian::Little);
        EXPECT_EQ(parseEndian("big"), PinpointI2C::Endian::Big);
    }

    TEST(ParseEndianTest, RejectsUnknownValues)
    {
        EXPECT_THROW(parseEndian("middle"), std::invalid_argument);
    }

    TEST_F(RclcppFixture, RejectsNonPositivePublishRate)
    {
        auto device = std::make_unique<FakeOdometryDevice>(
        std::vector<float>{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f});

        rclcpp::NodeOptions options;
        options.append_parameter_override("publish_rate_hz", 0.0);

        EXPECT_THROW(OdometryNode(options, std::move(device)), std::invalid_argument);
    }

    TEST_F(RclcppFixture, PreservesHighRateTimerResolution)
    {
        auto device = std::make_unique<FakeOdometryDevice>(
        std::vector<float>{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f});

        rclcpp::NodeOptions options;
        options.append_parameter_override("publish_rate_hz", 2000.0);

        OdometryNode node(options, std::move(device));

        EXPECT_GT(node.timerPeriod().count(), 0);
        EXPECT_EQ(node.timerPeriod(), std::chrono::nanoseconds(500000));
    }

    TEST_F(RclcppFixture, UsesRelativeOdomTopicByDefault)
    {
        auto device = std::make_unique<FakeOdometryDevice>(
        std::vector<float>{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f});

        OdometryNode node(rclcpp::NodeOptions(), std::move(device));

        EXPECT_EQ(node.odomTopic(), "odom");
    }

    TEST_F(RclcppFixture, SupportsOverriddenOdomTopic)
    {
        auto device = std::make_unique<FakeOdometryDevice>(
        std::vector<float>{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f});

        rclcpp::NodeOptions options;
        options.append_parameter_override("odom_topic", "robot1/odom");

        OdometryNode node(options, std::move(device));

        EXPECT_EQ(node.odomTopic(), "robot1/odom");
    }

    TEST_F(RclcppFixture, UpdateSwallowsRuntimeReadFailures)
    {
        auto device = std::make_unique<FakeOdometryDevice>(
        std::vector<float>{1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f});
        auto * device_ptr = device.get();

        OdometryNode node(rclcpp::NodeOptions(), std::move(device));

        device_ptr->throw_on_read_ = true;

        EXPECT_NO_THROW(node.updateOnce());
    }
}
