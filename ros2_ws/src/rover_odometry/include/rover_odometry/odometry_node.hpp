#pragma once

#include <memory>
#include <string>
#include <vector>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "rover_odometry/pinpoint_i2c.hpp"

namespace rover_odometry
{
    class OdometryNode : public rclcpp::Node
    {
        public:
            OdometryNode();

        private:
            void declareParameters();
            void loadParameters();
            void initializeDevice();
            void initializeState();
            void update();

            int bus_;
            int addr_;
            std::string endian_string_;
            double publish_rate_hz_;
            std::vector<double> pod_offsets_mm_;
            std::vector<bool> encoder_directions_;
            std::string odom_frame_;
            std::string base_frame_;

            float x_mm_{0.0f};
            float y_mm_{0.0f};
            float yaw_rad_{0.0f};
            float vx_mm_s_{0.0f};
            float vy_mm_s_{0.0f};
            float yaw_rate_rad_s_{0.0f};

            std::unique_ptr<PinpointI2C> i2c_;
            rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
            std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
            rclcpp::TimerBase::SharedPtr timer_;
    };
}
