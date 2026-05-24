#pragma once

#include <cmath>
#include <numbers>

#include <geometry_msgs/msg/quaternion.hpp>

namespace rover_odometry
{
    inline double wrapPi(double angle)
    {
        while (angle > std::numbers::pi) {
            angle -= 2.0 * std::numbers::pi;
        }
        while (angle < -std::numbers::pi) {
            angle += 2.0 * std::numbers::pi;
        }
        return angle;
    }

    inline geometry_msgs::msg::Quaternion yawToQuaternion(double yaw)
    {
        const double half = 0.5 * yaw;

        geometry_msgs::msg::Quaternion q;
        q.x = 0.0;
        q.y = 0.0;
        q.z = std::sin(half);
        q.w = std::cos(half);
        return q;
    }
}