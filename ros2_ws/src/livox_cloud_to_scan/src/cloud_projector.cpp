#include "livox_cloud_to_scan/cloud_projector.hpp"

#include <cmath>
#include <cstring>
#include <limits>

#include <geometry_msgs/msg/point_stamped>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace livox_cloud_to_scan
{
    CloudProjector::CloudProjector(const ScanProjectionParams & params)
    : params_(params)
    {
    }

    float CloudProjector::readFloat32(const std::uint8_t * point_data, int offset) const
    {
        float value;
        std::memcpy(&value, point_data + offset, sizeof(float));
        return value;
    }

    bool CloudProjector::getFieldOffsets(
        const sensor_msgs::msg::PointCloud2 & cloud,
        int & x_offset,
        int & y_offset,
        int & z_offset,
    ) const
    {
        x_offset = -1;
        y_offset = -1;
        z_offset = -1;

        for (const auto & field: cloud.fields) {
            if (field.name == "x") {
                x_offset = field.offset;
            } else if (field.name == "y") {
                y_offset = field.offset;
            } else if (field.name == "z") {
                z_offset = field.offset;
            }
        }

        return x_offset >= 0 && y_offset >= 0 && z_offset >= 0;
    }

    sensor_msgs::msg::LaserScan CloudProjector::project(
        const sensor_msgs::msg::PointCloud2 & cloud,
        const geometry_msgs::msg::TransformStamped & transform_msg)
    {
        sensor_msgs::msg::LaserScan scan;

        scan.header.stamp = cloud.header.stamp;
        scan.header.frame_id = params_.target_frame;

        scan.angle_min = static_cast<float>(params_.angle_min);
        scan.angle_max = static_cast<float>(params_.angle_max);
        scan.angle_increment = static_cast<float>(params_.angle_increment);

        scan.time_increment = 0.0f;
        scan.scan_time = static_cast<float>(params_.scan_time);
        
        scan.range_min = static_cast<float>(params_.min_range);
        scan.range_max = static_cast<float>(params_.max_range);

        const int num_bins = static_cast<int>(
            std::ceil((params_.angle_max - params_.angle_min) / params_.angle_increment)
        );

        scan.ranges.assign(num_bins, std::numeric_limits<float>::infinity());
        scan.intensities.assign(num_bins, 0.0f);

        int x_offset;
        int y_offset;
        int z_offset;

        if (!getFieldOffsets(cloud, x_offset, y_offset, z_offset)) {
            return scan;
        }

        const std::size_t num_points = static_cast<std::size_t>(cloud.width) * static_cast<std::size_t>(cloud.height);

        for (std::size_t i; i < num_points; ++i) {
            const std::uint8_t * point_data = &cloud.data[i * cloud.point_step];

            const float x_raw = readFloat32(point_data, x_offset);
            const float y_raw = readFloat32(point_data, y_offset);
            const float z_raw = readFloat32(point_data, z_offset);

            if (!std::isfinite(x_raw) || !std::isfinite(y_raw) || !std::isfinite(z_raw)) {
                continue;
            }

            geometry_msgs::msg::PointStamped p_in;
            p_in.header = cloud.header;
            p_in.point.x = x_raw;
            p_in.point.y = y_raw;
            p_in.point.z = z_raw;

            geometry_msgs::msg::PointStamped p_out;
            tf2::doTransform(p_in, p_out, transform_msg);

            const double x = p_out.point.x;
            const double y = p_out.point.y;
            const double z = p_out.point.z;

            if (z < params_.min_height || z > params_.max_height) {
                continue;
            }

            const double range = std::sqrt(x*x + y*y);

            if (range < params_.min_range || range > params_.max_range) {
                continue;
            }

            const double angle = std::atan2(y, x);

            if (angle < params_.angle_min || angle > params_.angle_max) {
                continue;
            }

            const int bin = static_cast<int>(
                std::floor((angle - params_.angle_min) / params_.angle_increment)
            );

            if (bin < 0 || bin > num_bins) {
                continue;
            }

            if (range < scan.ranges[bin]) {
                scan.ranges[bin] = range;
            }
        }

        return scan;
    }
}