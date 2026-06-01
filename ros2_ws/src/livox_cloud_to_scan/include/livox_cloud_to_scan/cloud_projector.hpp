#pragma once

#include <cstdint>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "livox_cloud_to_scan/scan_projection_params.hpp"

namespace livox_cloud_to_scan
{
    class CloudProjector
    {
        public:
            
            explicit CloudProjector(const ScanProjectionParams & params);

            // Convert one PointCloud2 into a LaserScan in target_frame using
            // height/range filters and nearest-return-per-angle-bin selection.
            sensor_msgs::msg::LaserScan project(
                const sensor_msgs::msg::PointCloud2 & cloud,
                const geometry_msgs::msg::TransformStamped & transform_msg,
                std::string * error = nullptr
            );
        
        private:

            ScanProjectionParams params_;

            // Livox driver clouds are expected to expose FLOAT32 x/y/z fields,
            // but offsets are discovered instead of assuming a packed layout.
            bool getFieldOffsets(
                const sensor_msgs::msg::PointCloud2 & cloud,
                int & x_offset,
                int & y_offset,
                int & z_offset
            ) const;

            float readFloat32(const std::uint8_t * point_data, int offset) const;
    };
}
