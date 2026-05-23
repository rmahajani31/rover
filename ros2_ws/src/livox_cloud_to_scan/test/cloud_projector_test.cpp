#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <numbers>
#include <string>
#include <tuple>
#include <vector>

#include <gtest/gtest.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include "livox_cloud_to_scan/cloud_projector.hpp"
#include "livox_cloud_to_scan/scan_projection_params.hpp"

namespace
{
sensor_msgs::msg::PointCloud2 makeCloud(
    const std::vector<std::tuple<float, float, float>> & points)
{
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.frame_id = "livox_frame";
    cloud.height = 1;
    cloud.width = static_cast<std::uint32_t>(points.size());
    cloud.is_bigendian = false;
    cloud.is_dense = false;
    cloud.point_step = 12;
    cloud.row_step = cloud.point_step * cloud.width;
    cloud.fields.resize(3);

    cloud.fields[0].name = "x";
    cloud.fields[0].offset = 0;
    cloud.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud.fields[0].count = 1;

    cloud.fields[1].name = "y";
    cloud.fields[1].offset = 4;
    cloud.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud.fields[1].count = 1;

    cloud.fields[2].name = "z";
    cloud.fields[2].offset = 8;
    cloud.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud.fields[2].count = 1;

    cloud.data.resize(static_cast<std::size_t>(cloud.row_step));
    std::size_t offset = 0;
    for (const auto & point : points) {
        const float x = std::get<0>(point);
        const float y = std::get<1>(point);
        const float z = std::get<2>(point);
        std::memcpy(cloud.data.data() + offset, &x, sizeof(float));
        std::memcpy(cloud.data.data() + offset + 4, &y, sizeof(float));
        std::memcpy(cloud.data.data() + offset + 8, &z, sizeof(float));
        offset += cloud.point_step;
    }

    return cloud;
}

geometry_msgs::msg::TransformStamped makeIdentityTransform()
{
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = "base_link";
    transform.child_frame_id = "livox_frame";
    transform.transform.rotation.w = 1.0;
    return transform;
}
}  // namespace

TEST(ScanProjectionParamsTest, RejectsInvalidRangesAndAngles)
{
    livox_cloud_to_scan::ScanProjectionParams params;
    std::string error;

    params.angle_increment = 0.0;
    EXPECT_FALSE(params.validate(&error));
    EXPECT_EQ(error, "angle_increment must be greater than zero");

    params.angle_increment = 0.1;
    params.angle_max = params.angle_min;
    EXPECT_FALSE(params.validate(&error));
    EXPECT_EQ(error, "angle_max must be greater than angle_min");

    params.angle_max = std::numbers::pi;
    params.min_range = 5.0;
    params.max_range = 2.0;
    EXPECT_FALSE(params.validate(&error));
    EXPECT_EQ(error, "range bounds must satisfy 0 <= min_range <= max_range");
}

TEST(CloudProjectorTest, KeepsNearestPointPerBinAndHandlesAngleMaxBoundary)
{
    livox_cloud_to_scan::ScanProjectionParams params;
    params.target_frame = "base_link";
    params.min_height = -1.0;
    params.max_height = 1.0;
    params.min_range = 0.0;
    params.max_range = 10.0;
    params.angle_min = -std::numbers::pi / 4.0;
    params.angle_max = std::numbers::pi / 4.0;
    params.angle_increment = std::numbers::pi / 4.0;

    livox_cloud_to_scan::CloudProjector projector(params);
    const auto transform = makeIdentityTransform();
    const auto cloud = makeCloud({
        {2.0f, 0.0f, 0.0f},
        {1.0f, 0.0f, 0.0f},
        {1.0f, 1.0f, 0.0f}
    });

    std::string error;
    const auto scan = projector.project(cloud, transform, &error);

    ASSERT_TRUE(error.empty());
    ASSERT_EQ(scan.ranges.size(), 2U);
    EXPECT_TRUE(std::isinf(scan.ranges[0]));
    EXPECT_FLOAT_EQ(scan.ranges[1], 1.0f);
}

TEST(CloudProjectorTest, FiltersHeightRangeAndNaNPoints)
{
    livox_cloud_to_scan::ScanProjectionParams params;
    params.target_frame = "base_link";
    params.min_height = 0.0;
    params.max_height = 0.5;
    params.min_range = 0.5;
    params.max_range = 3.0;
    params.angle_min = -1.0;
    params.angle_max = 1.0;
    params.angle_increment = 0.5;

    livox_cloud_to_scan::CloudProjector projector(params);
    const auto transform = makeIdentityTransform();
    const auto cloud = makeCloud({
        {1.0f, 0.0f, 1.0f},
        {0.2f, 0.0f, 0.1f},
        {std::numeric_limits<float>::quiet_NaN(), 0.0f, 0.0f},
        {1.5f, 0.0f, 0.2f}
    });

    std::string error;
    const auto scan = projector.project(cloud, transform, &error);

    ASSERT_TRUE(error.empty());
    EXPECT_EQ(scan.ranges.size(), 4U);
    EXPECT_FLOAT_EQ(scan.ranges[2], 1.5f);
}

TEST(CloudProjectorTest, RejectsMissingFieldsAndTruncatedData)
{
    livox_cloud_to_scan::ScanProjectionParams params;
    params.target_frame = "base_link";
    params.min_height = -1.0;
    params.max_height = 1.0;
    params.min_range = 0.0;
    params.max_range = 10.0;
    params.angle_min = -1.0;
    params.angle_max = 1.0;
    params.angle_increment = 0.5;

    livox_cloud_to_scan::CloudProjector projector(params);
    const auto transform = makeIdentityTransform();

    auto missing_field_cloud = makeCloud({{1.0f, 0.0f, 0.0f}});
    missing_field_cloud.fields.pop_back();

    std::string error;
    auto scan = projector.project(missing_field_cloud, transform, &error);
    EXPECT_EQ(error, "point cloud is missing FLOAT32 x/y/z fields");
    EXPECT_EQ(scan.ranges.size(), 4U);
    EXPECT_TRUE(std::all_of(
        scan.ranges.begin(),
        scan.ranges.end(),
        [](float range) { return std::isinf(range); }));

    auto truncated_cloud = makeCloud({{1.0f, 0.0f, 0.0f}});
    truncated_cloud.data.resize(4);

    error.clear();
    scan = projector.project(truncated_cloud, transform, &error);
    EXPECT_EQ(error, "point cloud data buffer is smaller than row_step * height");
    EXPECT_EQ(scan.ranges.size(), 4U);
}
