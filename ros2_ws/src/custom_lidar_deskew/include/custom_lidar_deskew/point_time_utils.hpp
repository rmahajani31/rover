#pragma once

#include <cstddef>
#include <optional>
#include <string>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

namespace custom_lidar_deskew
{

enum class PointTimeUnit
{
  Seconds,
  Microseconds,
  Nanoseconds
};

struct PointTimeSummary
{
  bool has_point_time = false;
  std::size_t total_point_count = 0;
  std::size_t valid_point_count = 0;
  double min_relative_time_sec = 0.0;
  double max_relative_time_sec = 0.0;
};

std::optional<PointTimeUnit> parsePointTimeUnit(const std::string& unit);

const sensor_msgs::msg::PointField* findPointField(
  const sensor_msgs::msg::PointCloud2& cloud,
  const std::string& field_name);

bool hasPointTimeField(
  const sensor_msgs::msg::PointCloud2& cloud,
  const std::string& field_name);

std::optional<double> readPointRelativeTimeSec(
  const sensor_msgs::msg::PointCloud2& cloud,
  std::size_t point_index,
  const sensor_msgs::msg::PointField& field,
  PointTimeUnit unit);

PointTimeSummary summarizePointTimes(
  const sensor_msgs::msg::PointCloud2& cloud,
  const std::string& field_name,
  PointTimeUnit unit);

}  // namespace custom_lidar_deskew
