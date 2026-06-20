#include "custom_lidar_deskew/point_time_utils.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>

namespace custom_lidar_deskew
{
namespace
{

double unitScale(PointTimeUnit unit)
{
  switch (unit) {
    case PointTimeUnit::Seconds:
      return 1.0;
    case PointTimeUnit::Microseconds:
      return 1e-6;
    case PointTimeUnit::Nanoseconds:
      return 1e-9;
  }

  return 1.0;
}

std::optional<std::size_t> datatypeSize(std::uint8_t datatype)
{
  using sensor_msgs::msg::PointField;

  switch (datatype) {
    case PointField::INT8:
    case PointField::UINT8:
      return 1;
    case PointField::INT16:
    case PointField::UINT16:
      return 2;
    case PointField::INT32:
    case PointField::UINT32:
    case PointField::FLOAT32:
      return 4;
    case PointField::FLOAT64:
      return 8;
    default:
      return std::nullopt;
  }
}

template <typename T>
std::optional<double> readScalarAsDouble(
  const sensor_msgs::msg::PointCloud2& cloud,
  std::size_t byte_offset)
{
  if (byte_offset + sizeof(T) > cloud.data.size()) {
    return std::nullopt;
  }

  T value{};
  std::memcpy(&value, cloud.data.data() + byte_offset, sizeof(T));
  return static_cast<double>(value);
}

std::optional<double> readRawFieldValue(
  const sensor_msgs::msg::PointCloud2& cloud,
  std::size_t byte_offset,
  std::uint8_t datatype)
{
  using sensor_msgs::msg::PointField;

  switch (datatype) {
    case PointField::INT8:
      return readScalarAsDouble<std::int8_t>(cloud, byte_offset);
    case PointField::UINT8:
      return readScalarAsDouble<std::uint8_t>(cloud, byte_offset);
    case PointField::INT16:
      return readScalarAsDouble<std::int16_t>(cloud, byte_offset);
    case PointField::UINT16:
      return readScalarAsDouble<std::uint16_t>(cloud, byte_offset);
    case PointField::INT32:
      return readScalarAsDouble<std::int32_t>(cloud, byte_offset);
    case PointField::UINT32:
      return readScalarAsDouble<std::uint32_t>(cloud, byte_offset);
    case PointField::FLOAT32:
      return readScalarAsDouble<float>(cloud, byte_offset);
    case PointField::FLOAT64:
      return readScalarAsDouble<double>(cloud, byte_offset);
    default:
      return std::nullopt;
  }
}

std::size_t pointCount(const sensor_msgs::msg::PointCloud2& cloud)
{
  return static_cast<std::size_t>(cloud.width) * static_cast<std::size_t>(cloud.height);
}

std::optional<std::size_t> pointFieldByteOffset(
  const sensor_msgs::msg::PointCloud2& cloud,
  std::size_t point_index,
  const sensor_msgs::msg::PointField& field)
{
  if (cloud.width == 0 || cloud.height == 0 || cloud.point_step == 0 || cloud.row_step == 0) {
    return std::nullopt;
  }

  if (field.count == 0) {
    return std::nullopt;
  }

  const auto field_size = datatypeSize(field.datatype);
  if (!field_size.has_value()) {
    return std::nullopt;
  }

  if (field.offset + field_size.value() > cloud.point_step) {
    return std::nullopt;
  }

  const std::size_t total_points = pointCount(cloud);
  if (point_index >= total_points) {
    return std::nullopt;
  }

  const std::size_t width = static_cast<std::size_t>(cloud.width);
  const std::size_t row = point_index / width;
  const std::size_t col = point_index % width;
  const std::size_t byte_offset =
    row * static_cast<std::size_t>(cloud.row_step) +
    col * static_cast<std::size_t>(cloud.point_step) +
    static_cast<std::size_t>(field.offset);

  if (byte_offset + field_size.value() > cloud.data.size()) {
    return std::nullopt;
  }

  return byte_offset;
}

}  // namespace

std::optional<PointTimeUnit> parsePointTimeUnit(const std::string& unit)
{
  if (unit == "seconds") {
    return PointTimeUnit::Seconds;
  }

  if (unit == "microseconds") {
    return PointTimeUnit::Microseconds;
  }

  if (unit == "nanoseconds") {
    return PointTimeUnit::Nanoseconds;
  }

  return std::nullopt;
}

const sensor_msgs::msg::PointField* findPointField(
  const sensor_msgs::msg::PointCloud2& cloud,
  const std::string& field_name)
{
  const auto it = std::find_if(
    cloud.fields.begin(),
    cloud.fields.end(),
    [&field_name](const sensor_msgs::msg::PointField& field) {
      return field.name == field_name;
    });

  if (it == cloud.fields.end()) {
    return nullptr;
  }

  return &(*it);
}

bool hasPointTimeField(
  const sensor_msgs::msg::PointCloud2& cloud,
  const std::string& field_name)
{
  return findPointField(cloud, field_name) != nullptr;
}

std::optional<double> readPointRelativeTimeSec(
  const sensor_msgs::msg::PointCloud2& cloud,
  std::size_t point_index,
  const sensor_msgs::msg::PointField& field,
  PointTimeUnit unit)
{
  const auto byte_offset = pointFieldByteOffset(cloud, point_index, field);
  if (!byte_offset.has_value()) {
    return std::nullopt;
  }

  const auto raw_value = readRawFieldValue(cloud, byte_offset.value(), field.datatype);
  if (!raw_value.has_value()) {
    return std::nullopt;
  }

  const double relative_time_sec = raw_value.value() * unitScale(unit);
  if (!std::isfinite(relative_time_sec)) {
    return std::nullopt;
  }

  return relative_time_sec;
}

PointTimeSummary summarizePointTimes(
  const sensor_msgs::msg::PointCloud2& cloud,
  const std::string& field_name,
  PointTimeUnit unit)
{
  PointTimeSummary summary;
  summary.total_point_count = pointCount(cloud);

  const auto* field = findPointField(cloud, field_name);
  if (field == nullptr || summary.total_point_count == 0) {
    return summary;
  }

  double min_time = std::numeric_limits<double>::infinity();
  double max_time = -std::numeric_limits<double>::infinity();

  for (std::size_t i = 0; i < summary.total_point_count; ++i) {
    const auto relative_time = readPointRelativeTimeSec(cloud, i, *field, unit);
    if (!relative_time.has_value()) {
      continue;
    }

    min_time = std::min(min_time, relative_time.value());
    max_time = std::max(max_time, relative_time.value());
    ++summary.valid_point_count;
  }

  if (summary.valid_point_count == 0) {
    return summary;
  }

  summary.has_point_time = true;
  summary.min_relative_time_sec = min_time;
  summary.max_relative_time_sec = max_time;
  return summary;
}

}  // namespace custom_lidar_deskew
