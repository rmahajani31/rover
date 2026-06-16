#include "custom_livox_costmap_projection/occupancy_grid_builder.hpp"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace custom_livox_costmap_projection
{

namespace
{

std::uint32_t metersToCells(const double meters, const double resolution)
{
  // Use ceil so the requested physical size is never rounded down.
  return static_cast<std::uint32_t>(
    std::max(1.0, std::ceil(meters / resolution)));
}

tf2::Matrix3x3 rotationMatrixFromTransform(
  const geometry_msgs::msg::TransformStamped & transform)
{
  const auto & q_msg = transform.transform.rotation;
  const tf2::Quaternion q(q_msg.x, q_msg.y, q_msg.z, q_msg.w);
  return tf2::Matrix3x3(q);
}

}  // namespace

OccupancyGridBuilder::OccupancyGridBuilder(const ProjectionParameters & params)
: params_(params),
  width_cells_(metersToCells(params.grid_width_m, params.grid_resolution)),
  height_cells_(metersToCells(params.grid_height_m, params.grid_resolution))
{
}

nav_msgs::msg::OccupancyGrid OccupancyGridBuilder::buildGrid(
  const pcl::PointCloud<pcl::PointXYZI> & obstacle_cloud,
  const geometry_msgs::msg::TransformStamped & grid_from_target_transform,
  const rclcpp::Time & stamp,
  std::uint64_t & occupied_cell_count) const
{
  occupied_cell_count = 0U;

  nav_msgs::msg::OccupancyGrid grid;
  grid.header.stamp = stamp;
  grid.header.frame_id = params_.grid_frame;

  grid.info.map_load_time = stamp;
  grid.info.resolution = static_cast<float>(params_.grid_resolution);
  grid.info.width = width_cells_;
  grid.info.height = height_cells_;

  const double robot_x = grid_from_target_transform.transform.translation.x;
  const double robot_y = grid_from_target_transform.transform.translation.y;

  // The grid origin is the lower-left corner in grid_frame. Centering it on the
  // robot makes cell coordinates local to a fixed-size window around the rover.
  grid.info.origin.position.x = robot_x - params_.grid_width_m * 0.5;
  grid.info.origin.position.y = robot_y - params_.grid_height_m * 0.5;
  grid.info.origin.position.z = 0.0;
  grid.info.origin.orientation.w = 1.0;

  grid.data.assign(
    static_cast<std::size_t>(width_cells_) * static_cast<std::size_t>(height_cells_),
    static_cast<std::int8_t>(params_.unknown_value));

  const auto rotation = rotationMatrixFromTransform(grid_from_target_transform);
  const auto & translation = grid_from_target_transform.transform.translation;

  for (const auto & point : obstacle_cloud.points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      continue;
    }

    const double grid_x =
      rotation[0][0] * point.x +
      rotation[0][1] * point.y +
      rotation[0][2] * point.z +
      translation.x;

    const double grid_y =
      rotation[1][0] * point.x +
      rotation[1][1] * point.y +
      rotation[1][2] * point.z +
      translation.y;

    int cell_x = 0;
    int cell_y = 0;
    if (!worldToGridCell(grid, grid_x, grid_y, cell_x, cell_y)) {
      continue;
    }

    markOccupiedWithDilation(grid, cell_x, cell_y, occupied_cell_count);
  }

  return grid;
}

void OccupancyGridBuilder::markOccupiedWithDilation(
  nav_msgs::msg::OccupancyGrid & grid,
  const int center_cell_x,
  const int center_cell_y,
  std::uint64_t & occupied_cell_count) const
{
  const int radius = params_.obstacle_cell_dilation_radius_cells;

  // Inflate each hit by a small square kernel so sparse Livox returns are easier
  // to see in RViz and less likely to disappear between cells.
  for (int dy = -radius; dy <= radius; ++dy) {
    for (int dx = -radius; dx <= radius; ++dx) {
      const int cell_x = center_cell_x + dx;
      const int cell_y = center_cell_y + dy;

      if (!isInsideGrid(cell_x, cell_y)) {
        continue;
      }

      const auto index = cellIndex(cell_x, cell_y);
      if (grid.data[index] != params_.occupied_value) {
        grid.data[index] = static_cast<std::int8_t>(params_.occupied_value);
        ++occupied_cell_count;
      }
    }
  }
}

bool OccupancyGridBuilder::worldToGridCell(
  const nav_msgs::msg::OccupancyGrid & grid,
  const double world_x,
  const double world_y,
  int & cell_x,
  int & cell_y) const
{
  const double local_x = world_x - grid.info.origin.position.x;
  const double local_y = world_y - grid.info.origin.position.y;

  if (local_x < 0.0 || local_y < 0.0) {
    return false;
  }

  // Resolution converts meters to discrete cell indices. floor maps every point
  // inside a cell-sized interval to that cell.
  cell_x = static_cast<int>(std::floor(local_x / params_.grid_resolution));
  cell_y = static_cast<int>(std::floor(local_y / params_.grid_resolution));

  return isInsideGrid(cell_x, cell_y);
}

bool OccupancyGridBuilder::isInsideGrid(const int cell_x, const int cell_y) const
{
  return cell_x >= 0 &&
         cell_y >= 0 &&
         cell_x < static_cast<int>(width_cells_) &&
         cell_y < static_cast<int>(height_cells_);
}

std::size_t OccupancyGridBuilder::cellIndex(const int cell_x, const int cell_y) const
{
  return static_cast<std::size_t>(
    cell_y * static_cast<int>(width_cells_) + cell_x);
}

}  // namespace custom_livox_costmap_projection
