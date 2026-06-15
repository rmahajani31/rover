#pragma once

#include "custom_livox_costmap_projection/projection_parameters.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rclcpp/rclcpp.hpp>

#include <cstddef>
#include <cstdint>

namespace custom_livox_costmap_projection
{

class OccupancyGridBuilder
{
public:
  explicit OccupancyGridBuilder(const ProjectionParameters & params);

  nav_msgs::msg::OccupancyGrid buildGrid(
    const pcl::PointCloud<pcl::PointXYZI> & obstacle_cloud,
    const geometry_msgs::msg::TransformStamped & grid_from_target_transform,
    const rclcpp::Time & stamp,
    std::uint64_t & occupied_cell_count) const;

private:
  void markOccupiedWithDilation(
    nav_msgs::msg::OccupancyGrid & grid,
    int center_cell_x,
    int center_cell_y,
    std::uint64_t & occupied_cell_count) const;

  bool worldToGridCell(
    const nav_msgs::msg::OccupancyGrid & grid,
    double world_x,
    double world_y,
    int & cell_x,
    int & cell_y) const;

  bool isInsideGrid(int cell_x, int cell_y) const;
  std::size_t cellIndex(int cell_x, int cell_y) const;

  ProjectionParameters params_;
  std::uint32_t width_cells_;
  std::uint32_t height_cells_;
};

}  // namespace custom_livox_costmap_projection
