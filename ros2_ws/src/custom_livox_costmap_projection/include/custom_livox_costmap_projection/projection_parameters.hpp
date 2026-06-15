#pragma once

#include <rclcpp/rclcpp.hpp>

#include <string>

namespace custom_livox_costmap_projection
{

struct ProjectionParameters
{
  std::string input_cloud_topic = "/custom/points_preprocessed";
  std::string obstacle_cloud_topic = "/custom/obstacle_cloud";
  std::string occupancy_grid_topic = "/custom/projected_occupancy_grid";
  std::string diagnostics_topic = "/custom/costmap_projection_diagnostics";

  bool publish_obstacle_cloud = true;
  bool publish_occupancy_grid = true;
  bool publish_diagnostics = true;

  std::string target_frame = "base_link";
  std::string grid_frame = "odom";
  std::string robot_frame = "base_link";

  double min_range = 0.25;
  double max_range = 8.0;

  double floor_threshold = 0.03;
  double ceiling_threshold = 1.20;
  double obstacle_min_z = 0.05;
  double obstacle_max_z = 1.20;

  bool remove_self_points = true;
  double self_filter_min_x = -0.35;
  double self_filter_max_x = 0.35;
  double self_filter_min_y = -0.35;
  double self_filter_max_y = 0.35;
  double self_filter_min_z = -0.20;
  double self_filter_max_z = 0.50;

  double grid_resolution = 0.05;
  double grid_width_m = 10.0;
  double grid_height_m = 10.0;
  int occupied_value = 100;
  int unknown_value = -1;

  int obstacle_cell_dilation_radius_cells = 1;

  double tf_lookup_timeout_sec = 0.05;

  int diagnostics_frame_count_window = 30;

  static void declare(rclcpp::Node & node);
  static ProjectionParameters load(rclcpp::Node & node);

  void validate() const;
  void logSummary(const rclcpp::Logger & logger) const;
};

}  // namespace custom_livox_costmap_projection
