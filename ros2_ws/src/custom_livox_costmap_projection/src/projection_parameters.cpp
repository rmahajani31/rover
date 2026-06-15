#include "custom_livox_costmap_projection/projection_parameters.hpp"

#include <stdexcept>
#include <string>

namespace custom_livox_costmap_projection
{

namespace
{

void requireNonEmpty(const std::string & value, const std::string & name)
{
  if (value.empty()) {
    throw std::runtime_error(name + " must not be empty");
  }
}

void requireGreaterThan(
  const double value,
  const double lower_bound,
  const std::string & name,
  const std::string & lower_bound_name)
{
  if (value <= lower_bound) {
    throw std::runtime_error(name + " must be greater than " + lower_bound_name);
  }
}

void requireInOccupancyRange(const int value, const std::string & name)
{
  if (value < -1 || value > 100) {
    throw std::runtime_error(name + " must be in the OccupancyGrid range [-1, 100]");
  }
}

}  // namespace

void ProjectionParameters::declare(rclcpp::Node & node)
{
  node.declare_parameter<std::string>("input_cloud_topic", "/custom/points_preprocessed");
  node.declare_parameter<std::string>("obstacle_cloud_topic", "/custom/obstacle_cloud");
  node.declare_parameter<std::string>("occupancy_grid_topic", "/custom/projected_occupancy_grid");
  node.declare_parameter<std::string>(
    "diagnostics_topic", "/custom/costmap_projection_diagnostics");

  node.declare_parameter<bool>("publish_obstacle_cloud", true);
  node.declare_parameter<bool>("publish_occupancy_grid", true);
  node.declare_parameter<bool>("publish_diagnostics", true);

  node.declare_parameter<std::string>("target_frame", "base_link");
  node.declare_parameter<std::string>("grid_frame", "odom");
  node.declare_parameter<std::string>("robot_frame", "base_link");

  node.declare_parameter<double>("min_range", 0.25);
  node.declare_parameter<double>("max_range", 8.0);

  node.declare_parameter<double>("floor_threshold", 0.03);
  node.declare_parameter<double>("ceiling_threshold", 1.20);
  node.declare_parameter<double>("obstacle_min_z", 0.05);
  node.declare_parameter<double>("obstacle_max_z", 1.20);

  node.declare_parameter<bool>("remove_self_points", true);
  node.declare_parameter<double>("self_filter_min_x", -0.35);
  node.declare_parameter<double>("self_filter_max_x", 0.35);
  node.declare_parameter<double>("self_filter_min_y", -0.35);
  node.declare_parameter<double>("self_filter_max_y", 0.35);
  node.declare_parameter<double>("self_filter_min_z", -0.20);
  node.declare_parameter<double>("self_filter_max_z", 0.50);

  node.declare_parameter<double>("grid_resolution", 0.05);
  node.declare_parameter<double>("grid_width_m", 10.0);
  node.declare_parameter<double>("grid_height_m", 10.0);
  node.declare_parameter<int>("occupied_value", 100);
  node.declare_parameter<int>("unknown_value", -1);

  node.declare_parameter<int>("obstacle_cell_dilation_radius_cells", 1);

  node.declare_parameter<double>("tf_lookup_timeout_sec", 0.05);

  node.declare_parameter<int>("diagnostics_frame_count_window", 30);
}

ProjectionParameters ProjectionParameters::load(rclcpp::Node & node)
{
  ProjectionParameters params;

  params.input_cloud_topic = node.get_parameter("input_cloud_topic").as_string();
  params.obstacle_cloud_topic = node.get_parameter("obstacle_cloud_topic").as_string();
  params.occupancy_grid_topic = node.get_parameter("occupancy_grid_topic").as_string();
  params.diagnostics_topic = node.get_parameter("diagnostics_topic").as_string();

  params.publish_obstacle_cloud = node.get_parameter("publish_obstacle_cloud").as_bool();
  params.publish_occupancy_grid = node.get_parameter("publish_occupancy_grid").as_bool();
  params.publish_diagnostics = node.get_parameter("publish_diagnostics").as_bool();

  params.target_frame = node.get_parameter("target_frame").as_string();
  params.grid_frame = node.get_parameter("grid_frame").as_string();
  params.robot_frame = node.get_parameter("robot_frame").as_string();

  params.min_range = node.get_parameter("min_range").as_double();
  params.max_range = node.get_parameter("max_range").as_double();

  params.floor_threshold = node.get_parameter("floor_threshold").as_double();
  params.ceiling_threshold = node.get_parameter("ceiling_threshold").as_double();
  params.obstacle_min_z = node.get_parameter("obstacle_min_z").as_double();
  params.obstacle_max_z = node.get_parameter("obstacle_max_z").as_double();

  params.remove_self_points = node.get_parameter("remove_self_points").as_bool();
  params.self_filter_min_x = node.get_parameter("self_filter_min_x").as_double();
  params.self_filter_max_x = node.get_parameter("self_filter_max_x").as_double();
  params.self_filter_min_y = node.get_parameter("self_filter_min_y").as_double();
  params.self_filter_max_y = node.get_parameter("self_filter_max_y").as_double();
  params.self_filter_min_z = node.get_parameter("self_filter_min_z").as_double();
  params.self_filter_max_z = node.get_parameter("self_filter_max_z").as_double();

  params.grid_resolution = node.get_parameter("grid_resolution").as_double();
  params.grid_width_m = node.get_parameter("grid_width_m").as_double();
  params.grid_height_m = node.get_parameter("grid_height_m").as_double();
  params.occupied_value = node.get_parameter("occupied_value").as_int();
  params.unknown_value = node.get_parameter("unknown_value").as_int();

  params.obstacle_cell_dilation_radius_cells =
    node.get_parameter("obstacle_cell_dilation_radius_cells").as_int();

  params.tf_lookup_timeout_sec = node.get_parameter("tf_lookup_timeout_sec").as_double();

  params.diagnostics_frame_count_window =
    node.get_parameter("diagnostics_frame_count_window").as_int();

  params.validate();
  return params;
}

void ProjectionParameters::validate() const
{
  requireNonEmpty(input_cloud_topic, "input_cloud_topic");
  requireNonEmpty(obstacle_cloud_topic, "obstacle_cloud_topic");
  requireNonEmpty(occupancy_grid_topic, "occupancy_grid_topic");
  requireNonEmpty(diagnostics_topic, "diagnostics_topic");

  requireNonEmpty(target_frame, "target_frame");
  requireNonEmpty(grid_frame, "grid_frame");
  requireNonEmpty(robot_frame, "robot_frame");

  if (min_range < 0.0) {
    throw std::runtime_error("min_range must be non-negative");
  }
  requireGreaterThan(max_range, min_range, "max_range", "min_range");

  requireGreaterThan(obstacle_max_z, obstacle_min_z, "obstacle_max_z", "obstacle_min_z");
  requireGreaterThan(ceiling_threshold, floor_threshold, "ceiling_threshold", "floor_threshold");

  if (remove_self_points) {
    requireGreaterThan(
      self_filter_max_x, self_filter_min_x, "self_filter_max_x", "self_filter_min_x");
    requireGreaterThan(
      self_filter_max_y, self_filter_min_y, "self_filter_max_y", "self_filter_min_y");
    requireGreaterThan(
      self_filter_max_z, self_filter_min_z, "self_filter_max_z", "self_filter_min_z");
  }

  if (grid_resolution <= 0.0) {
    throw std::runtime_error("grid_resolution must be greater than zero");
  }
  if (grid_width_m <= 0.0) {
    throw std::runtime_error("grid_width_m must be greater than zero");
  }
  if (grid_height_m <= 0.0) {
    throw std::runtime_error("grid_height_m must be greater than zero");
  }

  requireInOccupancyRange(occupied_value, "occupied_value");
  requireInOccupancyRange(unknown_value, "unknown_value");

  if (obstacle_cell_dilation_radius_cells < 0) {
    throw std::runtime_error("obstacle_cell_dilation_radius_cells must be non-negative");
  }
  if (tf_lookup_timeout_sec < 0.0) {
    throw std::runtime_error("tf_lookup_timeout_sec must be non-negative");
  }
  if (diagnostics_frame_count_window <= 0) {
    throw std::runtime_error("diagnostics_frame_count_window must be greater than zero");
  }
}

void ProjectionParameters::logSummary(const rclcpp::Logger & logger) const
{
  RCLCPP_INFO(logger, "Phase 6 Livox costmap projection parameters:");
  RCLCPP_INFO(logger, "  input_cloud_topic: %s", input_cloud_topic.c_str());
  RCLCPP_INFO(logger, "  obstacle_cloud_topic: %s", obstacle_cloud_topic.c_str());
  RCLCPP_INFO(logger, "  occupancy_grid_topic: %s", occupancy_grid_topic.c_str());
  RCLCPP_INFO(logger, "  diagnostics_topic: %s", diagnostics_topic.c_str());
  RCLCPP_INFO(logger, "  target_frame: %s", target_frame.c_str());
  RCLCPP_INFO(logger, "  grid_frame: %s", grid_frame.c_str());
  RCLCPP_INFO(logger, "  robot_frame: %s", robot_frame.c_str());
  RCLCPP_INFO(logger, "  range: %.2f m to %.2f m", min_range, max_range);
  RCLCPP_INFO(logger, "  obstacle z: %.2f m to %.2f m", obstacle_min_z, obstacle_max_z);
  RCLCPP_INFO(
    logger,
    "  grid: %.2f m x %.2f m at %.3f m/cell",
    grid_width_m,
    grid_height_m,
    grid_resolution);
  RCLCPP_INFO(
    logger,
    "  publish obstacle_cloud=%s occupancy_grid=%s diagnostics=%s",
    publish_obstacle_cloud ? "true" : "false",
    publish_occupancy_grid ? "true" : "false",
    publish_diagnostics ? "true" : "false");
}

}  // namespace custom_livox_costmap_projection
