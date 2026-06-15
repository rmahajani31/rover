#pragma once

#include "custom_livox_costmap_projection/occupancy_grid_builder.hpp"
#include "custom_livox_costmap_projection/projection_diagnostics.hpp"
#include "custom_livox_costmap_projection/projection_parameters.hpp"

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>

namespace custom_livox_costmap_projection
{

class LivoxCostmapProjectionNode : public rclcpp::Node
{
public:
  LivoxCostmapProjectionNode();

private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  bool transformCloudToTargetFrame(
    const sensor_msgs::msg::PointCloud2 & input_cloud,
    sensor_msgs::msg::PointCloud2 & target_cloud,
    TfStatus & tf_status) const;

  bool lookupGridTransform(
    const rclcpp::Time & stamp,
    geometry_msgs::msg::TransformStamped & grid_from_target_transform,
    TfStatus & tf_status) const;

  void filterObstacleCloud(
    const sensor_msgs::msg::PointCloud2 & target_cloud,
    pcl::PointCloud<pcl::PointXYZI> & obstacle_cloud,
    FilterStats & stats) const;

  bool isInsideSelfFilterBox(const pcl::PointXYZI & point) const;

  void publishObstacleCloud(
    const pcl::PointCloud<pcl::PointXYZI> & obstacle_cloud,
    const std_msgs::msg::Header & header) const;

  void publishDiagnostics(
    const rclcpp::Time & stamp,
    const FilterStats & stats,
    const TfStatus & tf_status);

  double elapsedMs(const rclcpp::Time & start_time) const;

  ProjectionParameters params_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::unique_ptr<ProjectionDiagnostics> diagnostics_;
  std::unique_ptr<OccupancyGridBuilder> grid_builder_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_cloud_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
};

}  // namespace custom_livox_costmap_projection
