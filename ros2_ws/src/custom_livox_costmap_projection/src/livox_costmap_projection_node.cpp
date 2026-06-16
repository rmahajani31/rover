#include "custom_livox_costmap_projection/livox_costmap_projection_node.hpp"

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/exceptions.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>

namespace custom_livox_costmap_projection
{

LivoxCostmapProjectionNode::LivoxCostmapProjectionNode()
: rclcpp::Node("livox_costmap_projection_node")
{
  ProjectionParameters::declare(*this);
  params_ = ProjectionParameters::load(*this);
  params_.logSummary(get_logger());

  // The listener fills the buffer asynchronously; early clouds may arrive before
  // every transform is available, so lookup failures are reported in diagnostics.
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  diagnostics_ = std::make_unique<ProjectionDiagnostics>(params_);
  grid_builder_ = std::make_unique<OccupancyGridBuilder>(params_);

  if (params_.publish_obstacle_cloud) {
    obstacle_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      params_.obstacle_cloud_topic,
      rclcpp::SensorDataQoS());
  }

  if (params_.publish_occupancy_grid) {
    occupancy_grid_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      params_.occupancy_grid_topic,
      rclcpp::QoS(1).reliable());
  }

  if (params_.publish_diagnostics) {
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      params_.diagnostics_topic,
      rclcpp::QoS(10));
  }

  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    params_.input_cloud_topic,
    rclcpp::SensorDataQoS(),
    std::bind(&LivoxCostmapProjectionNode::cloudCallback, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "Livox costmap projection node started.");
}

void LivoxCostmapProjectionNode::cloudCallback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  const rclcpp::Time start_time = now();

  FilterStats stats;
  TfStatus tf_status;
  tf_status.input_frame = msg->header.frame_id;
  tf_status.target_frame = params_.target_frame;
  tf_status.grid_frame = params_.grid_frame;

  sensor_msgs::msg::PointCloud2 target_cloud;
  if (!transformCloudToTargetFrame(*msg, target_cloud, tf_status)) {
    // Without the cloud transform, any downstream filtering would happen in the
    // wrong frame, so drop the frame and publish diagnostics instead.
    stats.processing_time_ms = elapsedMs(start_time);
    publishDiagnostics(msg->header.stamp, stats, tf_status);
    return;
  }

  pcl::PointCloud<pcl::PointXYZI> obstacle_cloud;
  filterObstacleCloud(target_cloud, obstacle_cloud, stats);

  if (params_.publish_obstacle_cloud) {
    publishObstacleCloud(obstacle_cloud, target_cloud.header);
  }

  if (params_.publish_occupancy_grid) {
    geometry_msgs::msg::TransformStamped grid_from_target_transform;
    // Grid lookup uses the original cloud stamp. This avoids drawing obstacles
    // at the robot's current pose when the cloud came from an older pose.
    if (lookupGridTransform(msg->header.stamp, grid_from_target_transform, tf_status)) {
      std::uint64_t occupied_cell_count = 0U;
      auto grid = grid_builder_->buildGrid(
        obstacle_cloud,
        grid_from_target_transform,
        msg->header.stamp,
        occupied_cell_count);

      stats.grid_occupied_count = occupied_cell_count;
      occupancy_grid_pub_->publish(grid);
    }
  }

  stats.processing_time_ms = elapsedMs(start_time);
  publishDiagnostics(msg->header.stamp, stats, tf_status);
}

bool LivoxCostmapProjectionNode::transformCloudToTargetFrame(
  const sensor_msgs::msg::PointCloud2 & input_cloud,
  sensor_msgs::msg::PointCloud2 & target_cloud,
  TfStatus & tf_status)
{
  if (input_cloud.header.frame_id.empty()) {
    tf_status.cloud_to_target_success = false;
    tf_status.error_message = "Input cloud frame_id is empty";
    return false;
  }

  try {
    // Transform at the cloud timestamp so the points line up with the robot pose
    // that existed when the sensor captured them.
    const auto transform = tf_buffer_->lookupTransform(
      params_.target_frame,
      input_cloud.header.frame_id,
      input_cloud.header.stamp,
      tf2::durationFromSec(params_.tf_lookup_timeout_sec));

    tf2::doTransform(input_cloud, target_cloud, transform);
    target_cloud.header.stamp = input_cloud.header.stamp;
    target_cloud.header.frame_id = params_.target_frame;

    tf_status.cloud_to_target_success = true;
    return true;
  } catch (const tf2::TransformException & ex) {
    tf_status.cloud_to_target_success = false;
    tf_status.error_message = ex.what();

    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "Failed to transform cloud from '%s' to '%s': %s",
      input_cloud.header.frame_id.c_str(),
      params_.target_frame.c_str(),
      ex.what());

    return false;
  }
}

bool LivoxCostmapProjectionNode::lookupGridTransform(
  const rclcpp::Time & stamp,
  geometry_msgs::msg::TransformStamped & grid_from_target_transform,
  TfStatus & tf_status)
{
  try {
    // This gives the robot pose in grid_frame at the same time as the cloud.
    grid_from_target_transform = tf_buffer_->lookupTransform(
      params_.grid_frame,
      params_.target_frame,
      stamp,
      tf2::durationFromSec(params_.tf_lookup_timeout_sec));

    tf_status.target_to_grid_success = true;
    return true;
  } catch (const tf2::TransformException & ex) {
    tf_status.target_to_grid_success = false;

    if (!tf_status.error_message.empty()) {
      tf_status.error_message += "; ";
    }
    tf_status.error_message += ex.what();

    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "Failed to transform from '%s' to '%s' for occupancy grid: %s",
      params_.target_frame.c_str(),
      params_.grid_frame.c_str(),
      ex.what());

    return false;
  }
}

void LivoxCostmapProjectionNode::filterObstacleCloud(
  const sensor_msgs::msg::PointCloud2 & target_cloud,
  pcl::PointCloud<pcl::PointXYZI> & obstacle_cloud,
  FilterStats & stats) const
{
  pcl::PointCloud<pcl::PointXYZI> input_cloud;
  pcl::fromROSMsg(target_cloud, input_cloud);

  obstacle_cloud.clear();
  obstacle_cloud.header = input_cloud.header;
  obstacle_cloud.reserve(input_cloud.size());

  for (const auto & point : input_cloud.points) {
    ++stats.raw_count;

    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      ++stats.rejected_nan;
      continue;
    }

    // Use XY range in base_link so vertical structure does not make nearby
    // points look farther away than they are on the ground plane.
    const double range_xy = std::sqrt(point.x * point.x + point.y * point.y);
    if (range_xy < params_.min_range || range_xy > params_.max_range) {
      ++stats.rejected_range;
      continue;
    }

    if (point.z < params_.obstacle_min_z) {
      ++stats.rejected_low;
      continue;
    }

    if (point.z > params_.obstacle_max_z) {
      ++stats.rejected_high;
      continue;
    }

    // Remove points from the rover body after transforming into target_frame.
    if (params_.remove_self_points && isInsideSelfFilterBox(point)) {
      ++stats.rejected_self;
      continue;
    }

    obstacle_cloud.push_back(point);
    ++stats.kept_count;
  }

  obstacle_cloud.width = static_cast<std::uint32_t>(obstacle_cloud.size());
  obstacle_cloud.height = 1;
  obstacle_cloud.is_dense = false;
}

bool LivoxCostmapProjectionNode::isInsideSelfFilterBox(const pcl::PointXYZI & point) const
{
  return point.x >= params_.self_filter_min_x &&
         point.x <= params_.self_filter_max_x &&
         point.y >= params_.self_filter_min_y &&
         point.y <= params_.self_filter_max_y &&
         point.z >= params_.self_filter_min_z &&
         point.z <= params_.self_filter_max_z;
}

void LivoxCostmapProjectionNode::publishObstacleCloud(
  const pcl::PointCloud<pcl::PointXYZI> & obstacle_cloud,
  const std_msgs::msg::Header & header) const
{
  if (!obstacle_cloud_pub_) {
    return;
  }

  sensor_msgs::msg::PointCloud2 output_msg;
  pcl::toROSMsg(obstacle_cloud, output_msg);
  output_msg.header = header;
  output_msg.header.frame_id = params_.target_frame;

  obstacle_cloud_pub_->publish(output_msg);
}

void LivoxCostmapProjectionNode::publishDiagnostics(
  const rclcpp::Time & stamp,
  const FilterStats & stats,
  const TfStatus & tf_status)
{
  if (!params_.publish_diagnostics || !diagnostics_pub_ || !diagnostics_) {
    return;
  }

  diagnostics_pub_->publish(diagnostics_->buildMessage(stamp, stats, tf_status));
}

double LivoxCostmapProjectionNode::elapsedMs(const rclcpp::Time & start_time) const
{
  const rclcpp::Duration elapsed = now() - start_time;
  return elapsed.seconds() * 1000.0;
}

}  // namespace custom_livox_costmap_projection

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<custom_livox_costmap_projection::LivoxCostmapProjectionNode>());
  rclcpp::shutdown();
  return 0;
}
