#include "custom_fastlio_preprocess/preprocess_node.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <string>

#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

namespace custom_fastlio_preprocess
{

PreprocessNode::PreprocessNode()
: Node("custom_fastlio_preprocess"), frame_count_(0)
{
  declareParameters();
  loadParameters();

  if (use_tf_transform_) {
    RCLCPP_WARN(
      get_logger(),
      "use_tf_transform is true, but TF transform support is not implemented in this no-TF phase.");
  }

  odom_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    odom_output_topic_, rclcpp::SensorDataQoS());

  nav2_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    nav2_output_topic_, rclcpp::SensorDataQoS());

  diag_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    diagnostics_topic_, 10);

  // Subscribe directly to the FAST-LIO2-compatible Livox stream instead of the
  // PointCloud2 convenience copy used by the older scan projection path.
  custom_cloud_sub_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(
    input_topic_,
    rclcpp::SensorDataQoS(),
    std::bind(&PreprocessNode::customCloudCallback, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "custom_fastlio_preprocess started");
  RCLCPP_INFO(get_logger(), "Input CustomMsg topic: %s", input_topic_.c_str());
  RCLCPP_INFO(get_logger(), "Odom cloud output: %s", odom_output_topic_.c_str());
  RCLCPP_INFO(get_logger(), "Nav2 cloud output: %s", nav2_output_topic_.c_str());
}

void PreprocessNode::declareParameters()
{
  declare_parameter<std::string>("input_topic", "/livox/lidar");
  declare_parameter<std::string>("odom_output_topic", "/custom/points_preprocessed");
  declare_parameter<std::string>("nav2_output_topic", "/custom/points_for_nav2");
  declare_parameter<std::string>("diagnostics_topic", "/custom/preprocess_diagnostics");
  declare_parameter<std::string>("target_frame", "base_link");
  declare_parameter<std::string>("fallback_frame_id", "livox_frame");

  declare_parameter<bool>("use_tf_transform", false);
  declare_parameter<bool>("remove_nan", true);
  declare_parameter<bool>("publish_diagnostics", true);
  declare_parameter<bool>("filter_livox_line", true);
  declare_parameter<bool>("filter_livox_tag", false);

  declare_parameter<double>("min_range", 0.30);
  declare_parameter<double>("max_range", 30.0);

  declare_parameter<double>("odom_voxel_leaf_size", 0.15);
  declare_parameter<bool>("odom_keep_floor", true);
  declare_parameter<bool>("odom_keep_ceiling", true);

  declare_parameter<double>("nav2_min_height", -0.24);
  declare_parameter<double>("nav2_max_height", -0.02);
  declare_parameter<double>("nav2_max_range", 12.0);
  declare_parameter<double>("nav2_voxel_leaf_size", 0.05);

  declare_parameter<int>("scan_line_count", 4);
  declare_parameter<int>("print_debug_every_n_frames", 30);
}

void PreprocessNode::loadParameters()
{
  input_topic_ = get_parameter("input_topic").as_string();
  odom_output_topic_ = get_parameter("odom_output_topic").as_string();
  nav2_output_topic_ = get_parameter("nav2_output_topic").as_string();
  diagnostics_topic_ = get_parameter("diagnostics_topic").as_string();
  target_frame_ = get_parameter("target_frame").as_string();
  fallback_frame_id_ = get_parameter("fallback_frame_id").as_string();

  use_tf_transform_ = get_parameter("use_tf_transform").as_bool();
  remove_nan_ = get_parameter("remove_nan").as_bool();
  publish_diagnostics_ = get_parameter("publish_diagnostics").as_bool();
  filter_livox_line_ = get_parameter("filter_livox_line").as_bool();
  filter_livox_tag_ = get_parameter("filter_livox_tag").as_bool();

  min_range_ = get_parameter("min_range").as_double();
  max_range_ = get_parameter("max_range").as_double();

  odom_voxel_leaf_size_ = get_parameter("odom_voxel_leaf_size").as_double();
  odom_keep_floor_ = get_parameter("odom_keep_floor").as_bool();
  odom_keep_ceiling_ = get_parameter("odom_keep_ceiling").as_bool();

  nav2_min_height_ = get_parameter("nav2_min_height").as_double();
  nav2_max_height_ = get_parameter("nav2_max_height").as_double();
  nav2_max_range_ = get_parameter("nav2_max_range").as_double();
  nav2_voxel_leaf_size_ = get_parameter("nav2_voxel_leaf_size").as_double();

  scan_line_count_ = get_parameter("scan_line_count").as_int();
  print_debug_every_n_frames_ = get_parameter("print_debug_every_n_frames").as_int();
}

bool PreprocessNode::isFinitePoint(const PointT & point) const
{
  return std::isfinite(point.x) &&
         std::isfinite(point.y) &&
         std::isfinite(point.z);
}

double PreprocessNode::range3D(const PointT & point) const
{
  return std::sqrt(
    static_cast<double>(point.x) * point.x +
    static_cast<double>(point.y) * point.y +
    static_cast<double>(point.z) * point.z);
}

void PreprocessNode::convertLivoxCustomMsg(
  const livox_ros_driver2::msg::CustomMsg & msg,
  CloudT::Ptr & output,
  std::size_t & rejected_line,
  std::size_t & rejected_tag) const
{
  output->clear();
  output->reserve(msg.points.size());

  rejected_line = 0;
  rejected_tag = 0;

  for (const auto & livox_point : msg.points) {
    // MID-360 is configured as a 4-line Livox sensor in the FAST-LIO2 params.
    if (filter_livox_line_ && livox_point.line >= scan_line_count_) {
      ++rejected_line;
      continue;
    }

    // Bits 4-5 encode the Livox return number. This conservative filter is
    // disabled by default so Phase 3 does not drop valid obstacle returns.
    const auto return_tag = livox_point.tag & 0x30;
    if (filter_livox_tag_ && return_tag != 0x10 && return_tag != 0x00) {
      ++rejected_tag;
      continue;
    }

    PointT point;
    point.x = livox_point.x;
    point.y = livox_point.y;
    point.z = livox_point.z;
    point.intensity = static_cast<float>(livox_point.reflectivity);
    output->push_back(point);
  }

  output->width = static_cast<std::uint32_t>(output->size());
  output->height = 1;
  output->is_dense = false;
}

void PreprocessNode::filterCommon(
  const CloudT::Ptr & input,
  CloudT::Ptr & output,
  std::size_t & rejected_nan,
  std::size_t & rejected_range) const
{
  output->clear();
  output->reserve(input->size());

  rejected_nan = 0;
  rejected_range = 0;

  for (const auto & point : input->points) {
    if (remove_nan_ && !isFinitePoint(point)) {
      ++rejected_nan;
      continue;
    }

    const double range = range3D(point);
    if (range < min_range_ || range > max_range_) {
      ++rejected_range;
      continue;
    }

    output->push_back(point);
  }

  output->width = static_cast<std::uint32_t>(output->size());
  output->height = 1;
  output->is_dense = false;
}

void PreprocessNode::voxelDownsample(
  const CloudT::Ptr & input,
  CloudT::Ptr & output,
  double leaf_size) const
{
  output->clear();

  if (input->empty()) {
    output->width = 0;
    output->height = 1;
    output->is_dense = false;
    return;
  }

  if (leaf_size <= 0.0) {
    *output = *input;
    return;
  }

  pcl::VoxelGrid<PointT> voxel;
  voxel.setInputCloud(input);
  voxel.setLeafSize(
    static_cast<float>(leaf_size),
    static_cast<float>(leaf_size),
    static_cast<float>(leaf_size));
  voxel.filter(*output);

  output->width = static_cast<std::uint32_t>(output->size());
  output->height = 1;
  output->is_dense = false;
}

void PreprocessNode::makeOdomCloud(
  const CloudT::Ptr & common_filtered,
  CloudT::Ptr & odom_cloud) const
{
  // The odometry stream keeps broad scene structure for future ICP experiments.
  voxelDownsample(common_filtered, odom_cloud, odom_voxel_leaf_size_);
}

void PreprocessNode::makeNav2Cloud(
  const CloudT::Ptr & common_filtered,
  CloudT::Ptr & nav2_cloud,
  std::size_t & rejected_height) const
{
  auto height_filtered = std::make_shared<CloudT>();
  height_filtered->reserve(common_filtered->size());

  rejected_height = 0;

  for (const auto & point : common_filtered->points) {
    // Height is still evaluated in the incoming Livox frame until TF support is
    // added to this node.
    const double range = range3D(point);
    if (range > nav2_max_range_) {
      ++rejected_height;
      continue;
    }

    if (point.z < nav2_min_height_ || point.z > nav2_max_height_) {
      ++rejected_height;
      continue;
    }

    height_filtered->push_back(point);
  }

  height_filtered->width = static_cast<std::uint32_t>(height_filtered->size());
  height_filtered->height = 1;
  height_filtered->is_dense = false;

  voxelDownsample(height_filtered, nav2_cloud, nav2_voxel_leaf_size_);
}

void PreprocessNode::publishDiagnostics(
  const std_msgs::msg::Header & header,
  std::size_t raw_count,
  std::size_t converted_count,
  std::size_t common_count,
  std::size_t odom_count,
  std::size_t nav2_count,
  std::size_t rejected_line,
  std::size_t rejected_tag,
  std::size_t rejected_nan,
  std::size_t rejected_range,
  std::size_t rejected_height,
  double processing_time_ms,
  bool has_point_time)
{
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header = header;

  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "custom_fastlio_preprocess";
  status.hardware_id = "livox_mid360";
  status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  status.message = "preprocess ok";

  auto add_key_value = [&status](const std::string & key, const std::string & value) {
    diagnostic_msgs::msg::KeyValue pair;
    pair.key = key;
    pair.value = value;
    status.values.push_back(pair);
  };

  add_key_value("point_count_raw", std::to_string(raw_count));
  add_key_value("point_count_converted", std::to_string(converted_count));
  add_key_value("point_count_common_filtered", std::to_string(common_count));
  add_key_value("point_count_odom", std::to_string(odom_count));
  add_key_value("point_count_nav2", std::to_string(nav2_count));
  add_key_value("rejected_line", std::to_string(rejected_line));
  add_key_value("rejected_tag", std::to_string(rejected_tag));
  add_key_value("rejected_nan", std::to_string(rejected_nan));
  add_key_value("rejected_range", std::to_string(rejected_range));
  add_key_value("rejected_height", std::to_string(rejected_height));
  add_key_value("processing_time_ms", std::to_string(processing_time_ms));
  add_key_value("has_point_time", has_point_time ? "true" : "false");

  array.status.push_back(status);
  diag_pub_->publish(array);
}

void PreprocessNode::customCloudCallback(
  const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
{
  const auto start_time = std::chrono::steady_clock::now();
  ++frame_count_;

  auto converted_cloud = std::make_shared<CloudT>();
  auto common_filtered = std::make_shared<CloudT>();
  auto odom_cloud = std::make_shared<CloudT>();
  auto nav2_cloud = std::make_shared<CloudT>();

  std::size_t rejected_line = 0;
  std::size_t rejected_tag = 0;
  std::size_t rejected_nan = 0;
  std::size_t rejected_range = 0;
  std::size_t rejected_height = 0;

  // Processing is staged so diagnostics can show exactly where points are lost.
  convertLivoxCustomMsg(*msg, converted_cloud, rejected_line, rejected_tag);
  filterCommon(converted_cloud, common_filtered, rejected_nan, rejected_range);
  makeOdomCloud(common_filtered, odom_cloud);
  makeNav2Cloud(common_filtered, nav2_cloud, rejected_height);

  std_msgs::msg::Header output_header = msg->header;
  if (output_header.frame_id.empty()) {
    output_header.frame_id = fallback_frame_id_;
  }

  sensor_msgs::msg::PointCloud2 odom_msg;
  pcl::toROSMsg(*odom_cloud, odom_msg);
  odom_msg.header = output_header;

  sensor_msgs::msg::PointCloud2 nav2_msg;
  pcl::toROSMsg(*nav2_cloud, nav2_msg);
  nav2_msg.header = output_header;

  odom_pub_->publish(odom_msg);
  nav2_pub_->publish(nav2_msg);

  const auto end_time = std::chrono::steady_clock::now();
  const double processing_time_ms =
    std::chrono::duration<double, std::milli>(end_time - start_time).count();

  // Livox CustomMsg carries offset_time per point; Phase 3 reports its presence
  // for future deskewing work even though this node does not use it yet.
  constexpr bool has_point_time = true;

  if (publish_diagnostics_) {
    publishDiagnostics(
      output_header,
      msg->points.size(),
      converted_cloud->size(),
      common_filtered->size(),
      odom_cloud->size(),
      nav2_cloud->size(),
      rejected_line,
      rejected_tag,
      rejected_nan,
      rejected_range,
      rejected_height,
      processing_time_ms,
      has_point_time);
  }

  if (print_debug_every_n_frames_ > 0 &&
      frame_count_ % static_cast<std::size_t>(print_debug_every_n_frames_) == 0) {
    RCLCPP_INFO(
      get_logger(),
      "raw=%zu converted=%zu common=%zu odom=%zu nav2=%zu time=%.2f ms",
      msg->points.size(),
      converted_cloud->size(),
      common_filtered->size(),
      odom_cloud->size(),
      nav2_cloud->size(),
      processing_time_ms);
  }
}

}  // namespace custom_fastlio_preprocess

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<custom_fastlio_preprocess::PreprocessNode>());
  rclcpp::shutdown();
  return 0;
}
