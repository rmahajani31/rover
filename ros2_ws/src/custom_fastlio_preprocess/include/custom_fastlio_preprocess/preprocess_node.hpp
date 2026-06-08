#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace custom_fastlio_preprocess
{

class PreprocessNode : public rclcpp::Node
{
public:
  PreprocessNode();

private:
  using PointT = pcl::PointXYZI;
  using CloudT = pcl::PointCloud<PointT>;

  void customCloudCallback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg);

  void declareParameters();
  void loadParameters();

  bool isFinitePoint(const PointT & point) const;
  double range3D(const PointT & point) const;

  void convertLivoxCustomMsg(
    const livox_ros_driver2::msg::CustomMsg & msg,
    CloudT::Ptr & output,
    std::size_t & rejected_line,
    std::size_t & rejected_tag) const;

  void filterCommon(
    const CloudT::Ptr & input,
    CloudT::Ptr & output,
    std::size_t & rejected_nan,
    std::size_t & rejected_range) const;

  void makeOdomCloud(
    const CloudT::Ptr & common_filtered,
    CloudT::Ptr & odom_cloud) const;

  void makeNav2Cloud(
    const CloudT::Ptr & common_filtered,
    CloudT::Ptr & nav2_cloud,
    std::size_t & rejected_height) const;

  void voxelDownsample(
    const CloudT::Ptr & input,
    CloudT::Ptr & output,
    double leaf_size) const;

  void publishDiagnostics(
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
    bool has_point_time);

  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr custom_cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr nav2_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;

  std::string input_topic_;
  std::string odom_output_topic_;
  std::string nav2_output_topic_;
  std::string diagnostics_topic_;
  std::string target_frame_;
  std::string fallback_frame_id_;

  bool use_tf_transform_;
  bool remove_nan_;
  bool publish_diagnostics_;
  bool filter_livox_line_;
  bool filter_livox_tag_;

  double min_range_;
  double max_range_;

  double odom_voxel_leaf_size_;
  bool odom_keep_floor_;
  bool odom_keep_ceiling_;

  double nav2_min_height_;
  double nav2_max_height_;
  double nav2_max_range_;
  double nav2_voxel_leaf_size_;

  int scan_line_count_;
  int print_debug_every_n_frames_;
  std::size_t frame_count_;
};

}  // namespace custom_fastlio_preprocess
