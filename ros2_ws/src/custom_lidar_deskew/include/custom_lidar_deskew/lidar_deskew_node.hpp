#pragma once

#include <cstddef>
#include <deque>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Core>

#include <builtin_interfaces/msg/time.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "custom_lidar_deskew/deskew_types.hpp"
#include "custom_lidar_deskew/point_time_utils.hpp"

namespace custom_lidar_deskew
{

class LidarDeskewNode : public rclcpp::Node
{
public:
  LidarDeskewNode();

private:
  void declareParameters();
  void loadParameters();

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  // Copies only the IMU samples needed for the current scan, including a small time margin.
  std::vector<ImuSample> getImuSamples(double scan_start, double scan_end) const;

  bool hasImuCoverage(
    const std::vector<ImuSample>& imu_samples,
    double scan_start,
    double scan_end) const;

  // Current implementation corrects rotational LiDAR motion only; translation support is reserved.
  sensor_msgs::msg::PointCloud2 deskewCloudRotationOnly(
    const sensor_msgs::msg::PointCloud2& cloud,
    const std::vector<ImuSample>& imu_samples,
    double scan_start,
    double scan_end,
    const sensor_msgs::msg::PointField& point_time_field,
    DeskewStats& stats) const;

  void publishRawCloud(
    const sensor_msgs::msg::PointCloud2& cloud,
    DeskewStats& stats) const;

  void publishDiagnostics(
    const DeskewStats& stats,
    const builtin_interfaces::msg::Time& stamp) const;

  static double stampToSec(const builtin_interfaces::msg::Time& stamp);

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::CallbackGroup::SharedPtr imu_callback_group_;
  rclcpp::CallbackGroup::SharedPtr cloud_callback_group_;

  mutable std::mutex imu_mutex_;
  std::deque<ImuSample> imu_buffer_;

  std::string lidar_topic_;
  std::string imu_topic_;
  std::string output_topic_;
  std::string diagnostics_topic_;
  std::string point_time_field_name_;
  std::string point_time_unit_name_;
  std::string output_frame_id_;

  PointTimeUnit point_time_unit_ = PointTimeUnit::Nanoseconds;

  bool deskew_to_scan_end_ = true;
  bool use_rotation_deskew_ = true;
  bool use_translation_deskew_ = false;
  bool allow_fallback_without_point_time_ = false;
  bool publish_raw_if_deskew_fails_ = true;
  bool preserve_input_header_stamp_ = true;
  bool publish_diagnostics_ = true;
  bool print_timing_debug_ = false;

  Eigen::Vector3d gyro_bias_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d accel_bias_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d gravity_ = Eigen::Vector3d(0.0, 0.0, -9.81);

  std::size_t max_imu_buffer_size_ = 5000;
  double imu_time_margin_sec_ = 0.02;
  double max_allowed_scan_duration_sec_ = 0.2;
  int min_required_imu_samples_ = 2;
  double fallback_scan_duration_sec_ = 0.1;
};

}  // namespace custom_lidar_deskew
