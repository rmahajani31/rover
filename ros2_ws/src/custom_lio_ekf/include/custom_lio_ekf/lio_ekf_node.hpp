#pragma once

#include <cstddef>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include "custom_imu_propagator/imu_sample.hpp"
#include "custom_lio_ekf/covariance_propagation.hpp"
#include "custom_lio_ekf/diagnostics.hpp"
#include "custom_lio_ekf/ekf_parameters.hpp"
#include "custom_lio_ekf/ekf_state.hpp"
#include "custom_lio_ekf/iterated_lidar_update.hpp"
#include "custom_lio_ekf/lidar_residual.hpp"
#include "custom_scan_to_map_odom/local_map_config.hpp"
#include "custom_scan_to_map_odom/local_map_manager.hpp"
#include "custom_scan_to_map_odom/ros_conversions.hpp"

namespace custom_lio_ekf
{

class LioEkfNode : public rclcpp::Node
{
public:
  explicit LioEkfNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  void declareParameters();
  void readParameters();

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  custom_scan_to_map_odom::CloudTPtr filterScan(
    const custom_scan_to_map_odom::CloudTConstPtr& cloud) const;

  custom_scan_to_map_odom::CloudTPtr transformCloudToWorld(
    const custom_scan_to_map_odom::CloudTConstPtr& cloud,
    const EkfState& state) const;

  bool getImuSamplesForScan(
    const rclcpp::Time& scan_stamp,
    std::vector<custom_imu_propagator::ImuSample>& samples);

  void trimImuBuffer(const rclcpp::Time& newest_stamp);

  bool initializeMap(
    const std_msgs::msg::Header& header,
    const custom_scan_to_map_odom::CloudTConstPtr& filtered_scan,
    LioEkfDiagnostics& diagnostics);

  bool calibrateInitialImuState(const rclcpp::Time& scan_stamp);

  bool runPrediction(
    const rclcpp::Time& scan_stamp,
    EkfPredictionStats& prediction_stats);

  bool runLidarUpdate(
    const custom_scan_to_map_odom::CloudTConstPtr& filtered_scan,
    LidarUpdateStats& update_stats);

  void updateMap(
    const custom_scan_to_map_odom::CloudTConstPtr& filtered_scan,
    LioEkfDiagnostics& diagnostics);

  bool lookupImuToBaseTransform(
    const rclcpp::Time& stamp,
    Eigen::Isometry3d& T_imu_base);

  void ensureTfListener();
  void ensureTfBroadcaster();
  void publishLatestTransform();
  void publishPredictedOdometry();
  void updatePredictionBaseState(const rclcpp::Time& stamp);
  bool getPredictionBaseState(EkfState& state, rclcpp::Time& stamp);

  bool publishOdometry(
    const std_msgs::msg::Header& header,
    const LidarUpdateStats& update_stats);

  bool publishOdometryForState(
    const std_msgs::msg::Header& header,
    const LidarUpdateStats& update_stats,
    const EkfState& state,
    bool publish_path);

  void publishPath(
    const std_msgs::msg::Header& header,
    const nav_msgs::msg::Odometry& odom);

  void publishLocalMap(const std_msgs::msg::Header& header);
  bool shouldPublishLocalMap(const std_msgs::msg::Header& header) const;

  void publishDiagnostics(
    const std_msgs::msg::Header& header,
    const LioEkfDiagnostics& diagnostics);

  void publishTransform(
    const rclcpp::Time& stamp,
    const Eigen::Isometry3d& T_odom_child,
    const std::string& child_frame);

  Eigen::Vector3d readVector3Parameter(
    const std::string& name,
    const Eigen::Vector3d& fallback) const;

  EkfParameters parametersFromRosParameters() const;
  custom_scan_to_map_odom::LocalMapConfig localMapConfigFromParameters() const;

  std::string input_topic_;
  std::string imu_topic_;
  std::string odom_topic_;
  std::string path_topic_;
  std::string local_map_topic_;
  std::string diagnostics_topic_;

  std::string odom_frame_;
  std::string base_frame_;
  std::string imu_frame_;
  std::string lidar_frame_;

  bool publish_tf_ = false;
  bool publish_path_ = true;
  bool publish_diagnostics_ = true;
  bool stop_tf_on_tracking_degraded_ = true;
  bool publish_predicted_odom_ = true;

  double tf_publish_rate_hz_ = 20.0;
  double predicted_odom_rate_hz_ = 10.0;
  double max_predicted_odom_interval_sec_ = 1.5;
  int max_consecutive_tracking_failures_ = 3;
  int max_path_poses_ = 2000;

  double scan_voxel_leaf_size_ = 0.20;
  double min_range_ = 0.30;
  double max_range_ = 30.0;
  int max_points_per_scan_ = 3000;

  double max_imu_buffer_seconds_ = 5.0;
  double imu_accel_scale_ = 9.80665;
  bool calibrate_initial_imu_ = true;
  int initial_imu_calibration_min_samples_ = 20;
  double initial_imu_calibration_window_sec_ = 1.0;
  bool initial_imu_calibrated_ = false;

  EkfParameters ekf_parameters_;
  LidarImuExtrinsics lidar_imu_extrinsics_;
  custom_scan_to_map_odom::LocalMapConfig local_map_config_;

  EkfState state_;
  EkfState prediction_base_state_;
  custom_scan_to_map_odom::LocalMapManager local_map_manager_;

  std::deque<custom_imu_propagator::ImuSample> imu_buffer_;
  std::mutex imu_mutex_;

  bool has_last_scan_stamp_ = false;
  rclcpp::Time last_scan_stamp_{0, 0, RCL_ROS_TIME};

  std::size_t frame_count_ = 0;
  std::size_t imu_samples_received_ = 0;
  int consecutive_tracking_failures_ = 0;

  nav_msgs::msg::Path path_msg_;

  Eigen::Isometry3d previous_odom_pose_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d latest_T_odom_child_ = Eigen::Isometry3d::Identity();
  std::string previous_odom_child_frame_;
  std::string latest_tf_child_frame_;

  bool has_previous_odom_ = false;
  bool has_latest_tf_ = false;
  bool has_prediction_base_state_ = false;
  bool has_last_local_map_publish_stamp_ = false;

  rclcpp::Time previous_odom_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time prediction_base_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_local_map_publish_stamp_{0, 0, RCL_ROS_TIME};

  std::mutex odom_publish_mutex_;
  std::mutex prediction_base_mutex_;
  std::mutex latest_tf_mutex_;

  rclcpp::CallbackGroup::SharedPtr cloud_callback_group_;
  rclcpp::CallbackGroup::SharedPtr imu_callback_group_;
  rclcpp::CallbackGroup::SharedPtr tf_callback_group_;
  rclcpp::CallbackGroup::SharedPtr predicted_odom_callback_group_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr tf_timer_;
  rclcpp::TimerBase::SharedPtr predicted_odom_timer_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr local_map_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
};

}  // namespace custom_lio_ekf
