#pragma once

#include <memory>
#include <string>

#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/header.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace custom_icp_odom
{

class IcpOdomNode : public rclcpp::Node
{
public:
  explicit IcpOdomNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using PointT = pcl::PointXYZI;
  using CloudT = pcl::PointCloud<PointT>;
  using CloudTPtr = CloudT::Ptr;

  void declareParameters();
  void readParameters();

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  CloudTPtr convertRosToPcl(const sensor_msgs::msg::PointCloud2 & msg) const;
  CloudTPtr downsampleCloud(const CloudTPtr & cloud) const;

  bool runPointToPointIcp(
    const CloudTPtr & source,
    const CloudTPtr & target,
    Eigen::Matrix4d & transform_source_to_target,
    double & fitness_score);

  void publishOdometry(
    const std_msgs::msg::Header & header,
    const Eigen::Matrix4d & pose);

  void publishPath(
    const std_msgs::msg::Header & header,
    const nav_msgs::msg::Odometry & odom);

  void publishDebugClouds(
    const std_msgs::msg::Header & header,
    const CloudTPtr & source,
    const CloudTPtr & target,
    const CloudTPtr & aligned);

  std::string input_topic_;
  std::string odom_topic_;
  std::string path_topic_;

  std::string odom_frame_;
  std::string base_frame_;
  std::string lidar_frame_;

  double voxel_leaf_size_;
  double max_correspondence_distance_;
  int max_iterations_;
  double transformation_epsilon_;
  double fitness_epsilon_;
  int min_points_per_scan_;

  bool publish_tf_;
  bool publish_debug_clouds_;
  bool use_gicp_;

  std::string aligned_cloud_topic_;
  std::string source_cloud_topic_;
  std::string target_cloud_topic_;

  bool has_previous_scan_;
  CloudTPtr previous_cloud_;

  Eigen::Matrix4d global_pose_;

  nav_msgs::msg::Path path_msg_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aligned_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr source_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr target_cloud_pub_;
};

}  // namespace custom_icp_odom
