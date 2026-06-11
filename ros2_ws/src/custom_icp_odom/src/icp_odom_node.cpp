#include "custom_icp_odom/icp_odom_node.hpp"
#include "custom_icp_odom/icp_utils.hpp"

#include <functional>
#include <memory>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>

using std::placeholders::_1;

namespace custom_icp_odom
{

IcpOdomNode::IcpOdomNode(const rclcpp::NodeOptions & options)
: Node("custom_icp_odom", options),
  has_previous_scan_(false),
  previous_cloud_(new CloudT),
  global_pose_(Eigen::Matrix4d::Identity())
{
  declareParameters();
  readParameters();

  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    input_topic_,
    rclcpp::SensorDataQoS(),
    std::bind(&IcpOdomNode::cloudCallback, this, _1));

  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);
  path_pub_ = create_publisher<nav_msgs::msg::Path>(path_topic_, 10);

  if (publish_debug_clouds_) {
    aligned_cloud_pub_ =
      create_publisher<sensor_msgs::msg::PointCloud2>(aligned_cloud_topic_, 10);

    source_cloud_pub_ =
      create_publisher<sensor_msgs::msg::PointCloud2>(source_cloud_topic_, 10);

    target_cloud_pub_ =
      create_publisher<sensor_msgs::msg::PointCloud2>(target_cloud_topic_, 10);
  }

  path_msg_.header.frame_id = odom_frame_;

  RCLCPP_INFO(get_logger(), "custom_icp_odom initialized");
}

void IcpOdomNode::declareParameters()
{
  declare_parameter<std::string>("input_topic", "/custom/points_preprocessed");
  declare_parameter<std::string>("odom_topic", "/custom/icp_odom");
  declare_parameter<std::string>("path_topic", "/custom/icp_path");

  declare_parameter<std::string>("odom_frame", "odom");
  declare_parameter<std::string>("base_frame", "base_link");
  declare_parameter<std::string>("lidar_frame", "livox_frame");

  declare_parameter<double>("voxel_leaf_size", 0.20);

  declare_parameter<double>("max_correspondence_distance", 0.75);
  declare_parameter<int>("max_iterations", 20);
  declare_parameter<double>("transformation_epsilon", 1.0e-6);
  declare_parameter<double>("fitness_epsilon", 1.0e-5);

  declare_parameter<int>("min_points_per_scan", 300);

  declare_parameter<bool>("publish_tf", false);

  declare_parameter<bool>("publish_debug_clouds", true);
  declare_parameter<std::string>("aligned_cloud_topic", "/custom/icp/aligned_cloud");
  declare_parameter<std::string>("source_cloud_topic", "/custom/icp/source_cloud");
  declare_parameter<std::string>("target_cloud_topic", "/custom/icp/target_cloud");

  declare_parameter<bool>("use_gicp", false);
}

void IcpOdomNode::readParameters()
{
  input_topic_ = get_parameter("input_topic").as_string();
  odom_topic_ = get_parameter("odom_topic").as_string();
  path_topic_ = get_parameter("path_topic").as_string();

  odom_frame_ = get_parameter("odom_frame").as_string();
  base_frame_ = get_parameter("base_frame").as_string();
  lidar_frame_ = get_parameter("lidar_frame").as_string();

  voxel_leaf_size_ = get_parameter("voxel_leaf_size").as_double();

  max_correspondence_distance_ =
    get_parameter("max_correspondence_distance").as_double();

  max_iterations_ = get_parameter("max_iterations").as_int();

  transformation_epsilon_ =
    get_parameter("transformation_epsilon").as_double();

  fitness_epsilon_ =
    get_parameter("fitness_epsilon").as_double();

  min_points_per_scan_ = get_parameter("min_points_per_scan").as_int();

  publish_tf_ = get_parameter("publish_tf").as_bool();

  publish_debug_clouds_ = get_parameter("publish_debug_clouds").as_bool();

  aligned_cloud_topic_ = get_parameter("aligned_cloud_topic").as_string();
  source_cloud_topic_ = get_parameter("source_cloud_topic").as_string();
  target_cloud_topic_ = get_parameter("target_cloud_topic").as_string();

  use_gicp_ = get_parameter("use_gicp").as_bool();

  if (publish_tf_) {
    RCLCPP_WARN(
      get_logger(),
      "publish_tf is true, but TF publishing is not implemented in this point-to-point ICP version");
  }

  RCLCPP_INFO(
    get_logger(),
    "Registration mode: %s",
    use_gicp_ ? "GICP" : "point-to-point ICP");
}

void IcpOdomNode::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  CloudTPtr raw_cloud = convertRosToPcl(*msg);
  CloudTPtr current_cloud = downsampleCloud(raw_cloud);

  if (static_cast<int>(current_cloud->size()) < min_points_per_scan_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "Skipping scan with too few points: %zu",
      current_cloud->size());
    return;
  }

  if (!has_previous_scan_) {
    previous_cloud_ = current_cloud;
    has_previous_scan_ = true;

    RCLCPP_INFO(
      get_logger(),
      "Stored first scan with %zu points",
      previous_cloud_->size());

    return;
  }

  Eigen::Matrix4d transform_current_to_previous = Eigen::Matrix4d::Identity();
  double fitness_score = 0.0;

  const char * registration_mode = use_gicp_ ? "GICP" : "ICP";

  const bool icp_ok = use_gicp_ ?
    runGicp(
      current_cloud,
      previous_cloud_,
      transform_current_to_previous,
      fitness_score) :
    runPointToPointIcp(
      current_cloud,
      previous_cloud_,
      transform_current_to_previous,
      fitness_score);

  if (!icp_ok) {
    RCLCPP_WARN(
      get_logger(),
      "%s failed to converge | points=%zu | fitness=%.6f",
      registration_mode,
      current_cloud->size(),
      fitness_score);

    previous_cloud_ = current_cloud;
    return;
  }

  const Eigen::Matrix4d transform_previous_to_current =
    transform_current_to_previous.inverse();

  // PCL aligns current -> previous; odometry needs the opposite motion.
  global_pose_ = global_pose_ * transform_previous_to_current;

  publishOdometry(msg->header, global_pose_);

  if (publish_debug_clouds_) {
    CloudTPtr aligned_cloud(new CloudT);
    pcl::transformPointCloud(
      *current_cloud,
      *aligned_cloud,
      transform_current_to_previous.cast<float>());

    publishDebugClouds(
      msg->header,
      current_cloud,
      previous_cloud_,
      aligned_cloud);
  }

  const Eigen::Vector3d delta_translation =
    transform_previous_to_current.block<3, 1>(0, 3);

  const double delta_yaw =
    yawFromRotationMatrix(transform_previous_to_current.block<3, 3>(0, 0));

  RCLCPP_INFO(
    get_logger(),
    "%s ok | points=%zu | fitness=%.6f | dx=%.3f dy=%.3f dz=%.3f dyaw=%.3f",
    registration_mode,
    current_cloud->size(),
    fitness_score,
    delta_translation.x(),
    delta_translation.y(),
    delta_translation.z(),
    delta_yaw);

  previous_cloud_ = current_cloud;
}

IcpOdomNode::CloudTPtr IcpOdomNode::convertRosToPcl(
  const sensor_msgs::msg::PointCloud2 & msg) const
{
  CloudTPtr cloud(new CloudT);
  pcl::fromROSMsg(msg, *cloud);
  return cloud;
}

IcpOdomNode::CloudTPtr IcpOdomNode::downsampleCloud(
  const CloudTPtr & cloud) const
{
  CloudTPtr filtered(new CloudT);

  if (!cloud || cloud->empty()) {
    return filtered;
  }

  pcl::VoxelGrid<PointT> voxel;
  voxel.setInputCloud(cloud);
  voxel.setLeafSize(
    static_cast<float>(voxel_leaf_size_),
    static_cast<float>(voxel_leaf_size_),
    static_cast<float>(voxel_leaf_size_));

  voxel.filter(*filtered);
  return filtered;
}

bool IcpOdomNode::runPointToPointIcp(
  const CloudTPtr & source,
  const CloudTPtr & target,
  Eigen::Matrix4d & transform_source_to_target,
  double & fitness_score)
{
  pcl::IterativeClosestPoint<PointT, PointT> icp;

  icp.setInputSource(source);
  icp.setInputTarget(target);

  icp.setMaxCorrespondenceDistance(max_correspondence_distance_);
  icp.setMaximumIterations(max_iterations_);
  icp.setTransformationEpsilon(transformation_epsilon_);
  icp.setEuclideanFitnessEpsilon(fitness_epsilon_);

  CloudT aligned;
  icp.align(aligned);

  fitness_score = icp.getFitnessScore();

  if (!icp.hasConverged()) {
    return false;
  }

  transform_source_to_target = icp.getFinalTransformation().cast<double>();
  return true;
}

bool IcpOdomNode::runGicp(
  const CloudTPtr & source,
  const CloudTPtr & target,
  Eigen::Matrix4d & transform_source_to_target,
  double & fitness_score)
{
  pcl::GeneralizedIterativeClosestPoint<PointT, PointT> gicp;

  gicp.setInputSource(source);
  gicp.setInputTarget(target);

  gicp.setMaxCorrespondenceDistance(max_correspondence_distance_);
  gicp.setMaximumIterations(max_iterations_);
  gicp.setTransformationEpsilon(transformation_epsilon_);
  gicp.setEuclideanFitnessEpsilon(fitness_epsilon_);

  CloudT aligned;
  gicp.align(aligned);

  fitness_score = gicp.getFitnessScore();

  if (!gicp.hasConverged()) {
    return false;
  }

  transform_source_to_target = gicp.getFinalTransformation().cast<double>();
  return true;
}

void IcpOdomNode::publishOdometry(
  const std_msgs::msg::Header & header,
  const Eigen::Matrix4d & pose)
{
  nav_msgs::msg::Odometry odom;

  odom.header.stamp = header.stamp;
  odom.header.frame_id = odom_frame_;
  odom.child_frame_id = base_frame_;

  odom.pose.pose.position.x = pose(0, 3);
  odom.pose.pose.position.y = pose(1, 3);
  odom.pose.pose.position.z = pose(2, 3);

  const Eigen::Matrix3d rotation = pose.block<3, 3>(0, 0);
  odom.pose.pose.orientation = rotationMatrixToQuaternion(rotation);

  odom.pose.covariance[0] = 0.05;
  odom.pose.covariance[7] = 0.05;
  odom.pose.covariance[14] = 0.05;
  odom.pose.covariance[21] = 0.10;
  odom.pose.covariance[28] = 0.10;
  odom.pose.covariance[35] = 0.10;

  odom_pub_->publish(odom);
  publishPath(header, odom);
}

void IcpOdomNode::publishPath(
  const std_msgs::msg::Header & header,
  const nav_msgs::msg::Odometry & odom)
{
  geometry_msgs::msg::PoseStamped pose_stamped;

  pose_stamped.header.stamp = header.stamp;
  pose_stamped.header.frame_id = odom_frame_;
  pose_stamped.pose = odom.pose.pose;

  path_msg_.header.stamp = header.stamp;
  path_msg_.header.frame_id = odom_frame_;
  path_msg_.poses.push_back(pose_stamped);

  path_pub_->publish(path_msg_);
}

void IcpOdomNode::publishDebugClouds(
  const std_msgs::msg::Header & header,
  const CloudTPtr & source,
  const CloudTPtr & target,
  const CloudTPtr & aligned)
{
  sensor_msgs::msg::PointCloud2 source_msg;
  sensor_msgs::msg::PointCloud2 target_msg;
  sensor_msgs::msg::PointCloud2 aligned_msg;

  pcl::toROSMsg(*source, source_msg);
  pcl::toROSMsg(*target, target_msg);
  pcl::toROSMsg(*aligned, aligned_msg);

  source_msg.header.stamp = header.stamp;
  source_msg.header.frame_id = lidar_frame_;

  target_msg.header.stamp = header.stamp;
  target_msg.header.frame_id = lidar_frame_;

  aligned_msg.header.stamp = header.stamp;
  aligned_msg.header.frame_id = lidar_frame_;

  source_cloud_pub_->publish(source_msg);
  target_cloud_pub_->publish(target_msg);
  aligned_cloud_pub_->publish(aligned_msg);
}

}  // namespace custom_icp_odom

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<custom_icp_odom::IcpOdomNode>());
  rclcpp::shutdown();
  return 0;
}
