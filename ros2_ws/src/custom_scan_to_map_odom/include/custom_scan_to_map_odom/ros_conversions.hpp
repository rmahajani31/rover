#pragma once

#include <memory>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>

namespace custom_scan_to_map_odom
{

using PointT = pcl::PointXYZI;
using CloudT = pcl::PointCloud<PointT>;
using CloudTPtr = CloudT::Ptr;
using CloudTConstPtr = CloudT::ConstPtr;

CloudTPtr fromRosCloud(const sensor_msgs::msg::PointCloud2& msg);

sensor_msgs::msg::PointCloud2 toRosCloud(
  const CloudT& cloud,
  const std_msgs::msg::Header& header);

}  // namespace custom_scan_to_map_odom
