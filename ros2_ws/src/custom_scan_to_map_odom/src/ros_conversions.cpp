#include "custom_scan_to_map_odom/ros_conversions.hpp"

#include <pcl_conversions/pcl_conversions.h>

namespace custom_scan_to_map_odom
{

CloudTPtr fromRosCloud(const sensor_msgs::msg::PointCloud2& msg)
{
  auto cloud = std::make_shared<CloudT>();
  // pcl_conversions preserves the XYZ/intensity fields expected by PointT.
  pcl::fromROSMsg(msg, *cloud);
  return cloud;
}

sensor_msgs::msg::PointCloud2 toRosCloud(
  const CloudT& cloud,
  const std_msgs::msg::Header& header)
{
  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(cloud, msg);
  msg.header = header;
  return msg;
}

}  // namespace custom_scan_to_map_odom
