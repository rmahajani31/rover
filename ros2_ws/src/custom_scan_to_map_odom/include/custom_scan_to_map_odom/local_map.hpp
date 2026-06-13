#pragma once

#include <cstddef>
#include <vector>

#include <Eigen/Core>

#include <pcl/kdtree/kdtree_flann.h>

#include "custom_scan_to_map_odom/ros_conversions.hpp"

namespace custom_scan_to_map_odom
{

class LocalMap
{
public:
  LocalMap();

  void initialize(const CloudTConstPtr& first_scan_map_frame);

  bool isInitialized() const;

  void insertCloud(const CloudTConstPtr& cloud_map_frame);

  void downsample(double leaf_size);

  void cropAround(
    const Eigen::Vector3d& center,
    const Eigen::Vector3d& half_size);

  void rebuildKdTree();

  bool nearestKSearch(
    const Eigen::Vector3d& query,
    int k,
    std::vector<Eigen::Vector3d>& neighbors,
    std::vector<float>& squared_distances) const;

  CloudTConstPtr cloud() const;

  std::size_t size() const;

private:
  CloudTPtr map_cloud_;
  pcl::KdTreeFLANN<PointT>::Ptr kdtree_;

  bool initialized_ = false;
  bool kdtree_ready_ = false;
};

}  // namespace custom_scan_to_map_odom
