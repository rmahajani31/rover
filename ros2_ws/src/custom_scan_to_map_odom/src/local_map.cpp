#include "custom_scan_to_map_odom/local_map.hpp"

#include <cstdint>

#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>

namespace custom_scan_to_map_odom
{

LocalMap::LocalMap()
: map_cloud_(new CloudT()),
  kdtree_(new pcl::KdTreeFLANN<PointT>())
{
}

void LocalMap::initialize(const CloudTConstPtr& first_scan_map_frame)
{
  map_cloud_.reset(new CloudT());

  if (first_scan_map_frame) {
    *map_cloud_ = *first_scan_map_frame;
  }

  map_cloud_->width = static_cast<std::uint32_t>(map_cloud_->points.size());
  map_cloud_->height = 1;
  map_cloud_->is_dense = false;

  initialized_ = true;
  kdtree_ready_ = false;
}

bool LocalMap::isInitialized() const
{
  return initialized_;
}

void LocalMap::insertCloud(const CloudTConstPtr& cloud_map_frame)
{
  if (!cloud_map_frame || cloud_map_frame->empty()) {
    return;
  }

  *map_cloud_ += *cloud_map_frame;
  map_cloud_->width = static_cast<std::uint32_t>(map_cloud_->points.size());
  map_cloud_->height = 1;
  map_cloud_->is_dense = false;

  kdtree_ready_ = false;
}

void LocalMap::downsample(double leaf_size)
{
  if (leaf_size <= 0.0 || map_cloud_->empty()) {
    return;
  }

  CloudTPtr filtered(new CloudT());

  pcl::VoxelGrid<PointT> voxel;
  voxel.setInputCloud(map_cloud_);
  voxel.setLeafSize(
    static_cast<float>(leaf_size),
    static_cast<float>(leaf_size),
    static_cast<float>(leaf_size));
  voxel.filter(*filtered);

  map_cloud_ = filtered;
  map_cloud_->width = static_cast<std::uint32_t>(map_cloud_->points.size());
  map_cloud_->height = 1;
  map_cloud_->is_dense = false;

  kdtree_ready_ = false;
}

void LocalMap::cropAround(
  const Eigen::Vector3d& center,
  const Eigen::Vector3d& half_size)
{
  if (map_cloud_->empty() || !center.allFinite() || !half_size.allFinite()) {
    return;
  }

  if (half_size.x() <= 0.0 || half_size.y() <= 0.0 || half_size.z() <= 0.0) {
    return;
  }

  const Eigen::Vector3d min = center - half_size;
  const Eigen::Vector3d max = center + half_size;

  CloudTPtr cropped(new CloudT());

  pcl::CropBox<PointT> crop;
  crop.setInputCloud(map_cloud_);
  crop.setMin(Eigen::Vector4f(
    static_cast<float>(min.x()),
    static_cast<float>(min.y()),
    static_cast<float>(min.z()),
    1.0F));
  crop.setMax(Eigen::Vector4f(
    static_cast<float>(max.x()),
    static_cast<float>(max.y()),
    static_cast<float>(max.z()),
    1.0F));
  crop.filter(*cropped);

  map_cloud_ = cropped;
  map_cloud_->width = static_cast<std::uint32_t>(map_cloud_->points.size());
  map_cloud_->height = 1;
  map_cloud_->is_dense = false;

  kdtree_ready_ = false;
}

void LocalMap::rebuildKdTree()
{
  if (map_cloud_->empty()) {
    kdtree_ready_ = false;
    return;
  }

  kdtree_->setInputCloud(map_cloud_);
  kdtree_ready_ = true;
}

bool LocalMap::nearestKSearch(
  const Eigen::Vector3d& query,
  int k,
  std::vector<Eigen::Vector3d>& neighbors,
  std::vector<float>& squared_distances) const
{
  neighbors.clear();
  squared_distances.clear();

  if (!kdtree_ready_ || !query.allFinite() || k <= 0) {
    return false;
  }

  if (map_cloud_->size() < static_cast<std::size_t>(k)) {
    return false;
  }

  PointT query_point;
  query_point.x = static_cast<float>(query.x());
  query_point.y = static_cast<float>(query.y());
  query_point.z = static_cast<float>(query.z());
  query_point.intensity = 0.0F;

  std::vector<int> indices(static_cast<std::size_t>(k));
  std::vector<float> distances(static_cast<std::size_t>(k));

  const int found = kdtree_->nearestKSearch(query_point, k, indices, distances);

  if (found < k) {
    return false;
  }

  neighbors.reserve(static_cast<std::size_t>(found));
  squared_distances.reserve(static_cast<std::size_t>(found));

  for (int i = 0; i < found; ++i) {
    const auto& point = map_cloud_->points[static_cast<std::size_t>(indices[i])];

    neighbors.emplace_back(
      static_cast<double>(point.x),
      static_cast<double>(point.y),
      static_cast<double>(point.z));
    squared_distances.push_back(distances[static_cast<std::size_t>(i)]);
  }

  return true;
}

CloudTConstPtr LocalMap::cloud() const
{
  return map_cloud_;
}

std::size_t LocalMap::size() const
{
  return map_cloud_->size();
}

}  // namespace custom_scan_to_map_odom
