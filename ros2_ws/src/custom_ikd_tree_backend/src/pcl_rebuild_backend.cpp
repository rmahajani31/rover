#include "custom_ikd_tree_backend/pcl_rebuild_backend.hpp"

#include <cmath>
#include <cstdint>

#include <pcl/filters/voxel_grid.h>

namespace custom_ikd_tree_backend
{

PclRebuildBackend::PclRebuildBackend(double voxel_leaf_size)
: voxel_leaf_size_(voxel_leaf_size),
  cloud_(new CloudT()),
  kdtree_(new pcl::KdTreeFLANN<PointT>())
{
  refreshProfileSizes();
}

void PclRebuildBackend::setVoxelLeafSize(double voxel_leaf_size)
{
  voxel_leaf_size_ = voxel_leaf_size;
}

double PclRebuildBackend::voxelLeafSize() const
{
  return voxel_leaf_size_;
}

const BackendProfileSnapshot& PclRebuildBackend::profileSnapshot() const
{
  return profiler_.snapshot();
}

void PclRebuildBackend::resetProfile()
{
  profiler_.resetFrame();
  refreshProfileSizes();
}

void PclRebuildBackend::clear()
{
  cloud_.reset(new CloudT());
  kdtree_.reset(new pcl::KdTreeFLANN<PointT>());
  kdtree_ready_ = false;
  profiler_.resetFrame();
  refreshProfileSizes();
}

void PclRebuildBackend::buildFromPoints(
  const std::vector<Eigen::Vector3d>& points)
{
  profiler_.resetFrame();
  ScopedTimer total_timer(profiler_.mutableSnapshot().total_backend_time_ms);

  cloud_.reset(new CloudT());

  {
    ScopedTimer insert_timer(profiler_.mutableSnapshot().insert_time_ms);
    appendFinitePoints(points);
  }

  updateCloudLayout();
  rebuildKdTree();
  refreshProfileSizes();
  profiler_.mutableSnapshot().status = "success";
}

void PclRebuildBackend::insertPoints(
  const std::vector<Eigen::Vector3d>& points)
{
  profiler_.resetFrame();
  ScopedTimer total_timer(profiler_.mutableSnapshot().total_backend_time_ms);

  if (points.empty()) {
    refreshProfileSizes();
    profiler_.mutableSnapshot().status = "empty_input";
    return;
  }

  {
    ScopedTimer insert_timer(profiler_.mutableSnapshot().insert_time_ms);
    appendFinitePoints(points);
  }

  updateCloudLayout();
  rebuildKdTree();
  refreshProfileSizes();
  profiler_.mutableSnapshot().status = "success";
}

void PclRebuildBackend::insertPointsWithDownsampling(
  const std::vector<Eigen::Vector3d>& points)
{
  profiler_.resetFrame();
  ScopedTimer total_timer(profiler_.mutableSnapshot().total_backend_time_ms);

  if (points.empty()) {
    refreshProfileSizes();
    profiler_.mutableSnapshot().status = "empty_input";
    return;
  }

  {
    ScopedTimer insert_timer(profiler_.mutableSnapshot().insert_time_ms);
    appendFinitePoints(points);
  }

  updateCloudLayout();
  downsampleMap();
  rebuildKdTree();
  refreshProfileSizes();
  profiler_.mutableSnapshot().status = "success";
}

void PclRebuildBackend::deleteOutsideBox(
  const Eigen::Vector3d& box_min,
  const Eigen::Vector3d& box_max)
{
  profiler_.resetFrame();
  ScopedTimer total_timer(profiler_.mutableSnapshot().total_backend_time_ms);

  if (!isFiniteVector(box_min) || !isFiniteVector(box_max)) {
    refreshProfileSizes();
    profiler_.mutableSnapshot().status = "invalid_box";
    return;
  }

  if (box_min.x() > box_max.x() ||
      box_min.y() > box_max.y() ||
      box_min.z() > box_max.z()) {
    refreshProfileSizes();
    profiler_.mutableSnapshot().status = "invalid_box";
    return;
  }

  CloudTPtr filtered(new CloudT());
  filtered->points.reserve(cloud_->points.size());

  {
    ScopedTimer delete_timer(profiler_.mutableSnapshot().delete_time_ms);

    for (const auto& point : cloud_->points) {
      const Eigen::Vector3d eigen_point = toEigenPoint(point);

      if (eigen_point.x() >= box_min.x() && eigen_point.x() <= box_max.x() &&
          eigen_point.y() >= box_min.y() && eigen_point.y() <= box_max.y() &&
          eigen_point.z() >= box_min.z() && eigen_point.z() <= box_max.z()) {
        filtered->points.push_back(point);
      }
    }
  }

  if (cloud_->points.size() > filtered->points.size()) {
    profiler_.addDeletedPoints(cloud_->points.size() - filtered->points.size());
  }

  cloud_ = filtered;
  updateCloudLayout();
  rebuildKdTree();
  refreshProfileSizes();
  profiler_.mutableSnapshot().status = "success";
}

bool PclRebuildBackend::knnSearch(
  const Eigen::Vector3d& query,
  int k,
  double max_distance,
  std::vector<Eigen::Vector3d>& neighbors) const
{
  neighbors.clear();

  profiler_.addKnnQuery();
  ScopedTimer knn_timer(profiler_.mutableSnapshot().knn_time_ms);

  if (!kdtree_ready_ || !isFiniteVector(query) || k <= 0 || max_distance <= 0.0) {
    return false;
  }

  if (cloud_->points.size() < static_cast<std::size_t>(k)) {
    return false;
  }

  const PointT query_point = toPclPoint(query);

  std::vector<int> indices(static_cast<std::size_t>(k));
  std::vector<float> squared_distances(static_cast<std::size_t>(k));

  const int found = kdtree_->nearestKSearch(
    query_point,
    k,
    indices,
    squared_distances);

  if (found < k) {
    return false;
  }

  const double max_distance_sq = max_distance * max_distance;

  if (static_cast<double>(squared_distances[static_cast<std::size_t>(found - 1)]) >
      max_distance_sq) {
    return false;
  }

  neighbors.reserve(static_cast<std::size_t>(found));

  for (int i = 0; i < found; ++i) {
    neighbors.push_back(
      toEigenPoint(cloud_->points[static_cast<std::size_t>(indices[i])]));
  }

  return true;
}

std::size_t PclRebuildBackend::size() const
{
  return cloud_->points.size();
}

std::size_t PclRebuildBackend::activeSize() const
{
  return cloud_->points.size();
}

void PclRebuildBackend::getAllActivePoints(
  std::vector<Eigen::Vector3d>& points) const
{
  points.clear();
  points.reserve(cloud_->points.size());

  for (const auto& point : cloud_->points) {
    points.push_back(toEigenPoint(point));
  }
}

bool PclRebuildBackend::isFiniteVector(const Eigen::Vector3d& point)
{
  return point.allFinite();
}

PclRebuildBackend::PointT PclRebuildBackend::toPclPoint(
  const Eigen::Vector3d& point)
{
  PointT pcl_point;
  pcl_point.x = static_cast<float>(point.x());
  pcl_point.y = static_cast<float>(point.y());
  pcl_point.z = static_cast<float>(point.z());
  pcl_point.intensity = 0.0F;
  return pcl_point;
}

Eigen::Vector3d PclRebuildBackend::toEigenPoint(const PointT& point)
{
  return Eigen::Vector3d(
    static_cast<double>(point.x),
    static_cast<double>(point.y),
    static_cast<double>(point.z));
}

void PclRebuildBackend::appendFinitePoints(
  const std::vector<Eigen::Vector3d>& points)
{
  cloud_->points.reserve(cloud_->points.size() + points.size());

  for (const auto& point : points) {
    if (isFiniteVector(point)) {
      cloud_->points.push_back(toPclPoint(point));
      profiler_.addInsertedPoints(1);
    }
  }
}

void PclRebuildBackend::rebuildKdTree()
{
  ScopedTimer rebuild_timer(profiler_.mutableSnapshot().rebuild_time_ms);

  if (cloud_->points.empty()) {
    kdtree_ready_ = false;
    return;
  }

  kdtree_->setInputCloud(cloud_);
  kdtree_ready_ = true;
  profiler_.addRebuild();
}

void PclRebuildBackend::downsampleMap()
{
  if (!std::isfinite(voxel_leaf_size_) ||
      voxel_leaf_size_ <= 0.0 ||
      cloud_->points.empty()) {
    return;
  }

  ScopedTimer downsample_timer(profiler_.mutableSnapshot().downsample_time_ms);

  CloudTPtr filtered(new CloudT());

  pcl::VoxelGrid<PointT> voxel;
  voxel.setInputCloud(cloud_);
  voxel.setLeafSize(
    static_cast<float>(voxel_leaf_size_),
    static_cast<float>(voxel_leaf_size_),
    static_cast<float>(voxel_leaf_size_));
  voxel.filter(*filtered);

  cloud_ = filtered;
  updateCloudLayout();
}

void PclRebuildBackend::updateCloudLayout()
{
  cloud_->width = static_cast<std::uint32_t>(cloud_->points.size());
  cloud_->height = 1;
  cloud_->is_dense = false;
}

void PclRebuildBackend::refreshProfileSizes()
{
  profiler_.setMapSizes(
    cloud_->points.size(),
    cloud_->points.size(),
    0);
}

}  // namespace custom_ikd_tree_backend
