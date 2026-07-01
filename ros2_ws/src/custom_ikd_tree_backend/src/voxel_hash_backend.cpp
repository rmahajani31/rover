#include "custom_ikd_tree_backend/voxel_hash_backend.hpp"

#include <cstdint>

namespace custom_ikd_tree_backend
{

VoxelHashBackend::VoxelHashBackend(double voxel_size)
: voxel_size_(voxel_size),
  cloud_(new CloudT()),
  kdtree_(new pcl::KdTreeFLANN<PointT>())
{
  refreshProfileSizes();
}

void VoxelHashBackend::setVoxelSize(double voxel_size)
{
  if (!isValidVoxelSize(voxel_size)) {
    return;
  }

  voxel_size_ = voxel_size;

  std::vector<Eigen::Vector3d> points;
  getAllActivePoints(points);
  buildFromPoints(points);
}

double VoxelHashBackend::voxelSize() const
{
  return voxel_size_;
}

const BackendProfileSnapshot& VoxelHashBackend::profileSnapshot() const
{
  return profiler_.snapshot();
}

void VoxelHashBackend::resetProfile()
{
  profiler_.resetFrame();
  refreshProfileSizes();
}

void VoxelHashBackend::clear()
{
  voxel_representatives_.clear();
  cloud_.reset(new CloudT());
  kdtree_.reset(new pcl::KdTreeFLANN<PointT>());
  kdtree_ready_ = false;
  profiler_.resetFrame();
  refreshProfileSizes();
}

void VoxelHashBackend::buildFromPoints(
  const std::vector<Eigen::Vector3d>& points)
{
  profiler_.resetFrame();
  ScopedTimer total_timer(profiler_.mutableSnapshot().total_backend_time_ms);

  voxel_representatives_.clear();

  {
    ScopedTimer insert_timer(profiler_.mutableSnapshot().insert_time_ms);

    for (const auto& point : points) {
      insertPointWithVoxelRule(point);
    }
  }

  rebuildCloudAndKdTree();
  refreshProfileSizes();
  profiler_.mutableSnapshot().status = "success";
}

void VoxelHashBackend::insertPoints(
  const std::vector<Eigen::Vector3d>& points)
{
  insertPointsWithDownsampling(points);
}

void VoxelHashBackend::insertPointsWithDownsampling(
  const std::vector<Eigen::Vector3d>& points)
{
  profiler_.resetFrame();
  ScopedTimer total_timer(profiler_.mutableSnapshot().total_backend_time_ms);

  {
    ScopedTimer insert_timer(profiler_.mutableSnapshot().insert_time_ms);

    for (const auto& point : points) {
      insertPointWithVoxelRule(point);
    }
  }

  rebuildCloudAndKdTree();
  refreshProfileSizes();
  profiler_.mutableSnapshot().status = "success";
}

void VoxelHashBackend::deleteOutsideBox(
  const Eigen::Vector3d& box_min,
  const Eigen::Vector3d& box_max)
{
  profiler_.resetFrame();
  ScopedTimer total_timer(profiler_.mutableSnapshot().total_backend_time_ms);

  if (!isValidBox(box_min, box_max)) {
    refreshProfileSizes();
    profiler_.mutableSnapshot().status = "invalid_box";
    return;
  }

  std::size_t deleted_count = 0;

  {
    ScopedTimer delete_timer(profiler_.mutableSnapshot().delete_time_ms);

    for (auto it = voxel_representatives_.begin();
         it != voxel_representatives_.end();) {
      const Eigen::Vector3d& point = it->second;

      const bool inside =
        point.x() >= box_min.x() && point.x() <= box_max.x() &&
        point.y() >= box_min.y() && point.y() <= box_max.y() &&
        point.z() >= box_min.z() && point.z() <= box_max.z();

      if (inside) {
        ++it;
      } else {
        it = voxel_representatives_.erase(it);
        ++deleted_count;
      }
    }
  }

  profiler_.addDeletedPoints(deleted_count);
  rebuildCloudAndKdTree();
  refreshProfileSizes();
  profiler_.mutableSnapshot().status = "success";
}

bool VoxelHashBackend::knnSearch(
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

std::size_t VoxelHashBackend::size() const
{
  return voxel_representatives_.size();
}

std::size_t VoxelHashBackend::activeSize() const
{
  return voxel_representatives_.size();
}

void VoxelHashBackend::getAllActivePoints(
  std::vector<Eigen::Vector3d>& points) const
{
  points.clear();
  points.reserve(voxel_representatives_.size());

  for (const auto& item : voxel_representatives_) {
    points.push_back(item.second);
  }
}

bool VoxelHashBackend::isFiniteVector(const Eigen::Vector3d& point)
{
  return point.allFinite();
}

bool VoxelHashBackend::isValidBox(
  const Eigen::Vector3d& box_min,
  const Eigen::Vector3d& box_max)
{
  return isFiniteVector(box_min) &&
         isFiniteVector(box_max) &&
         box_min.x() <= box_max.x() &&
         box_min.y() <= box_max.y() &&
         box_min.z() <= box_max.z();
}

VoxelHashBackend::PointT VoxelHashBackend::toPclPoint(
  const Eigen::Vector3d& point)
{
  PointT pcl_point;
  pcl_point.x = static_cast<float>(point.x());
  pcl_point.y = static_cast<float>(point.y());
  pcl_point.z = static_cast<float>(point.z());
  pcl_point.intensity = 0.0F;
  return pcl_point;
}

Eigen::Vector3d VoxelHashBackend::toEigenPoint(const PointT& point)
{
  return Eigen::Vector3d(
    static_cast<double>(point.x),
    static_cast<double>(point.y),
    static_cast<double>(point.z));
}

void VoxelHashBackend::insertPointWithVoxelRule(const Eigen::Vector3d& point)
{
  if (!isFiniteVector(point) || !isValidVoxelSize(voxel_size_)) {
    return;
  }

  const VoxelKey key = voxelKeyFromPoint(point, voxel_size_);
  const Eigen::Vector3d center = voxelCenter(key, voxel_size_);

  auto it = voxel_representatives_.find(key);

  if (it == voxel_representatives_.end()) {
    voxel_representatives_.emplace(key, point);
    profiler_.addInsertedPoints(1);
    return;
  }

  const double old_distance_sq = (it->second - center).squaredNorm();
  const double new_distance_sq = (point - center).squaredNorm();

  // Keep the sample closest to the voxel center for a stable downsampled map.
  if (new_distance_sq < old_distance_sq) {
    it->second = point;
    profiler_.addVoxelReplacement(1);
  } else {
    profiler_.addRejectedByVoxel(1);
  }
}

void VoxelHashBackend::rebuildCloudAndKdTree()
{
  ScopedTimer rebuild_timer(profiler_.mutableSnapshot().rebuild_time_ms);

  // PCL owns the query index, so the cloud is rebuilt from voxel representatives.
  cloud_.reset(new CloudT());
  cloud_->points.reserve(voxel_representatives_.size());

  for (const auto& item : voxel_representatives_) {
    cloud_->points.push_back(toPclPoint(item.second));
  }

  updateCloudLayout();
  rebuildKdTree();
  profiler_.addRebuild();
}

void VoxelHashBackend::rebuildKdTree()
{
  if (cloud_->points.empty()) {
    kdtree_ready_ = false;
    return;
  }

  kdtree_->setInputCloud(cloud_);
  kdtree_ready_ = true;
}

void VoxelHashBackend::updateCloudLayout()
{
  cloud_->width = static_cast<std::uint32_t>(cloud_->points.size());
  cloud_->height = 1;
  cloud_->is_dense = false;
}

void VoxelHashBackend::refreshProfileSizes()
{
  profiler_.setMapSizes(
    voxel_representatives_.size(),
    voxel_representatives_.size(),
    0);
}

}  // namespace custom_ikd_tree_backend
