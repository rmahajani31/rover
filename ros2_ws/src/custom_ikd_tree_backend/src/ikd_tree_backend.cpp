#include "custom_ikd_tree_backend/ikd_tree_backend.hpp"

namespace custom_ikd_tree_backend
{

IkdTreeBackend::IkdTreeBackend(const IkdTreeBackendOptions& options)
: options_(options)
{
  setOptions(options_);
  refreshProfileSizes();
}

void IkdTreeBackend::setOptions(const IkdTreeBackendOptions& options)
{
  options_ = options;

  if (!isValidVoxelSize(options_.voxel_size)) {
    options_.voxel_size = 0.25;
  }

  if (options_.delete_tolerance < 0.0) {
    options_.delete_tolerance = 1.0e-6;
  }

  std::vector<Eigen::Vector3d> active_points;
  getAllActivePoints(active_points);
  buildFromPoints(active_points);
}

const IkdTreeBackendOptions& IkdTreeBackend::options() const
{
  return options_;
}

const BackendProfileSnapshot& IkdTreeBackend::profileSnapshot() const
{
  return profiler_.snapshot();
}

void IkdTreeBackend::clear()
{
  tree_.clear();
  voxel_representatives_.clear();
  insertions_since_rebuild_ = 0;
  rebuild_needed_ = false;
  profiler_.resetFrame();
  refreshProfileSizes();
}

void IkdTreeBackend::buildFromPoints(
  const std::vector<Eigen::Vector3d>& points)
{
  profiler_.resetFrame();
  ScopedTimer total_timer(profiler_.mutableSnapshot().total_backend_time_ms);

  tree_.clear();
  voxel_representatives_.clear();
  insertions_since_rebuild_ = 0;
  rebuild_needed_ = false;

  {
    ScopedTimer insert_timer(profiler_.mutableSnapshot().insert_time_ms);

    for (const auto& point : points) {
      updateVoxelRepresentativeOnly(point, true);
    }
  }

  {
    ScopedTimer rebuild_timer(profiler_.mutableSnapshot().rebuild_time_ms);
    tree_.buildFromPoints(voxelRepresentativesAsVector());
    profiler_.addRebuild();
  }

  refreshProfileSizes();
  profiler_.mutableSnapshot().status = "success";
}

void IkdTreeBackend::insertPoints(
  const std::vector<Eigen::Vector3d>& points)
{
  insertPointsWithDownsampling(points);
}

void IkdTreeBackend::insertPointsWithDownsampling(
  const std::vector<Eigen::Vector3d>& points)
{
  profiler_.resetFrame();
  ScopedTimer total_timer(profiler_.mutableSnapshot().total_backend_time_ms);

  {
    ScopedTimer insert_timer(profiler_.mutableSnapshot().insert_time_ms);

    for (const auto& point : points) {
      insertPointWithDownsampling(point);
    }
  }

  rebuildIfNeeded();
  refreshProfileSizes();
  profiler_.mutableSnapshot().status = "success";
}

void IkdTreeBackend::deleteOutsideBox(
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

  {
    ScopedTimer delete_timer(profiler_.mutableSnapshot().delete_time_ms);

    const BoundingBox keep_box(box_min, box_max);
    const std::size_t before_active = tree_.activeSize();

    tree_.deleteOutsideBox(keep_box);

    const std::size_t after_active = tree_.activeSize();
    if (before_active > after_active) {
      profiler_.addDeletedPoints(before_active - after_active);
    }
  }

  if (shouldRebuild()) {
    rebuildNow();
  } else {
    rebuildVoxelHashFromActiveTree();
  }

  refreshProfileSizes();
  profiler_.mutableSnapshot().status = "success";
}

bool IkdTreeBackend::knnSearch(
  const Eigen::Vector3d& query,
  int k,
  double max_distance,
  std::vector<Eigen::Vector3d>& neighbors) const
{
  profiler_.addKnnQuery();
  ScopedTimer knn_timer(profiler_.mutableSnapshot().knn_time_ms);

  return tree_.knnSearch(query, k, max_distance, neighbors);
}

std::size_t IkdTreeBackend::size() const
{
  return tree_.size();
}

std::size_t IkdTreeBackend::activeSize() const
{
  return tree_.activeSize();
}

void IkdTreeBackend::getAllActivePoints(
  std::vector<Eigen::Vector3d>& points) const
{
  tree_.getAllActivePoints(points);
}

void IkdTreeBackend::rebuildIfNeeded()
{
  if (!shouldRebuild()) {
    return;
  }

  rebuildNow();
}

void IkdTreeBackend::rebuildNow()
{
  ScopedTimer rebuild_timer(profiler_.mutableSnapshot().rebuild_time_ms);

  rebuildVoxelHashFromActiveTree();
  tree_.buildFromPoints(voxelRepresentativesAsVector());

  insertions_since_rebuild_ = 0;
  rebuild_needed_ = false;

  profiler_.addRebuild();
  refreshProfileSizes();
}

bool IkdTreeBackend::isFiniteVector(const Eigen::Vector3d& point)
{
  return point.allFinite();
}

bool IkdTreeBackend::isValidBox(
  const Eigen::Vector3d& box_min,
  const Eigen::Vector3d& box_max)
{
  return isFiniteVector(box_min) &&
         isFiniteVector(box_max) &&
         box_min.x() <= box_max.x() &&
         box_min.y() <= box_max.y() &&
         box_min.z() <= box_max.z();
}

void IkdTreeBackend::insertPointWithDownsampling(
  const Eigen::Vector3d& point)
{
  if (!isFiniteVector(point) || !isValidVoxelSize(options_.voxel_size)) {
    return;
  }

  const VoxelKey key = voxelKeyFromPoint(point, options_.voxel_size);
  const Eigen::Vector3d center = voxelCenter(key, options_.voxel_size);

  auto it = voxel_representatives_.find(key);

  if (it == voxel_representatives_.end()) {
    voxel_representatives_.emplace(key, point);
    tree_.insertPoint(point);
    ++insertions_since_rebuild_;
    profiler_.addInsertedPoints(1);
    return;
  }

  const Eigen::Vector3d old_point = it->second;

  const double old_distance_sq = (old_point - center).squaredNorm();
  const double new_distance_sq = (point - center).squaredNorm();

  if (new_distance_sq < old_distance_sq) {
    const bool deleted =
      tree_.deletePoint(old_point, options_.delete_tolerance);

    if (deleted) {
      it->second = point;
      tree_.insertPoint(point);
      ++insertions_since_rebuild_;
      profiler_.addVoxelReplacement(1);
    } else {
      rebuild_needed_ = true;
    }
  } else {
    profiler_.addRejectedByVoxel(1);
  }
}

void IkdTreeBackend::updateVoxelRepresentativeOnly(
  const Eigen::Vector3d& point,
  bool update_profile)
{
  if (!isFiniteVector(point) || !isValidVoxelSize(options_.voxel_size)) {
    return;
  }

  const VoxelKey key = voxelKeyFromPoint(point, options_.voxel_size);
  const Eigen::Vector3d center = voxelCenter(key, options_.voxel_size);

  auto it = voxel_representatives_.find(key);

  if (it == voxel_representatives_.end()) {
    voxel_representatives_.emplace(key, point);
    if (update_profile) {
      profiler_.addInsertedPoints(1);
    }
    return;
  }

  const double old_distance_sq = (it->second - center).squaredNorm();
  const double new_distance_sq = (point - center).squaredNorm();

  if (new_distance_sq < old_distance_sq) {
    it->second = point;
    if (update_profile) {
      profiler_.addVoxelReplacement(1);
    }
  } else {
    if (update_profile) {
      profiler_.addRejectedByVoxel(1);
    }
  }
}

void IkdTreeBackend::rebuildVoxelHashFromActiveTree()
{
  std::vector<Eigen::Vector3d> active_points;
  tree_.getAllActivePoints(active_points);

  voxel_representatives_.clear();

  for (const auto& point : active_points) {
    updateVoxelRepresentativeOnly(point, false);
  }
}

std::vector<Eigen::Vector3d> IkdTreeBackend::voxelRepresentativesAsVector() const
{
  std::vector<Eigen::Vector3d> representatives;
  representatives.reserve(voxel_representatives_.size());

  for (const auto& item : voxel_representatives_) {
    representatives.push_back(item.second);
  }

  return representatives;
}

bool IkdTreeBackend::shouldRebuild() const
{
  if (!options_.rebuild_enabled) {
    return false;
  }

  if (rebuild_needed_) {
    return true;
  }

  if (tree_.size() >= options_.min_rebuild_size &&
      tree_.invalidRatio() > options_.max_invalid_ratio) {
    return true;
  }

  if (options_.max_insertions_before_rebuild > 0 &&
      insertions_since_rebuild_ > options_.max_insertions_before_rebuild) {
    return true;
  }

  return false;
}

void IkdTreeBackend::refreshProfileSizes()
{
  profiler_.setMapSizes(
    tree_.size(),
    tree_.activeSize(),
    tree_.invalidCount());
}

}  // namespace custom_ikd_tree_backend
