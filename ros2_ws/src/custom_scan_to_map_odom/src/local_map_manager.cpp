#include "custom_scan_to_map_odom/local_map_manager.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>

namespace custom_scan_to_map_odom
{

namespace
{

double elapsedMilliseconds(
  const std::chrono::steady_clock::time_point& start,
  const std::chrono::steady_clock::time_point& end)
{
  return std::chrono::duration<double, std::milli>(end - start).count();
}

void updateCloudLayout(const CloudTPtr& cloud)
{
  if (!cloud) {
    return;
  }

  cloud->width = static_cast<std::uint32_t>(cloud->points.size());
  cloud->height = 1;
  cloud->is_dense = false;
}

}  // namespace

void LocalMapManager::configure(const LocalMapConfig& config)
{
  config_ = config;
}

bool LocalMapManager::isInitialized() const
{
  return local_map_.isInitialized();
}

void LocalMapManager::initialize(
  const CloudTConstPtr& first_scan_map_frame,
  const Eigen::Vector3d& robot_position)
{
  const auto start = std::chrono::steady_clock::now();

  diagnostics_ = LocalMapDiagnostics{};
  cube_center_ = robot_position;

  diagnostics_.map_size_before_update =
    first_scan_map_frame ? first_scan_map_frame->size() : 0;
  diagnostics_.inserted_points =
    first_scan_map_frame ? first_scan_map_frame->size() : 0;

  local_map_.initialize(first_scan_map_frame);

  cropOutsideCube();
  downsampleMap();
  rebuildKdTree();

  const auto end = std::chrono::steady_clock::now();
  diagnostics_.total_update_time_ms = elapsedMilliseconds(start, end);

  updateDiagnosticCubeCenter();
}

void LocalMapManager::updateAfterOptimization(
  const CloudTConstPtr& scan_map_frame,
  const Eigen::Vector3d& robot_position)
{
  const auto start = std::chrono::steady_clock::now();

  diagnostics_ = LocalMapDiagnostics{};
  diagnostics_.map_size_before_update = local_map_.size();

  updateCubeIfNeeded(robot_position);
  cropOutsideCube();
  insertScanInsideCube(scan_map_frame);
  downsampleMap();
  rebuildKdTree();

  const auto end = std::chrono::steady_clock::now();
  diagnostics_.total_update_time_ms = elapsedMilliseconds(start, end);

  updateDiagnosticCubeCenter();
}

const LocalMap& LocalMapManager::localMap() const
{
  return local_map_;
}

CloudTConstPtr LocalMapManager::cloud() const
{
  return local_map_.cloud();
}

std::size_t LocalMapManager::size() const
{
  return local_map_.size();
}

const LocalMapDiagnostics& LocalMapManager::diagnostics() const
{
  return diagnostics_;
}

void LocalMapManager::updateCubeIfNeeded(const Eigen::Vector3d& robot_position)
{
  const double dx = std::abs(robot_position.x() - cube_center_.x());
  const double dy = std::abs(robot_position.y() - cube_center_.y());
  const double dz = std::abs(robot_position.z() - cube_center_.z());

  const bool shift_needed =
    dx > config_.movement_threshold_xy ||
    dy > config_.movement_threshold_xy ||
    dz > config_.movement_threshold_z;

  diagnostics_.cube_shifted = shift_needed;

  if (shift_needed) {
    cube_center_ = robot_position;
  }

  updateDiagnosticCubeCenter();
}

bool LocalMapManager::isInsideCube(const PointT& point) const
{
  const Eigen::Vector3d half_size = cubeHalfSize();

  return std::abs(static_cast<double>(point.x) - cube_center_.x()) <= half_size.x() &&
         std::abs(static_cast<double>(point.y) - cube_center_.y()) <= half_size.y() &&
         std::abs(static_cast<double>(point.z) - cube_center_.z()) <= half_size.z();
}

Eigen::Vector3d LocalMapManager::cubeHalfSize() const
{
  return Eigen::Vector3d(
    0.5 * config_.cube_size_x,
    0.5 * config_.cube_size_y,
    0.5 * config_.cube_size_z);
}

void LocalMapManager::cropOutsideCube()
{
  const auto start = std::chrono::steady_clock::now();

  const std::size_t before = local_map_.size();
  local_map_.cropAround(cube_center_, cubeHalfSize());
  const std::size_t after = local_map_.size();

  diagnostics_.removed_points_outside_cube = before > after ? before - after : 0;
  diagnostics_.map_size_after_crop = after;

  const auto end = std::chrono::steady_clock::now();
  diagnostics_.crop_time_ms = elapsedMilliseconds(start, end);
}

void LocalMapManager::insertScanInsideCube(const CloudTConstPtr& scan_map_frame)
{
  const auto start = std::chrono::steady_clock::now();

  if (!scan_map_frame || scan_map_frame->empty()) {
    diagnostics_.inserted_points = 0;
    diagnostics_.map_size_after_insert = local_map_.size();

    const auto end = std::chrono::steady_clock::now();
    diagnostics_.insert_time_ms = elapsedMilliseconds(start, end);
    return;
  }

  CloudTPtr filtered_scan(new CloudT());
  filtered_scan->points.reserve(scan_map_frame->points.size());

  for (const auto& point : scan_map_frame->points) {
    if (isInsideCube(point)) {
      filtered_scan->points.push_back(point);
    }
  }

  updateCloudLayout(filtered_scan);

  diagnostics_.inserted_points = filtered_scan->size();

  local_map_.insertCloud(filtered_scan);
  diagnostics_.map_size_after_insert = local_map_.size();

  const auto end = std::chrono::steady_clock::now();
  diagnostics_.insert_time_ms = elapsedMilliseconds(start, end);
}

void LocalMapManager::downsampleMap()
{
  const auto start = std::chrono::steady_clock::now();

  local_map_.downsample(config_.voxel_leaf_size);
  diagnostics_.map_size_after_downsample = local_map_.size();

  const auto end = std::chrono::steady_clock::now();
  diagnostics_.downsample_time_ms = elapsedMilliseconds(start, end);
}

void LocalMapManager::rebuildKdTree()
{
  const auto start = std::chrono::steady_clock::now();

  local_map_.rebuildKdTree();

  const auto end = std::chrono::steady_clock::now();
  diagnostics_.kdtree_rebuild_time_ms = elapsedMilliseconds(start, end);
}

void LocalMapManager::updateDiagnosticCubeCenter()
{
  diagnostics_.cube_center_x = cube_center_.x();
  diagnostics_.cube_center_y = cube_center_.y();
  diagnostics_.cube_center_z = cube_center_.z();
}

}  // namespace custom_scan_to_map_odom
