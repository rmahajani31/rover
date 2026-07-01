#include "custom_ikd_tree_backend/backend_profiler.hpp"

namespace custom_ikd_tree_backend
{

ScopedTimer::ScopedTimer(double& output_ms)
: output_ms_(output_ms),
  start_(std::chrono::steady_clock::now())
{
}

ScopedTimer::~ScopedTimer()
{
  const auto end = std::chrono::steady_clock::now();
  output_ms_ += std::chrono::duration<double, std::milli>(end - start_).count();
}

void BackendProfiler::resetFrame()
{
  snapshot_ = BackendProfileSnapshot{};
}

BackendProfileSnapshot& BackendProfiler::mutableSnapshot()
{
  return snapshot_;
}

const BackendProfileSnapshot& BackendProfiler::snapshot() const
{
  return snapshot_;
}

void BackendProfiler::setMapSizes(
  std::size_t map_size,
  std::size_t active_size,
  std::size_t invalid_node_count)
{
  snapshot_.map_size = map_size;
  snapshot_.active_size = active_size;
  snapshot_.invalid_node_count = invalid_node_count;
  updateInvalidRatio();
}

void BackendProfiler::addKnnQuery()
{
  ++snapshot_.knn_query_count;
}

void BackendProfiler::addInsertedPoints(std::size_t count)
{
  snapshot_.inserted_point_count += count;
}

void BackendProfiler::addDeletedPoints(std::size_t count)
{
  snapshot_.deleted_point_count += count;
}

void BackendProfiler::addRejectedByVoxel(std::size_t count)
{
  snapshot_.rejected_by_voxel_count += count;
}

void BackendProfiler::addVoxelReplacement(std::size_t count)
{
  snapshot_.voxel_replacement_count += count;
}

void BackendProfiler::addRebuild()
{
  ++snapshot_.rebuild_count;
}

void BackendProfiler::updateInvalidRatio()
{
  if (snapshot_.map_size == 0) {
    snapshot_.invalid_ratio = 0.0;
    return;
  }

  snapshot_.invalid_ratio =
    static_cast<double>(snapshot_.invalid_node_count) /
    static_cast<double>(snapshot_.map_size);
}

}  // namespace custom_ikd_tree_backend
