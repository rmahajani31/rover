#pragma once

#include <chrono>
#include <cstddef>
#include <string>

namespace custom_ikd_tree_backend
{

struct BackendProfileSnapshot
{
  std::size_t map_size = 0;
  std::size_t active_size = 0;
  std::size_t invalid_node_count = 0;

  std::size_t knn_query_count = 0;
  std::size_t inserted_point_count = 0;
  std::size_t deleted_point_count = 0;
  std::size_t rejected_by_voxel_count = 0;
  std::size_t voxel_replacement_count = 0;
  std::size_t rebuild_count = 0;

  double invalid_ratio = 0.0;

  double knn_time_ms = 0.0;
  double insert_time_ms = 0.0;
  double delete_time_ms = 0.0;
  double downsample_time_ms = 0.0;
  double rebuild_time_ms = 0.0;
  double total_backend_time_ms = 0.0;

  std::string status = "not_started";
};

class ScopedTimer
{
public:
  explicit ScopedTimer(double& output_ms);

  ~ScopedTimer();

  ScopedTimer(const ScopedTimer&) = delete;
  ScopedTimer& operator=(const ScopedTimer&) = delete;

private:
  double& output_ms_;
  std::chrono::steady_clock::time_point start_;
};

class BackendProfiler
{
public:
  void resetFrame();

  BackendProfileSnapshot& mutableSnapshot();
  const BackendProfileSnapshot& snapshot() const;

  void setMapSizes(
    std::size_t map_size,
    std::size_t active_size,
    std::size_t invalid_node_count);

  void addKnnQuery();
  void addInsertedPoints(std::size_t count);
  void addDeletedPoints(std::size_t count);
  void addRejectedByVoxel(std::size_t count);
  void addVoxelReplacement(std::size_t count);
  void addRebuild();

private:
  void updateInvalidRatio();

  BackendProfileSnapshot snapshot_;
};

}  // namespace custom_ikd_tree_backend
