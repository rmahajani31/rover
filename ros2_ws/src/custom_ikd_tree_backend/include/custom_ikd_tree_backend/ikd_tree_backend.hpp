#pragma once

#include <cstddef>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>

#include "custom_ikd_tree_backend/backend_profiler.hpp"
#include "custom_ikd_tree_backend/bounding_box.hpp"
#include "custom_ikd_tree_backend/incremental_kd_tree.hpp"
#include "custom_ikd_tree_backend/map_backend_interface.hpp"
#include "custom_ikd_tree_backend/voxel_key.hpp"

namespace custom_ikd_tree_backend
{

struct IkdTreeBackendOptions
{
  double voxel_size = 0.25;
  double delete_tolerance = 1.0e-6;

  bool rebuild_enabled = true;
  double max_invalid_ratio = 0.30;
  std::size_t min_rebuild_size = 100;
  std::size_t max_insertions_before_rebuild = 2000;
};

class IkdTreeBackend final : public MapBackendInterface
{
public:
  explicit IkdTreeBackend(
    const IkdTreeBackendOptions& options = IkdTreeBackendOptions());

  void setOptions(const IkdTreeBackendOptions& options);
  const IkdTreeBackendOptions& options() const;

  const BackendProfileSnapshot& profileSnapshot() const;

  void clear() override;

  void buildFromPoints(
    const std::vector<Eigen::Vector3d>& points) override;

  void insertPoints(
    const std::vector<Eigen::Vector3d>& points) override;

  void insertPointsWithDownsampling(
    const std::vector<Eigen::Vector3d>& points) override;

  void deleteOutsideBox(
    const Eigen::Vector3d& box_min,
    const Eigen::Vector3d& box_max) override;

  bool knnSearch(
    const Eigen::Vector3d& query,
    int k,
    double max_distance,
    std::vector<Eigen::Vector3d>& neighbors) const override;

  std::size_t size() const override;

  std::size_t activeSize() const override;

  void getAllActivePoints(
    std::vector<Eigen::Vector3d>& points) const override;

  void rebuildIfNeeded();

  void rebuildNow();

private:
  static bool isFiniteVector(const Eigen::Vector3d& point);

  static bool isValidBox(
    const Eigen::Vector3d& box_min,
    const Eigen::Vector3d& box_max);

  void insertPointWithDownsampling(const Eigen::Vector3d& point);

  void updateVoxelRepresentativeOnly(
    const Eigen::Vector3d& point,
    bool update_profile);

  void rebuildVoxelHashFromActiveTree();

  void rebuildTreeFromVoxelHash();

  std::vector<Eigen::Vector3d> voxelRepresentativesAsVector() const;

  bool shouldRebuild() const;

  void refreshProfileSizes();

  IkdTreeBackendOptions options_;

  IncrementalKdTree tree_;

  std::unordered_map<VoxelKey, Eigen::Vector3d, VoxelKeyHash> voxel_representatives_;

  std::size_t insertions_since_rebuild_ = 0;
  bool rebuild_needed_ = false;

  mutable BackendProfiler profiler_;
};

}  // namespace custom_ikd_tree_backend
