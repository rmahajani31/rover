#pragma once

#include <cstddef>
#include <memory>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "custom_ikd_tree_backend/backend_profiler.hpp"
#include "custom_ikd_tree_backend/map_backend_interface.hpp"
#include "custom_ikd_tree_backend/voxel_key.hpp"

namespace custom_ikd_tree_backend
{

class VoxelHashBackend final : public MapBackendInterface
{
public:
  explicit VoxelHashBackend(double voxel_size = 0.25);

  void setVoxelSize(double voxel_size);
  double voxelSize() const;

  const BackendProfileSnapshot& profileSnapshot() const override;
  void resetProfile() override;

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

private:
  using PointT = pcl::PointXYZI;
  using CloudT = pcl::PointCloud<PointT>;
  using CloudTPtr = CloudT::Ptr;

  static bool isFiniteVector(const Eigen::Vector3d& point);
  static bool isValidBox(
    const Eigen::Vector3d& box_min,
    const Eigen::Vector3d& box_max);

  static PointT toPclPoint(const Eigen::Vector3d& point);
  static Eigen::Vector3d toEigenPoint(const PointT& point);

  void insertPointWithVoxelRule(const Eigen::Vector3d& point);
  void rebuildCloudAndKdTree();
  void rebuildKdTree();
  void updateCloudLayout();
  void refreshProfileSizes();

  double voxel_size_ = 0.25;

  std::unordered_map<VoxelKey, Eigen::Vector3d, VoxelKeyHash> voxel_representatives_;

  CloudTPtr cloud_;
  pcl::KdTreeFLANN<PointT>::Ptr kdtree_;

  bool kdtree_ready_ = false;

  mutable BackendProfiler profiler_;
};

}  // namespace custom_ikd_tree_backend
