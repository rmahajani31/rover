#pragma once

#include <cstddef>
#include <memory>
#include <vector>

#include <Eigen/Core>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "custom_ikd_tree_backend/backend_profiler.hpp"
#include "custom_ikd_tree_backend/map_backend_interface.hpp"

namespace custom_ikd_tree_backend
{

class PclRebuildBackend final : public MapBackendInterface
{
public:
  explicit PclRebuildBackend(double voxel_leaf_size = 0.25);

  void setVoxelLeafSize(double voxel_leaf_size);
  double voxelLeafSize() const;

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
  static PointT toPclPoint(const Eigen::Vector3d& point);
  static Eigen::Vector3d toEigenPoint(const PointT& point);

  void appendFinitePoints(const std::vector<Eigen::Vector3d>& points);
  void rebuildKdTree();
  void downsampleMap();
  void updateCloudLayout();
  void refreshProfileSizes();

  double voxel_leaf_size_ = 0.25;

  CloudTPtr cloud_;
  pcl::KdTreeFLANN<PointT>::Ptr kdtree_;

  bool kdtree_ready_ = false;

  mutable BackendProfiler profiler_;
};

}  // namespace custom_ikd_tree_backend
