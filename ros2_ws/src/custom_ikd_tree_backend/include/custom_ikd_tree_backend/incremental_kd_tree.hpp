#pragma once

#include <cstddef>
#include <memory>
#include <vector>

#include <Eigen/Core>

#include "custom_ikd_tree_backend/bounding_box.hpp"
#include "custom_ikd_tree_backend/kd_tree_node.hpp"

namespace custom_ikd_tree_backend
{

// Lightweight incremental kd-tree with lazy deletion and subtree-box KNN pruning.
class IncrementalKdTree
{
public:
  void clear();

  void buildFromPoints(const std::vector<Eigen::Vector3d>& points);

  void insertPoint(const Eigen::Vector3d& point);

  void insertPoints(const std::vector<Eigen::Vector3d>& points);

  bool deletePoint(
    const Eigen::Vector3d& point,
    double tolerance);

  void deleteOutsideBox(const BoundingBox& box);

  void rebuild();

  bool knnSearch(
    const Eigen::Vector3d& query,
    int k,
    double max_distance,
    std::vector<Eigen::Vector3d>& neighbors) const;

  void getAllActivePoints(std::vector<Eigen::Vector3d>& points) const;

  std::size_t size() const;
  std::size_t activeSize() const;
  std::size_t invalidCount() const;
  double invalidRatio() const;

  bool empty() const;

  const BoundingBox* rootBox() const;

private:
  struct NeighborCandidate
  {
    double squared_distance = 0.0;
    Eigen::Vector3d point = Eigen::Vector3d::Zero();
  };

  struct NeighborSet
  {
    // K is tiny in the EKF path, so a small vector avoids priority-queue overhead.
    std::vector<NeighborCandidate>& candidates;
    int capacity = 0;
    double max_distance_sq = 0.0;
    double worst_squared_distance = 0.0;
    std::size_t worst_index = 0;
  };

  static bool isFinitePoint(const Eigen::Vector3d& point);

  static std::unique_ptr<KdTreeNode> buildRecursive(
    std::vector<Eigen::Vector3d>& points,
    int begin,
    int end,
    int depth);

  static void insertRecursive(
    std::unique_ptr<KdTreeNode>& node,
    const Eigen::Vector3d& point,
    int depth);

  static void collectActivePointsRecursive(
    const KdTreeNode* node,
    std::vector<Eigen::Vector3d>& points);

  static bool deletePointRecursive(
    KdTreeNode* node,
    const Eigen::Vector3d& point,
    double tolerance_sq);

  static void deleteOutsideBoxRecursive(
    KdTreeNode* node,
    const BoundingBox& box);

  static double currentWorstSquared(
    const NeighborSet& neighbors);

  static void refreshWorstNeighbor(NeighborSet& neighbors);

  static void tryAddNeighbor(
    const KdTreeNode& node,
    const Eigen::Vector3d& query,
    NeighborSet& neighbors);

  static void knnRecursive(
    const KdTreeNode* node,
    const Eigen::Vector3d& query,
    NeighborSet& neighbors);

  std::unique_ptr<KdTreeNode> root_;
};

}  // namespace custom_ikd_tree_backend
