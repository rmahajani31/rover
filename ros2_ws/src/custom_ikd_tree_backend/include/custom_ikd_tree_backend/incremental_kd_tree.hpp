#pragma once

#include <cstddef>
#include <memory>
#include <queue>
#include <vector>

#include <Eigen/Core>

#include "custom_ikd_tree_backend/bounding_box.hpp"
#include "custom_ikd_tree_backend/kd_tree_node.hpp"

namespace custom_ikd_tree_backend
{

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

  bool knnSearch(
    const Eigen::Vector3d& query,
    int k,
    double max_distance,
    std::vector<Eigen::Vector3d>& neighbors) const;

  void getAllActivePoints(std::vector<Eigen::Vector3d>& points) const;

  std::size_t size() const;
  std::size_t activeSize() const;
  std::size_t invalidCount() const;

  bool empty() const;

  const BoundingBox* rootBox() const;

private:
  struct NeighborCandidate
  {
    double squared_distance = 0.0;
    Eigen::Vector3d point = Eigen::Vector3d::Zero();
  };

  struct WorseNeighborFirst
  {
    bool operator()(
      const NeighborCandidate& a,
      const NeighborCandidate& b) const;
  };

  using NeighborHeap =
    std::priority_queue<
      NeighborCandidate,
      std::vector<NeighborCandidate>,
      WorseNeighborFirst>;

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
    const NeighborHeap& heap,
    int k,
    double max_distance_sq);

  static void tryAddNeighbor(
    const KdTreeNode& node,
    const Eigen::Vector3d& query,
    int k,
    double max_distance_sq,
    NeighborHeap& heap);

  static void knnRecursive(
    const KdTreeNode* node,
    const Eigen::Vector3d& query,
    int k,
    double max_distance_sq,
    NeighborHeap& heap);

  std::unique_ptr<KdTreeNode> root_;
};

}  // namespace custom_ikd_tree_backend
