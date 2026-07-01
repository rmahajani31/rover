#pragma once

#include <memory>

#include <Eigen/Core>

#include "custom_ikd_tree_backend/bounding_box.hpp"

namespace custom_ikd_tree_backend
{

// Node metadata is intentionally cached so searches and rebuild decisions stay cheap.
struct KdTreeNode
{
  explicit KdTreeNode(
    const Eigen::Vector3d& point_in = Eigen::Vector3d::Zero(),
    int split_axis_in = 0);

  Eigen::Vector3d point = Eigen::Vector3d::Zero();

  std::unique_ptr<KdTreeNode> left;
  std::unique_ptr<KdTreeNode> right;

  int split_axis = 0;
  int tree_size = 1;

  BoundingBox box;

  // Deletions are lazy; rebuilds compact active points back into a fresh tree.
  bool deleted = false;
  bool subtree_deleted = false;
  int invalid_count = 0;
};

int sizeOf(const std::unique_ptr<KdTreeNode>& node);

int invalidCountOf(const std::unique_ptr<KdTreeNode>& node);

bool isActive(const KdTreeNode& node);

void updateMetadata(KdTreeNode& node);

}  // namespace custom_ikd_tree_backend
