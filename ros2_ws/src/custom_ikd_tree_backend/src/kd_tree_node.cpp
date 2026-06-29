#include "custom_ikd_tree_backend/kd_tree_node.hpp"

namespace custom_ikd_tree_backend
{

KdTreeNode::KdTreeNode(
  const Eigen::Vector3d& point_in,
  int split_axis_in)
: point(point_in),
  split_axis(split_axis_in),
  box(point_in, point_in)
{
}

int sizeOf(const std::unique_ptr<KdTreeNode>& node)
{
  return node ? node->tree_size : 0;
}

int invalidCountOf(const std::unique_ptr<KdTreeNode>& node)
{
  return node ? node->invalid_count : 0;
}

bool isActive(const KdTreeNode& node)
{
  return !node.deleted && !node.subtree_deleted;
}

void updateMetadata(KdTreeNode& node)
{
  node.tree_size = 1 + sizeOf(node.left) + sizeOf(node.right);

  if (node.subtree_deleted) {
    node.invalid_count = node.tree_size;
  } else {
    node.invalid_count =
      (node.deleted ? 1 : 0) +
      invalidCountOf(node.left) +
      invalidCountOf(node.right);
  }

  node.box = BoundingBox(node.point, node.point);

  if (node.left) {
    node.box.expandToInclude(node.left->box);
  }

  if (node.right) {
    node.box.expandToInclude(node.right->box);
  }
}

}  // namespace custom_ikd_tree_backend
