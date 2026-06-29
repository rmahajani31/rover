#include "custom_ikd_tree_backend/incremental_kd_tree.hpp"

#include <algorithm>

namespace custom_ikd_tree_backend
{

void IncrementalKdTree::clear()
{
  root_.reset();
}

void IncrementalKdTree::buildFromPoints(
  const std::vector<Eigen::Vector3d>& points)
{
  std::vector<Eigen::Vector3d> finite_points;
  finite_points.reserve(points.size());

  for (const auto& point : points) {
    if (isFinitePoint(point)) {
      finite_points.push_back(point);
    }
  }

  root_ = buildRecursive(
    finite_points,
    0,
    static_cast<int>(finite_points.size()),
    0);
}

void IncrementalKdTree::insertPoint(const Eigen::Vector3d& point)
{
  if (!isFinitePoint(point)) {
    return;
  }

  insertRecursive(root_, point, 0);
}

void IncrementalKdTree::insertPoints(
  const std::vector<Eigen::Vector3d>& points)
{
  for (const auto& point : points) {
    insertPoint(point);
  }
}

bool IncrementalKdTree::deletePoint(
  const Eigen::Vector3d& point,
  double tolerance)
{
  if (!root_ || !isFinitePoint(point) || tolerance < 0.0) {
    return false;
  }

  const double tolerance_sq = tolerance * tolerance;
  return deletePointRecursive(root_.get(), point, tolerance_sq);
}

void IncrementalKdTree::deleteOutsideBox(const BoundingBox& box)
{
  if (!root_ || !box.isValid()) {
    return;
  }

  deleteOutsideBoxRecursive(root_.get(), box);
}

bool IncrementalKdTree::knnSearch(
  const Eigen::Vector3d& query,
  int k,
  double max_distance,
  std::vector<Eigen::Vector3d>& neighbors) const
{
  neighbors.clear();

  if (!root_ || !isFinitePoint(query) || k <= 0 || max_distance <= 0.0) {
    return false;
  }

  const double max_distance_sq = max_distance * max_distance;

  NeighborHeap heap;
  knnRecursive(root_.get(), query, k, max_distance_sq, heap);

  if (static_cast<int>(heap.size()) < k) {
    return false;
  }

  neighbors.reserve(heap.size());

  while (!heap.empty()) {
    neighbors.push_back(heap.top().point);
    heap.pop();
  }

  std::reverse(neighbors.begin(), neighbors.end());
  return true;
}

void IncrementalKdTree::getAllActivePoints(
  std::vector<Eigen::Vector3d>& points) const
{
  points.clear();
  collectActivePointsRecursive(root_.get(), points);
}

std::size_t IncrementalKdTree::size() const
{
  return root_ ? static_cast<std::size_t>(root_->tree_size) : 0;
}

std::size_t IncrementalKdTree::activeSize() const
{
  if (!root_) {
    return 0;
  }

  return static_cast<std::size_t>(root_->tree_size - root_->invalid_count);
}

std::size_t IncrementalKdTree::invalidCount() const
{
  return root_ ? static_cast<std::size_t>(root_->invalid_count) : 0;
}

bool IncrementalKdTree::empty() const
{
  return !root_;
}

const BoundingBox* IncrementalKdTree::rootBox() const
{
  return root_ ? &root_->box : nullptr;
}

bool IncrementalKdTree::WorseNeighborFirst::operator()(
  const NeighborCandidate& a,
  const NeighborCandidate& b) const
{
  return a.squared_distance < b.squared_distance;
}

bool IncrementalKdTree::isFinitePoint(const Eigen::Vector3d& point)
{
  return point.allFinite();
}

std::unique_ptr<KdTreeNode> IncrementalKdTree::buildRecursive(
  std::vector<Eigen::Vector3d>& points,
  int begin,
  int end,
  int depth)
{
  if (begin >= end) {
    return nullptr;
  }

  const int axis = depth % 3;
  const int mid = begin + (end - begin) / 2;

  std::nth_element(
    points.begin() + begin,
    points.begin() + mid,
    points.begin() + end,
    [axis](const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
      return a[axis] < b[axis];
    });

  auto node = std::make_unique<KdTreeNode>(
    points[static_cast<std::size_t>(mid)],
    axis);

  node->left = buildRecursive(points, begin, mid, depth + 1);
  node->right = buildRecursive(points, mid + 1, end, depth + 1);

  updateMetadata(*node);
  return node;
}

void IncrementalKdTree::insertRecursive(
  std::unique_ptr<KdTreeNode>& node,
  const Eigen::Vector3d& point,
  int depth)
{
  if (!node) {
    node = std::make_unique<KdTreeNode>(point, depth % 3);
    return;
  }

  const int axis = node->split_axis;

  if (point[axis] < node->point[axis]) {
    insertRecursive(node->left, point, depth + 1);
  } else {
    insertRecursive(node->right, point, depth + 1);
  }

  updateMetadata(*node);
}

void IncrementalKdTree::collectActivePointsRecursive(
  const KdTreeNode* node,
  std::vector<Eigen::Vector3d>& points)
{
  if (!node || node->subtree_deleted) {
    return;
  }

  if (!node->deleted) {
    points.push_back(node->point);
  }

  collectActivePointsRecursive(node->left.get(), points);
  collectActivePointsRecursive(node->right.get(), points);
}

bool IncrementalKdTree::deletePointRecursive(
  KdTreeNode* node,
  const Eigen::Vector3d& point,
  double tolerance_sq)
{
  if (!node || node->subtree_deleted) {
    return false;
  }

  bool deleted_here = false;

  if (!node->deleted &&
      (node->point - point).squaredNorm() <= tolerance_sq) {
    node->deleted = true;
    deleted_here = true;
  }

  bool deleted_child = false;

  if (!deleted_here) {
    deleted_child =
      deletePointRecursive(node->left.get(), point, tolerance_sq) ||
      deletePointRecursive(node->right.get(), point, tolerance_sq);
  }

  if (deleted_here || deleted_child) {
    updateMetadata(*node);
    return true;
  }

  return false;
}

void IncrementalKdTree::deleteOutsideBoxRecursive(
  KdTreeNode* node,
  const BoundingBox& box)
{
  if (!node || node->subtree_deleted) {
    return;
  }

  if (!box.contains(node->point)) {
    node->deleted = true;
  }

  deleteOutsideBoxRecursive(node->left.get(), box);
  deleteOutsideBoxRecursive(node->right.get(), box);

  updateMetadata(*node);
}

double IncrementalKdTree::currentWorstSquared(
  const NeighborHeap& heap,
  int k,
  double max_distance_sq)
{
  if (static_cast<int>(heap.size()) < k) {
    return max_distance_sq;
  }

  return heap.top().squared_distance;
}

void IncrementalKdTree::tryAddNeighbor(
  const KdTreeNode& node,
  const Eigen::Vector3d& query,
  int k,
  double max_distance_sq,
  NeighborHeap& heap)
{
  if (node.deleted) {
    return;
  }

  const double squared_distance = (query - node.point).squaredNorm();

  if (squared_distance > max_distance_sq) {
    return;
  }

  if (static_cast<int>(heap.size()) < k) {
    heap.push(NeighborCandidate{squared_distance, node.point});
    return;
  }

  if (squared_distance < heap.top().squared_distance) {
    heap.pop();
    heap.push(NeighborCandidate{squared_distance, node.point});
  }
}

void IncrementalKdTree::knnRecursive(
  const KdTreeNode* node,
  const Eigen::Vector3d& query,
  int k,
  double max_distance_sq,
  NeighborHeap& heap)
{
  if (!node || node->subtree_deleted) {
    return;
  }

  const double worst_before_node =
    currentWorstSquared(heap, k, max_distance_sq);

  if (node->box.squaredDistanceTo(query) > worst_before_node) {
    return;
  }

  tryAddNeighbor(*node, query, k, max_distance_sq, heap);

  const int axis = node->split_axis;

  const KdTreeNode* near_child = nullptr;
  const KdTreeNode* far_child = nullptr;

  if (query[axis] < node->point[axis]) {
    near_child = node->left.get();
    far_child = node->right.get();
  } else {
    near_child = node->right.get();
    far_child = node->left.get();
  }

  knnRecursive(near_child, query, k, max_distance_sq, heap);

  const double worst_after_near =
    currentWorstSquared(heap, k, max_distance_sq);

  if (far_child &&
      far_child->box.squaredDistanceTo(query) <= worst_after_near) {
    knnRecursive(far_child, query, k, max_distance_sq, heap);
  }
}

}  // namespace custom_ikd_tree_backend
