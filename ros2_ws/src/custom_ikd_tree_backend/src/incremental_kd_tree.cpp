#include "custom_ikd_tree_backend/incremental_kd_tree.hpp"

#include <algorithm>

namespace custom_ikd_tree_backend
{

namespace
{

double squaredDistance(
  const Eigen::Vector3d& a,
  const Eigen::Vector3d& b)
{
  const double dx = a.x() - b.x();
  const double dy = a.y() - b.y();
  const double dz = a.z() - b.z();
  return dx * dx + dy * dy + dz * dz;
}

double squaredDistanceToBoxUnchecked(
  const BoundingBox& box,
  const Eigen::Vector3d& point)
{
  double distance_sq = 0.0;

  if (point.x() < box.min.x()) {
    const double d = box.min.x() - point.x();
    distance_sq += d * d;
  } else if (point.x() > box.max.x()) {
    const double d = point.x() - box.max.x();
    distance_sq += d * d;
  }

  if (point.y() < box.min.y()) {
    const double d = box.min.y() - point.y();
    distance_sq += d * d;
  } else if (point.y() > box.max.y()) {
    const double d = point.y() - box.max.y();
    distance_sq += d * d;
  }

  if (point.z() < box.min.z()) {
    const double d = box.min.z() - point.z();
    distance_sq += d * d;
  } else if (point.z() > box.max.z()) {
    const double d = point.z() - box.max.z();
    distance_sq += d * d;
  }

  return distance_sq;
}

}  // namespace

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

void IncrementalKdTree::rebuild()
{
  // Rebuilds are the compaction point for lazy deletions.
  std::vector<Eigen::Vector3d> active_points;
  getAllActivePoints(active_points);
  buildFromPoints(active_points);
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

  // The EKF calls KNN thousands of times per scan; reuse storage per thread.
  thread_local std::vector<NeighborCandidate> candidate_storage;
  candidate_storage.clear();

  if (candidate_storage.capacity() < static_cast<std::size_t>(k)) {
    candidate_storage.reserve(static_cast<std::size_t>(k));
  }

  NeighborSet neighbor_set{
    candidate_storage,
    k,
    max_distance_sq,
    max_distance_sq,
    0};

  knnRecursive(root_.get(), query, neighbor_set);

  if (static_cast<int>(neighbor_set.candidates.size()) < k) {
    return false;
  }

  std::sort(
    neighbor_set.candidates.begin(),
    neighbor_set.candidates.end(),
    [](const NeighborCandidate& a, const NeighborCandidate& b) {
      return a.squared_distance < b.squared_distance;
    });

  neighbors.reserve(neighbor_set.candidates.size());

  for (const auto& candidate : neighbor_set.candidates) {
    neighbors.push_back(candidate.point);
  }

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

double IncrementalKdTree::invalidRatio() const
{
  const std::size_t physical_size = size();

  if (physical_size == 0) {
    return 0.0;
  }

  return static_cast<double>(invalidCount()) /
         static_cast<double>(physical_size);
}

bool IncrementalKdTree::empty() const
{
  return !root_;
}

const BoundingBox* IncrementalKdTree::rootBox() const
{
  return root_ ? &root_->box : nullptr;
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

  // nth_element gives a balanced split without fully sorting each subtree.
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

  // Keep deletion cheap and defer physical removal to rebuild().
  if (!box.contains(node->point)) {
    node->deleted = true;
  }

  deleteOutsideBoxRecursive(node->left.get(), box);
  deleteOutsideBoxRecursive(node->right.get(), box);

  updateMetadata(*node);
}

double IncrementalKdTree::currentWorstSquared(
  const NeighborSet& neighbors)
{
  if (static_cast<int>(neighbors.candidates.size()) < neighbors.capacity) {
    return neighbors.max_distance_sq;
  }

  return neighbors.worst_squared_distance;
}

void IncrementalKdTree::refreshWorstNeighbor(NeighborSet& neighbors)
{
  neighbors.worst_index = 0;
  neighbors.worst_squared_distance = 0.0;

  if (neighbors.candidates.empty()) {
    return;
  }

  neighbors.worst_squared_distance =
    neighbors.candidates.front().squared_distance;

  for (std::size_t i = 1; i < neighbors.candidates.size(); ++i) {
    if (neighbors.candidates[i].squared_distance >
        neighbors.worst_squared_distance) {
      neighbors.worst_squared_distance =
        neighbors.candidates[i].squared_distance;
      neighbors.worst_index = i;
    }
  }
}

void IncrementalKdTree::tryAddNeighbor(
  const KdTreeNode& node,
  const Eigen::Vector3d& query,
  NeighborSet& neighbors)
{
  if (node.deleted) {
    return;
  }

  const double squared_distance = squaredDistance(query, node.point);

  if (squared_distance > neighbors.max_distance_sq) {
    return;
  }

  if (static_cast<int>(neighbors.candidates.size()) < neighbors.capacity) {
    neighbors.candidates.push_back(NeighborCandidate{squared_distance, node.point});

    if (neighbors.candidates.size() == 1 ||
        squared_distance > neighbors.worst_squared_distance) {
      neighbors.worst_squared_distance = squared_distance;
      neighbors.worst_index = neighbors.candidates.size() - 1;
    }
    return;
  }

  if (squared_distance < neighbors.worst_squared_distance) {
    neighbors.candidates[neighbors.worst_index] =
      NeighborCandidate{squared_distance, node.point};
    refreshWorstNeighbor(neighbors);
  }
}

void IncrementalKdTree::knnRecursive(
  const KdTreeNode* node,
  const Eigen::Vector3d& query,
  NeighborSet& neighbors)
{
  if (!node || node->subtree_deleted) {
    return;
  }

  const double worst_before_node =
    currentWorstSquared(neighbors);

  // The subtree box gives a lower bound; if it is already worse, skip the branch.
  if (squaredDistanceToBoxUnchecked(node->box, query) > worst_before_node) {
    return;
  }

  tryAddNeighbor(*node, query, neighbors);

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

  knnRecursive(near_child, query, neighbors);

  const double worst_after_near =
    currentWorstSquared(neighbors);

  if (far_child &&
      squaredDistanceToBoxUnchecked(far_child->box, query) <= worst_after_near) {
    knnRecursive(far_child, query, neighbors);
  }
}

}  // namespace custom_ikd_tree_backend
