#include "custom_ikd_tree_backend/bounding_box.hpp"

namespace custom_ikd_tree_backend
{

BoundingBox::BoundingBox(
  const Eigen::Vector3d& min_in,
  const Eigen::Vector3d& max_in)
: min(min_in),
  max(max_in)
{
}

BoundingBox BoundingBox::fromCenterAndHalfSize(
  const Eigen::Vector3d& center,
  const Eigen::Vector3d& half_size)
{
  return BoundingBox(center - half_size, center + half_size);
}

bool BoundingBox::isFinite() const
{
  return min.allFinite() && max.allFinite();
}

bool BoundingBox::isValid() const
{
  return isFinite() &&
         min.x() <= max.x() &&
         min.y() <= max.y() &&
         min.z() <= max.z();
}

bool BoundingBox::contains(const Eigen::Vector3d& point) const
{
  if (!isValid() || !point.allFinite()) {
    return false;
  }

  return point.x() >= min.x() && point.x() <= max.x() &&
         point.y() >= min.y() && point.y() <= max.y() &&
         point.z() >= min.z() && point.z() <= max.z();
}

bool BoundingBox::contains(const BoundingBox& other) const
{
  if (!isValid() || !other.isValid()) {
    return false;
  }

  return other.min.x() >= min.x() && other.max.x() <= max.x() &&
         other.min.y() >= min.y() && other.max.y() <= max.y() &&
         other.min.z() >= min.z() && other.max.z() <= max.z();
}

bool BoundingBox::intersects(const BoundingBox& other) const
{
  return !isDisjoint(other);
}

bool BoundingBox::isDisjoint(const BoundingBox& other) const
{
  if (!isValid() || !other.isValid()) {
    return true;
  }

  return max.x() < other.min.x() || min.x() > other.max.x() ||
         max.y() < other.min.y() || min.y() > other.max.y() ||
         max.z() < other.min.z() || min.z() > other.max.z();
}

void BoundingBox::expandToInclude(const Eigen::Vector3d& point)
{
  if (!point.allFinite()) {
    return;
  }

  if (!isValid()) {
    min = point;
    max = point;
    return;
  }

  min = min.cwiseMin(point);
  max = max.cwiseMax(point);
}

void BoundingBox::expandToInclude(const BoundingBox& other)
{
  if (!other.isValid()) {
    return;
  }

  if (!isValid()) {
    min = other.min;
    max = other.max;
    return;
  }

  min = min.cwiseMin(other.min);
  max = max.cwiseMax(other.max);
}

double BoundingBox::squaredDistanceTo(const Eigen::Vector3d& point) const
{
  if (!isValid() || !point.allFinite()) {
    return 0.0;
  }

  double distance_sq = 0.0;

  for (int axis = 0; axis < 3; ++axis) {
    if (point[axis] < min[axis]) {
      const double d = min[axis] - point[axis];
      distance_sq += d * d;
    } else if (point[axis] > max[axis]) {
      const double d = point[axis] - max[axis];
      distance_sq += d * d;
    }
  }

  return distance_sq;
}

Eigen::Vector3d BoundingBox::size() const
{
  if (!isValid()) {
    return Eigen::Vector3d::Zero();
  }

  return max - min;
}

Eigen::Vector3d BoundingBox::center() const
{
  if (!isValid()) {
    return Eigen::Vector3d::Zero();
  }

  return 0.5 * (min + max);
}

}  // namespace custom_ikd_tree_backend
