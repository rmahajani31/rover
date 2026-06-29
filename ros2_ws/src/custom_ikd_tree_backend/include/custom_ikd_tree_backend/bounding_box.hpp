#pragma once

#include <Eigen/Core>

namespace custom_ikd_tree_backend
{

struct BoundingBox
{
  BoundingBox() = default;

  BoundingBox(
    const Eigen::Vector3d& min_in,
    const Eigen::Vector3d& max_in);

  static BoundingBox fromCenterAndHalfSize(
    const Eigen::Vector3d& center,
    const Eigen::Vector3d& half_size);

  bool isFinite() const;

  bool isValid() const;

  bool contains(const Eigen::Vector3d& point) const;

  bool contains(const BoundingBox& other) const;

  bool intersects(const BoundingBox& other) const;

  bool isDisjoint(const BoundingBox& other) const;

  void expandToInclude(const Eigen::Vector3d& point);

  void expandToInclude(const BoundingBox& other);

  double squaredDistanceTo(const Eigen::Vector3d& point) const;

  Eigen::Vector3d size() const;

  Eigen::Vector3d center() const;

  Eigen::Vector3d min = Eigen::Vector3d::Zero();
  Eigen::Vector3d max = Eigen::Vector3d::Zero();
};

}  // namespace custom_ikd_tree_backend
