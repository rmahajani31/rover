#pragma once

#include <Eigen/Core>

namespace custom_ikd_tree_backend
{

// Small point wrapper kept for backend-facing geometry helpers.
struct MapPoint
{
  MapPoint() = default;

  explicit MapPoint(const Eigen::Vector3d& position_in);

  MapPoint(double x, double y, double z);

  bool isFinite() const;

  double squaredDistanceTo(const Eigen::Vector3d& query) const;

  Eigen::Vector3d position = Eigen::Vector3d::Zero();
};

}  // namespace custom_ikd_tree_backend
