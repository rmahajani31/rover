#pragma once

#include <vector>

#include <Eigen/Core>

namespace custom_scan_to_map_odom
{

struct Plane
{
  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
  Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();

  double smallest_eigenvalue = 0.0;
  double middle_eigenvalue = 0.0;
  double largest_eigenvalue = 0.0;

  double max_error = 0.0;
};

struct PlaneFitterOptions
{
  double max_plane_error = 0.10;
  double min_plane_eigen_ratio = 5.0;
};

class PlaneFitter
{
public:
  explicit PlaneFitter(const PlaneFitterOptions& options = PlaneFitterOptions());

  bool fitPlane(
    const std::vector<Eigen::Vector3d>& points,
    Plane& plane) const;

private:
  PlaneFitterOptions options_;
};

}  // namespace custom_scan_to_map_odom
