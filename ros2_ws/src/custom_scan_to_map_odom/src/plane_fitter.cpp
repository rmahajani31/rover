#include "custom_scan_to_map_odom/plane_fitter.hpp"

#include <algorithm>
#include <cmath>

#include <Eigen/Eigenvalues>

namespace custom_scan_to_map_odom
{

PlaneFitter::PlaneFitter(const PlaneFitterOptions& options)
: options_(options)
{
}

bool PlaneFitter::fitPlane(
  const std::vector<Eigen::Vector3d>& points,
  Plane& plane) const
{
  if (points.size() < 3) {
    return false;
  }

  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();

  for (const auto& point : points) {
    if (!point.allFinite()) {
      return false;
    }

    centroid += point;
  }

  centroid /= static_cast<double>(points.size());

  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();

  // The smallest covariance direction is the plane normal; scale does not matter here.
  for (const auto& point : points) {
    const Eigen::Vector3d centered = point - centroid;
    covariance += centered * centered.transpose();
  }

  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);

  if (solver.info() != Eigen::Success) {
    return false;
  }

  const Eigen::Vector3d eigenvalues = solver.eigenvalues();
  const Eigen::Matrix3d eigenvectors = solver.eigenvectors();

  const double smallest = eigenvalues.x();
  const double middle = eigenvalues.y();
  const double largest = eigenvalues.z();

  if (!std::isfinite(smallest) || !std::isfinite(middle) || !std::isfinite(largest)) {
    return false;
  }

  constexpr double kNumericalEpsilon = 1.0e-12;

  if (middle <= kNumericalEpsilon || largest <= kNumericalEpsilon) {
    return false;
  }

  const double ratio_denominator = std::max(smallest, kNumericalEpsilon);
  const double middle_ratio = middle / ratio_denominator;
  const double largest_ratio = largest / ratio_denominator;

  // Planes should have two strong in-plane directions and one weak normal direction.
  if (middle_ratio < options_.min_plane_eigen_ratio ||
      largest_ratio < options_.min_plane_eigen_ratio) {
    return false;
  }

  Eigen::Vector3d normal = eigenvectors.col(0);
  const double normal_norm = normal.norm();

  if (normal_norm <= kNumericalEpsilon || !std::isfinite(normal_norm)) {
    return false;
  }

  normal /= normal_norm;

  double max_error = 0.0;

  for (const auto& point : points) {
    const double error = std::abs(normal.dot(point - centroid));

    if (!std::isfinite(error)) {
      return false;
    }

    max_error = std::max(max_error, error);
  }

  if (max_error > options_.max_plane_error) {
    return false;
  }

  plane.centroid = centroid;
  plane.normal = normal;
  plane.smallest_eigenvalue = smallest;
  plane.middle_eigenvalue = middle;
  plane.largest_eigenvalue = largest;
  plane.max_error = max_error;

  return true;
}

}  // namespace custom_scan_to_map_odom
