#include "custom_scan_to_map_odom/scan_to_map_optimizer.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include <Eigen/Cholesky>

#include <pcl/common/point_tests.h>

#include "custom_scan_to_map_odom/se3_utils.hpp"

namespace custom_scan_to_map_odom
{

namespace
{

double degreesToRadians(double degrees)
{
  constexpr double kPi = 3.14159265358979323846;
  return degrees * kPi / 180.0;
}

}  // namespace

ScanToMapOptimizer::ScanToMapOptimizer(
  const ScanToMapOptimizerOptions& options)
: options_(options),
  plane_fitter_(options.plane_fitter)
{
}

bool ScanToMapOptimizer::optimize(
  const CloudTConstPtr& scan_lidar_frame,
  const LocalMap& local_map,
  const Eigen::Isometry3d& initial_guess,
  Eigen::Isometry3d& optimized_pose,
  OptimizationStats& stats) const
{
  stats = OptimizationStats();
  optimized_pose = initial_guess;

  if (!scan_lidar_frame || scan_lidar_frame->empty()) {
    stats.status = "empty_scan";
    return false;
  }

  if (options_.max_iterations <= 0 || options_.k_neighbors < 3) {
    stats.status = "invalid_options";
    return false;
  }

  if (!local_map.isInitialized() ||
      local_map.size() < static_cast<std::size_t>(options_.k_neighbors)) {
    stats.status = "map_not_ready";
    return false;
  }

  stats.input_points = scan_lidar_frame->size();

  const double max_neighbor_distance_sq =
    options_.max_neighbor_distance * options_.max_neighbor_distance;
  const double max_rotation_update =
    degreesToRadians(options_.max_pose_update_rotation_deg);

  for (int iteration = 0; iteration < options_.max_iterations; ++iteration) {
    Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 6, 1> b = Eigen::Matrix<double, 6, 1>::Zero();

    std::size_t valid_correspondences = 0;
    std::size_t valid_planes = 0;
    double residual_sum = 0.0;
    double max_residual = 0.0;

    const Eigen::Matrix3d R = optimized_pose.rotation();
    const Eigen::Vector3d t = optimized_pose.translation();

    for (const auto& point : scan_lidar_frame->points) {
      if (!pcl::isFinite(point)) {
        continue;
      }

      const Eigen::Vector3d point_lidar(
        static_cast<double>(point.x),
        static_cast<double>(point.y),
        static_cast<double>(point.z));

      const Eigen::Vector3d point_map = R * point_lidar + t;

      std::vector<Eigen::Vector3d> neighbors;
      std::vector<float> squared_distances;

      if (!local_map.nearestKSearch(
          point_map,
          options_.k_neighbors,
          neighbors,
          squared_distances)) {
        continue;
      }

      if (squared_distances.empty() ||
          squared_distances.back() > static_cast<float>(max_neighbor_distance_sq)) {
        continue;
      }

      Plane plane;
      if (!plane_fitter_.fitPlane(neighbors, plane)) {
        continue;
      }

      ++valid_planes;

      const double residual = plane.normal.dot(point_map - plane.centroid);
      const double abs_residual = std::abs(residual);

      if (!std::isfinite(residual) ||
          abs_residual > options_.max_point_to_plane_residual) {
        continue;
      }

      Eigen::Matrix<double, 1, 6> J;
      J.block<1, 3>(0, 0) = -plane.normal.transpose() * skew(point_map);
      J.block<1, 3>(0, 3) = plane.normal.transpose();

      H += J.transpose() * J;
      b += J.transpose() * residual;

      ++valid_correspondences;
      residual_sum += abs_residual;
      max_residual = std::max(max_residual, abs_residual);
    }

    stats.valid_correspondences = valid_correspondences;
    stats.valid_planes = valid_planes;
    stats.mean_residual =
      valid_correspondences > 0 ? residual_sum / static_cast<double>(valid_correspondences) : 0.0;
    stats.max_residual = max_residual;
    stats.iterations = iteration + 1;

    if (valid_correspondences < options_.min_valid_correspondences) {
      stats.status = "not_enough_correspondences";
      return false;
    }

    constexpr double kMinLdltDiagonal = 1.0e-9;
    Eigen::Matrix<double, 6, 1> dx = Eigen::Matrix<double, 6, 1>::Zero();

    if (options_.constrain_to_planar) {
      Eigen::Matrix3d H_planar;
      Eigen::Vector3d b_planar;

      H_planar << H(2, 2), H(2, 3), H(2, 4),
                  H(3, 2), H(3, 3), H(3, 4),
                  H(4, 2), H(4, 3), H(4, 4);
      b_planar << b(2), b(3), b(4);

      const Eigen::LDLT<Eigen::Matrix3d> ldlt(H_planar);

      if (ldlt.info() != Eigen::Success) {
        stats.status = "linear_solve_failed";
        return false;
      }

      if (!ldlt.isPositive() ||
          ldlt.vectorD().array().abs().minCoeff() < kMinLdltDiagonal) {
        stats.status = "singular_system";
        return false;
      }

      const Eigen::Vector3d planar_dx = -ldlt.solve(b_planar);
      dx(2) = planar_dx(0);
      dx(3) = planar_dx(1);
      dx(4) = planar_dx(2);
    } else {
      const Eigen::LDLT<Eigen::Matrix<double, 6, 6>> ldlt(H);

      if (ldlt.info() != Eigen::Success) {
        stats.status = "linear_solve_failed";
        return false;
      }

      if (!ldlt.isPositive() ||
          ldlt.vectorD().array().abs().minCoeff() < kMinLdltDiagonal) {
        stats.status = "singular_system";
        return false;
      }

      dx = -ldlt.solve(b);
    }

    if (!dx.allFinite()) {
      stats.status = "non_finite_update";
      return false;
    }

    const double rotation_norm = dx.head<3>().norm();
    const double translation_norm = dx.tail<3>().norm();

    stats.final_update_rotation_norm = rotation_norm;
    stats.final_update_translation_norm = translation_norm;

    if (translation_norm > options_.max_pose_update_translation ||
        rotation_norm > max_rotation_update) {
      stats.status = "pose_update_too_large";
      return false;
    }

    optimized_pose = leftUpdateSE3(dx, optimized_pose);

    if (!optimized_pose.matrix().allFinite()) {
      stats.status = "non_finite_pose";
      return false;
    }

    if (translation_norm < options_.convergence_translation_epsilon &&
        rotation_norm < options_.convergence_rotation_epsilon) {
      stats.success = true;
      stats.status = "converged";
      return true;
    }
  }

  stats.success = true;
  stats.status = "max_iterations_reached";
  return true;
}

}  // namespace custom_scan_to_map_odom
