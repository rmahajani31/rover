#pragma once

#include <cstddef>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "custom_scan_to_map_odom/local_map.hpp"
#include "custom_scan_to_map_odom/plane_fitter.hpp"
#include "custom_scan_to_map_odom/ros_conversions.hpp"

namespace custom_scan_to_map_odom
{

struct ScanToMapOptimizerOptions
{
  int max_iterations = 5;
  int k_neighbors = 5;

  std::size_t min_valid_correspondences = 100;

  double max_neighbor_distance = 1.0;
  double max_point_to_plane_residual = 0.50;

  double convergence_translation_epsilon = 0.001;
  double convergence_rotation_epsilon = 0.001;

  double max_pose_update_translation = 0.15;
  double max_pose_update_rotation_deg = 5.0;

  bool constrain_to_planar = true;

  PlaneFitterOptions plane_fitter;
};

struct OptimizationStats
{
  bool success = false;
  std::string status = "not_started";

  std::size_t input_points = 0;
  std::size_t valid_correspondences = 0;
  std::size_t valid_planes = 0;

  double mean_residual = 0.0;
  double max_residual = 0.0;

  double final_update_translation_norm = 0.0;
  double final_update_rotation_norm = 0.0;

  int iterations = 0;
};

class ScanToMapOptimizer
{
public:
  explicit ScanToMapOptimizer(
    const ScanToMapOptimizerOptions& options = ScanToMapOptimizerOptions());

  bool optimize(
    const CloudTConstPtr& scan_lidar_frame,
    const LocalMap& local_map,
    const Eigen::Isometry3d& initial_guess,
    Eigen::Isometry3d& optimized_pose,
    OptimizationStats& stats) const;

private:
  ScanToMapOptimizerOptions options_;
  PlaneFitter plane_fitter_;
};

}  // namespace custom_scan_to_map_odom
