#include "custom_lio_ekf/lidar_residual.hpp"

#include <cmath>
#include <cstddef>
#include <vector>

namespace custom_lio_ekf
{

custom_scan_to_map_odom::PlaneFitterOptions planeFitterOptionsFromLidarOptions(
  const LidarUpdateOptions& options)
{
  custom_scan_to_map_odom::PlaneFitterOptions plane_options;
  plane_options.max_plane_error = options.max_plane_error;
  plane_options.min_plane_eigen_ratio = options.min_plane_eigen_ratio;
  return plane_options;
}

Eigen::Vector3d transformLidarPointToImu(
  const Eigen::Vector3d& point_L,
  const LidarImuExtrinsics& extrinsics)
{
  return extrinsics.q_IL.normalized() * point_L + extrinsics.p_L_in_I;
}

Eigen::Vector3d transformImuPointToWorld(
  const EkfState& state,
  const Eigen::Vector3d& point_I)
{
  return state.q_WI.normalized() * point_I + state.p_I_W;
}

double pointToPlaneResidual(
  const Eigen::Vector3d& point_W,
  const Eigen::Vector3d& plane_centroid,
  const Eigen::Vector3d& plane_normal)
{
  return plane_normal.dot(point_W - plane_centroid);
}

Matrix1x18d pointToPlaneJacobian(
  const EkfState& state,
  const Eigen::Vector3d& point_I,
  const Eigen::Vector3d& plane_normal)
{
  Matrix1x18d H = Matrix1x18d::Zero();

  const Eigen::Matrix3d R_WI =
    state.q_WI.normalized().toRotationMatrix();
  const Eigen::RowVector3d n_transpose =
    plane_normal.normalized().transpose();

  // Jacobian of n^T(R_WI p_I + p_I_W - centroid) w.r.t. the error state.
  H.block<1, 3>(0, kThetaOffset) =
    n_transpose * (-R_WI * skewSymmetric(point_I));

  H.block<1, 3>(0, kPositionOffset) =
    n_transpose;

  return H;
}

bool buildPointToPlaneResidual(
  const Eigen::Vector3d& point_L,
  const EkfState& state,
  const LidarImuExtrinsics& extrinsics,
  const custom_ikd_tree_backend::MapBackendInterface& map_backend,
  const custom_scan_to_map_odom::PlaneFitter& plane_fitter,
  const LidarUpdateOptions& options,
  LidarResidual& result)
{
  result = LidarResidual{};

  if (!point_L.allFinite()) {
    result.status = "nonfinite_lidar_point";
    return false;
  }

  if (!isFinite(state)) {
    result.status = "nonfinite_state";
    return false;
  }

  if (map_backend.activeSize() < static_cast<std::size_t>(options.k_neighbors)) {
    result.status = "map_has_too_few_points";
    return false;
  }

  result.point_I = transformLidarPointToImu(point_L, extrinsics);
  result.point_W = transformImuPointToWorld(state, result.point_I);

  if (!result.point_I.allFinite() || !result.point_W.allFinite()) {
    result.status = "nonfinite_transformed_point";
    return false;
  }

  std::vector<Eigen::Vector3d> neighbors;

  if (!map_backend.knnSearch(
      result.point_W,
      options.k_neighbors,
      options.max_neighbor_distance,
      neighbors)) {
    result.status = "nearest_neighbor_search_failed";
    return false;
  }

  if (neighbors.size() < static_cast<std::size_t>(options.k_neighbors)) {
    result.status = "too_few_neighbors";
    return false;
  }

  custom_scan_to_map_odom::Plane plane;
  if (!plane_fitter.fitPlane(neighbors, plane)) {
    result.status = "invalid_plane";
    return false;
  }

  result.plane_centroid = plane.centroid;
  result.plane_normal = plane.normal.normalized();

  result.residual = pointToPlaneResidual(
    result.point_W,
    result.plane_centroid,
    result.plane_normal);

  if (!std::isfinite(result.residual)) {
    result.status = "nonfinite_residual";
    return false;
  }

  if (std::abs(result.residual) > options.max_point_to_plane_residual) {
    result.status = "residual_too_large";
    return false;
  }

  result.H = pointToPlaneJacobian(
    state,
    result.point_I,
    result.plane_normal);

  if (!result.H.allFinite()) {
    result.status = "nonfinite_jacobian";
    return false;
  }

  result.robust_weight = robustResidualWeight(result.residual, options);
  // Convert residual variance and robust loss into a scalar information weight.
  result.information_weight =
    result.robust_weight / lidarResidualVariance(options);

  result.valid = true;
  result.status = "success";
  return true;
}

}  // namespace custom_lio_ekf
