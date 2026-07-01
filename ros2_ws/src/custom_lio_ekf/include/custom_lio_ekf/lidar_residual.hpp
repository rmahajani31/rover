#pragma once

#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "custom_ikd_tree_backend/map_backend_interface.hpp"
#include "custom_scan_to_map_odom/plane_fitter.hpp"
#include "custom_lio_ekf/ekf_parameters.hpp"
#include "custom_lio_ekf/ekf_state.hpp"

namespace custom_lio_ekf
{

using Matrix1x18d = Eigen::Matrix<double, 1, kErrorStateDim>;

struct LidarImuExtrinsics
{
  // Rotation from LiDAR frame L to IMU frame I.
  Eigen::Quaterniond q_IL = Eigen::Quaterniond::Identity();

  // LiDAR origin position expressed in IMU frame I.
  Eigen::Vector3d p_L_in_I = Eigen::Vector3d::Zero();
};

struct LidarResidual
{
  bool valid = false;
  std::string status = "not_started";

  double residual = 0.0;
  double robust_weight = 1.0;
  double information_weight = 1.0;

  Matrix1x18d H = Matrix1x18d::Zero();

  Eigen::Vector3d point_I = Eigen::Vector3d::Zero();
  Eigen::Vector3d point_W = Eigen::Vector3d::Zero();

  Eigen::Vector3d plane_centroid = Eigen::Vector3d::Zero();
  Eigen::Vector3d plane_normal = Eigen::Vector3d::UnitZ();
};

custom_scan_to_map_odom::PlaneFitterOptions planeFitterOptionsFromLidarOptions(
  const LidarUpdateOptions& options);

Eigen::Vector3d transformLidarPointToImu(
  const Eigen::Vector3d& point_L,
  const LidarImuExtrinsics& extrinsics);

Eigen::Vector3d transformImuPointToWorld(
  const EkfState& state,
  const Eigen::Vector3d& point_I);

double pointToPlaneResidual(
  const Eigen::Vector3d& point_W,
  const Eigen::Vector3d& plane_centroid,
  const Eigen::Vector3d& plane_normal);

Matrix1x18d pointToPlaneJacobian(
  const EkfState& state,
  const Eigen::Vector3d& point_I,
  const Eigen::Vector3d& plane_normal);

bool buildPointToPlaneResidual(
  const Eigen::Vector3d& point_L,
  const EkfState& state,
  const LidarImuExtrinsics& extrinsics,
  const custom_ikd_tree_backend::MapBackendInterface& map_backend,
  const custom_scan_to_map_odom::PlaneFitter& plane_fitter,
  const LidarUpdateOptions& options,
  LidarResidual& result);

}  // namespace custom_lio_ekf
