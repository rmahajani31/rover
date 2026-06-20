#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace custom_lidar_deskew
{

struct ImuSample
{
  double t = 0.0;
  Eigen::Vector3d gyro = Eigen::Vector3d::Zero();
  Eigen::Vector3d accel = Eigen::Vector3d::Zero();
};

struct RotationSample
{
  double t = 0.0;
  Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
};

struct PoseSample
{
  double t = 0.0;
  Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
  Eigen::Vector3d p = Eigen::Vector3d::Zero();
  Eigen::Vector3d v = Eigen::Vector3d::Zero();
};

struct DeskewStats
{
  int raw_point_count = 0;
  int deskewed_point_count = 0;
  int imu_samples_used = 0;
  int missing_imu_count = 0;

  bool has_point_time = false;
  bool imu_coverage_ok = false;
  bool deskew_success = false;

  double scan_duration_sec = 0.0;
  double min_point_rel_time_sec = 0.0;
  double max_point_rel_time_sec = 0.0;
  double processing_time_ms = 0.0;
  double max_angular_velocity_rad_s = 0.0;
};

}  // namespace custom_lidar_deskew
