#pragma once

#include <cstddef>

#include <Eigen/Core>

#include "custom_scan_to_map_odom/local_map.hpp"
#include "custom_scan_to_map_odom/local_map_config.hpp"
#include "custom_scan_to_map_odom/local_map_diagnostics.hpp"
#include "custom_scan_to_map_odom/ros_conversions.hpp"

namespace custom_scan_to_map_odom
{

class LocalMapManager
{
public:
  LocalMapManager() = default;

  void configure(const LocalMapConfig& config);

  bool isInitialized() const;

  void initialize(
    const CloudTConstPtr& first_scan_map_frame,
    const Eigen::Vector3d& robot_position);

  void updateAfterOptimization(
    const CloudTConstPtr& scan_map_frame,
    const Eigen::Vector3d& robot_position);

  const LocalMap& localMap() const;

  CloudTConstPtr cloud() const;

  std::size_t size() const;

  const LocalMapDiagnostics& diagnostics() const;

private:
  void updateCubeIfNeeded(const Eigen::Vector3d& robot_position);
  bool isInsideCube(const PointT& point) const;
  Eigen::Vector3d cubeHalfSize() const;

  void cropOutsideCube();
  void insertScanInsideCube(const CloudTConstPtr& scan_map_frame);
  void downsampleMap();
  void rebuildKdTree();
  void updateDiagnosticCubeCenter();

  LocalMapConfig config_;
  LocalMapDiagnostics diagnostics_;

  LocalMap local_map_;
  Eigen::Vector3d cube_center_ = Eigen::Vector3d::Zero();
};

}  // namespace custom_scan_to_map_odom
