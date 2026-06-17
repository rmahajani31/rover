#include "custom_scan_to_map_odom/local_map_manager.hpp"

#include <cstdint>
#include <vector>

#include <Eigen/Core>
#include <gtest/gtest.h>

namespace custom_scan_to_map_odom
{
namespace
{

CloudTPtr makeCloud(const std::vector<Eigen::Vector3d>& points)
{
  CloudTPtr cloud(new CloudT());
  cloud->points.reserve(points.size());

  for (const auto& point : points) {
    PointT output;
    output.x = static_cast<float>(point.x());
    output.y = static_cast<float>(point.y());
    output.z = static_cast<float>(point.z());
    output.intensity = 1.0F;
    cloud->points.push_back(output);
  }

  cloud->width = static_cast<std::uint32_t>(cloud->points.size());
  cloud->height = 1;
  cloud->is_dense = false;
  return cloud;
}

LocalMapConfig testConfig()
{
  LocalMapConfig config;
  config.cube_size_x = 4.0;
  config.cube_size_y = 4.0;
  config.cube_size_z = 4.0;
  config.movement_threshold_xy = 2.0;
  config.movement_threshold_z = 1.0;
  config.voxel_leaf_size = 0.0;
  return config;
}

TEST(LocalMapManagerTest, InitializeCropsAndBuildsMap)
{
  LocalMapManager manager;
  manager.configure(testConfig());

  const auto first_scan = makeCloud({
    Eigen::Vector3d(0.0, 0.0, 0.0),
    Eigen::Vector3d(1.0, 0.0, 0.0),
    Eigen::Vector3d(0.0, 1.0, 0.0),
    Eigen::Vector3d(3.0, 0.0, 0.0),
  });

  manager.initialize(first_scan, Eigen::Vector3d::Zero());

  EXPECT_TRUE(manager.isInitialized());
  EXPECT_EQ(manager.size(), 3U);

  const auto& diagnostics = manager.diagnostics();
  EXPECT_EQ(diagnostics.map_size_before_update, 4U);
  EXPECT_EQ(diagnostics.map_size_after_crop, 3U);
  EXPECT_EQ(diagnostics.removed_points_outside_cube, 1U);
  EXPECT_DOUBLE_EQ(diagnostics.cube_center_x, 0.0);
  EXPECT_DOUBLE_EQ(diagnostics.cube_center_y, 0.0);
  EXPECT_DOUBLE_EQ(diagnostics.cube_center_z, 0.0);

  std::vector<Eigen::Vector3d> neighbors;
  std::vector<float> distances;
  EXPECT_TRUE(manager.localMap().nearestKSearch(
    Eigen::Vector3d(0.0, 0.0, 0.0),
    3,
    neighbors,
    distances));
  EXPECT_EQ(neighbors.size(), 3U);
}

TEST(LocalMapManagerTest, UpdateBelowThresholdDoesNotShiftCube)
{
  LocalMapManager manager;
  manager.configure(testConfig());
  manager.initialize(
    makeCloud({
      Eigen::Vector3d(0.0, 0.0, 0.0),
      Eigen::Vector3d(0.5, 0.0, 0.0),
      Eigen::Vector3d(0.0, 0.5, 0.0),
    }),
    Eigen::Vector3d::Zero());

  manager.updateAfterOptimization(
    makeCloud({Eigen::Vector3d(1.5, 0.0, 0.0)}),
    Eigen::Vector3d(1.5, 0.0, 0.0));

  const auto& diagnostics = manager.diagnostics();
  EXPECT_FALSE(diagnostics.cube_shifted);
  EXPECT_DOUBLE_EQ(diagnostics.cube_center_x, 0.0);
  EXPECT_EQ(diagnostics.inserted_points, 1U);
}

TEST(LocalMapManagerTest, UpdateAboveThresholdShiftsCubeAndCropsOldPoints)
{
  LocalMapManager manager;
  manager.configure(testConfig());
  manager.initialize(
    makeCloud({
      Eigen::Vector3d(0.0, 0.0, 0.0),
      Eigen::Vector3d(0.5, 0.0, 0.0),
      Eigen::Vector3d(0.0, 0.5, 0.0),
    }),
    Eigen::Vector3d::Zero());

  manager.updateAfterOptimization(
    makeCloud({
      Eigen::Vector3d(3.0, 0.0, 0.0),
      Eigen::Vector3d(3.5, 0.0, 0.0),
    }),
    Eigen::Vector3d(3.0, 0.0, 0.0));

  const auto& diagnostics = manager.diagnostics();
  EXPECT_TRUE(diagnostics.cube_shifted);
  EXPECT_DOUBLE_EQ(diagnostics.cube_center_x, 3.0);
  EXPECT_GT(diagnostics.removed_points_outside_cube, 0U);
  EXPECT_EQ(diagnostics.inserted_points, 2U);
  EXPECT_EQ(manager.size(), diagnostics.map_size_after_downsample);
}

TEST(LocalMapManagerTest, InsertRejectsScanPointsOutsideCurrentCube)
{
  LocalMapManager manager;
  manager.configure(testConfig());
  manager.initialize(
    makeCloud({
      Eigen::Vector3d(0.0, 0.0, 0.0),
      Eigen::Vector3d(0.5, 0.0, 0.0),
      Eigen::Vector3d(0.0, 0.5, 0.0),
    }),
    Eigen::Vector3d::Zero());

  manager.updateAfterOptimization(
    makeCloud({
      Eigen::Vector3d(1.0, 0.0, 0.0),
      Eigen::Vector3d(5.0, 0.0, 0.0),
    }),
    Eigen::Vector3d::Zero());

  const auto& diagnostics = manager.diagnostics();
  EXPECT_FALSE(diagnostics.cube_shifted);
  EXPECT_EQ(diagnostics.inserted_points, 1U);
  EXPECT_EQ(manager.size(), 4U);
}

TEST(LocalMapManagerTest, RebuildInvalidatesSearchAfterMapBecomesEmpty)
{
  LocalMapManager manager;
  manager.configure(testConfig());
  manager.initialize(
    makeCloud({
      Eigen::Vector3d(0.0, 0.0, 0.0),
      Eigen::Vector3d(0.5, 0.0, 0.0),
      Eigen::Vector3d(0.0, 0.5, 0.0),
    }),
    Eigen::Vector3d::Zero());

  manager.updateAfterOptimization(
    makeCloud({}),
    Eigen::Vector3d(10.0, 0.0, 0.0));

  EXPECT_EQ(manager.size(), 0U);

  std::vector<Eigen::Vector3d> neighbors;
  std::vector<float> distances;
  EXPECT_FALSE(manager.localMap().nearestKSearch(
    Eigen::Vector3d(0.0, 0.0, 0.0),
    3,
    neighbors,
    distances));
}

}  // namespace
}  // namespace custom_scan_to_map_odom
