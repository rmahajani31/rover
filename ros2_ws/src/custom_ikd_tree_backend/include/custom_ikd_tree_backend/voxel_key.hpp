#pragma once

#include <cstddef>

#include <Eigen/Core>

namespace custom_ikd_tree_backend
{

// Integer grid coordinate for one downsampling voxel.
struct VoxelKey
{
  int x = 0;
  int y = 0;
  int z = 0;

  bool operator==(const VoxelKey& other) const;
  bool operator!=(const VoxelKey& other) const;
};

struct VoxelKeyHash
{
  std::size_t operator()(const VoxelKey& key) const;
};

// Voxel helpers keep the downsampling rule consistent across backends.
bool isValidVoxelSize(double voxel_size);

VoxelKey voxelKeyFromPoint(
  const Eigen::Vector3d& point,
  double voxel_size);

Eigen::Vector3d voxelCenter(
  const VoxelKey& key,
  double voxel_size);

}  // namespace custom_ikd_tree_backend
