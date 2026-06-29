#include "custom_ikd_tree_backend/voxel_key.hpp"

#include <cmath>
#include <functional>

namespace custom_ikd_tree_backend
{

bool VoxelKey::operator==(const VoxelKey& other) const
{
  return x == other.x && y == other.y && z == other.z;
}

bool VoxelKey::operator!=(const VoxelKey& other) const
{
  return !(*this == other);
}

std::size_t VoxelKeyHash::operator()(const VoxelKey& key) const
{
  const std::size_t h1 = std::hash<int>{}(key.x);
  const std::size_t h2 = std::hash<int>{}(key.y);
  const std::size_t h3 = std::hash<int>{}(key.z);

  return h1 ^ (h2 << 1) ^ (h3 << 2);
}

bool isValidVoxelSize(double voxel_size)
{
  return std::isfinite(voxel_size) && voxel_size > 0.0;
}

VoxelKey voxelKeyFromPoint(
  const Eigen::Vector3d& point,
  double voxel_size)
{
  if (!point.allFinite() || !isValidVoxelSize(voxel_size)) {
    return VoxelKey{};
  }

  return VoxelKey{
    static_cast<int>(std::floor(point.x() / voxel_size)),
    static_cast<int>(std::floor(point.y() / voxel_size)),
    static_cast<int>(std::floor(point.z() / voxel_size)),
  };
}

Eigen::Vector3d voxelCenter(
  const VoxelKey& key,
  double voxel_size)
{
  if (!isValidVoxelSize(voxel_size)) {
    return Eigen::Vector3d::Zero();
  }

  return Eigen::Vector3d(
    (static_cast<double>(key.x) + 0.5) * voxel_size,
    (static_cast<double>(key.y) + 0.5) * voxel_size,
    (static_cast<double>(key.z) + 0.5) * voxel_size);
}

}  // namespace custom_ikd_tree_backend
