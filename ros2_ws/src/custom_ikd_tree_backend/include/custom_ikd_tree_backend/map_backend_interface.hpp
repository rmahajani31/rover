#pragma once

#include <cstddef>
#include <vector>

#include <Eigen/Core>

namespace custom_ikd_tree_backend
{

class MapBackendInterface
{
public:
  virtual ~MapBackendInterface() = default;

  virtual void clear() = 0;

  virtual void buildFromPoints(
    const std::vector<Eigen::Vector3d>& points) = 0;

  virtual void insertPoints(
    const std::vector<Eigen::Vector3d>& points) = 0;

  virtual void insertPointsWithDownsampling(
    const std::vector<Eigen::Vector3d>& points) = 0;

  virtual void deleteOutsideBox(
    const Eigen::Vector3d& box_min,
    const Eigen::Vector3d& box_max) = 0;

  virtual bool knnSearch(
    const Eigen::Vector3d& query,
    int k,
    double max_distance,
    std::vector<Eigen::Vector3d>& neighbors) const = 0;

  virtual std::size_t size() const = 0;

  virtual std::size_t activeSize() const = 0;

  virtual void getAllActivePoints(
    std::vector<Eigen::Vector3d>& points) const = 0;
};

}  // namespace custom_ikd_tree_backend
