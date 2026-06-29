#include "custom_ikd_tree_backend/map_point.hpp"

namespace custom_ikd_tree_backend
{

MapPoint::MapPoint(const Eigen::Vector3d& position_in)
: position(position_in)
{
}

MapPoint::MapPoint(double x, double y, double z)
: position(x, y, z)
{
}

bool MapPoint::isFinite() const
{
  return position.allFinite();
}

double MapPoint::squaredDistanceTo(const Eigen::Vector3d& query) const
{
  return (position - query).squaredNorm();
}

}  // namespace custom_ikd_tree_backend
