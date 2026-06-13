#include "custom_scan_to_map_odom/plane_fitter.hpp"

#include <limits>
#include <vector>

#include <gtest/gtest.h>

namespace custom_scan_to_map_odom
{
namespace
{

TEST(PlaneFitterTest, FitsHorizontalPlane)
{
  const std::vector<Eigen::Vector3d> points{
    Eigen::Vector3d(-1.0, -1.0, 0.0),
    Eigen::Vector3d(1.0, -1.0, 0.0),
    Eigen::Vector3d(-1.0, 1.0, 0.0),
    Eigen::Vector3d(1.0, 1.0, 0.0),
    Eigen::Vector3d(0.0, 0.0, 0.0),
  };

  Plane plane;
  const PlaneFitter fitter;

  ASSERT_TRUE(fitter.fitPlane(points, plane));
  EXPECT_NEAR(std::abs(plane.normal.dot(Eigen::Vector3d::UnitZ())), 1.0, 1.0e-9);
  EXPECT_NEAR(plane.max_error, 0.0, 1.0e-9);
}

TEST(PlaneFitterTest, FitsVerticalPlane)
{
  const std::vector<Eigen::Vector3d> points{
    Eigen::Vector3d(2.0, -1.0, -1.0),
    Eigen::Vector3d(2.0, 1.0, -1.0),
    Eigen::Vector3d(2.0, -1.0, 1.0),
    Eigen::Vector3d(2.0, 1.0, 1.0),
    Eigen::Vector3d(2.0, 0.0, 0.0),
  };

  Plane plane;
  const PlaneFitter fitter;

  ASSERT_TRUE(fitter.fitPlane(points, plane));
  EXPECT_NEAR(std::abs(plane.normal.dot(Eigen::Vector3d::UnitX())), 1.0, 1.0e-9);
  EXPECT_NEAR(plane.centroid.x(), 2.0, 1.0e-9);
}

TEST(PlaneFitterTest, RejectsTooFewPoints)
{
  const std::vector<Eigen::Vector3d> points{
    Eigen::Vector3d(0.0, 0.0, 0.0),
    Eigen::Vector3d(1.0, 0.0, 0.0),
  };

  Plane plane;
  const PlaneFitter fitter;

  EXPECT_FALSE(fitter.fitPlane(points, plane));
}

TEST(PlaneFitterTest, RejectsNonFinitePoint)
{
  const double quiet_nan = std::numeric_limits<double>::quiet_NaN();
  const std::vector<Eigen::Vector3d> points{
    Eigen::Vector3d(0.0, 0.0, 0.0),
    Eigen::Vector3d(1.0, 0.0, 0.0),
    Eigen::Vector3d(0.0, 1.0, 0.0),
    Eigen::Vector3d(quiet_nan, 0.0, 0.0),
  };

  Plane plane;
  const PlaneFitter fitter;

  EXPECT_FALSE(fitter.fitPlane(points, plane));
}

TEST(PlaneFitterTest, RejectsNoisyNeighborhood)
{
  const std::vector<Eigen::Vector3d> points{
    Eigen::Vector3d(0.0, 0.0, 0.0),
    Eigen::Vector3d(1.0, 0.0, 0.8),
    Eigen::Vector3d(0.0, 1.0, -0.8),
    Eigen::Vector3d(1.0, 1.0, 0.7),
    Eigen::Vector3d(0.5, 0.2, -0.9),
  };

  PlaneFitterOptions options;
  options.max_plane_error = 0.05;
  options.min_plane_eigen_ratio = 5.0;

  Plane plane;
  const PlaneFitter fitter(options);

  EXPECT_FALSE(fitter.fitPlane(points, plane));
}

}  // namespace
}  // namespace custom_scan_to_map_odom
