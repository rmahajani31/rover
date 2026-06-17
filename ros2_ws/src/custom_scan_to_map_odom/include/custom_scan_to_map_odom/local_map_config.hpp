#pragma once

namespace custom_scan_to_map_odom
{

struct LocalMapConfig
{
  // Full cube dimensions in map frame; LocalMapManager converts these to half extents.
  double cube_size_x = 30.0;
  double cube_size_y = 30.0;
  double cube_size_z = 6.0;

  // The cube is recentered only after the rover moves past these thresholds.
  double movement_threshold_xy = 5.0;
  double movement_threshold_z = 2.0;

  double voxel_leaf_size = 0.15;

  bool publish_local_map = true;
  double local_map_publish_period_sec = 1.0;
};

}  // namespace custom_scan_to_map_odom
