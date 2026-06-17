#pragma once

#include <cstddef>

namespace custom_scan_to_map_odom
{

struct LocalMapDiagnostics
{
  // Per-update counters for the crop -> insert -> downsample -> rebuild sequence.
  std::size_t map_size_before_update = 0;
  std::size_t map_size_after_crop = 0;
  std::size_t map_size_after_insert = 0;
  std::size_t map_size_after_downsample = 0;

  std::size_t inserted_points = 0;
  std::size_t removed_points_outside_cube = 0;

  bool cube_shifted = false;

  double cube_center_x = 0.0;
  double cube_center_y = 0.0;
  double cube_center_z = 0.0;

  double crop_time_ms = 0.0;
  double insert_time_ms = 0.0;
  double downsample_time_ms = 0.0;
  double kdtree_rebuild_time_ms = 0.0;
  double total_update_time_ms = 0.0;
};

}  // namespace custom_scan_to_map_odom
