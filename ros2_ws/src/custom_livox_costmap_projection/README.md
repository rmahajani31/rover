# custom_livox_costmap_projection

Phase 6 obstacle perception module for the custom Livox MID-360 Nav2 rover stack.

This package will subscribe to `/custom/points_preprocessed`, transform the cloud into `base_link`, filter points by range and obstacle height, remove rover self-points, and publish `/custom/obstacle_cloud` for Nav2 costmaps.

It will also publish `/custom/projected_occupancy_grid` for RViz visualization and debugging.

## Topics

Inputs:

- `/custom/points_preprocessed`

Outputs:

- `/custom/obstacle_cloud`
- `/custom/projected_occupancy_grid`
- `/custom/costmap_projection_diagnostics`

## Frames

- `target_frame`: `base_link`
- `grid_frame`: `odom`
- `robot_frame`: `base_link`

Filtering should happen in `base_link` so `z` represents height relative to the rover. The debug occupancy grid should be published in `odom` so it stays visually stable while the rover moves.

## First Run

After the C++ node is implemented:

```bash
ros2 launch custom_livox_costmap_projection livox_costmap_projection.launch.py
```

## Phase 6 Goal

Replace the simple `/scan_from_livox` obstacle projection with a cleaner 3D obstacle filtering pipeline that better supports Nav2 local planning.
