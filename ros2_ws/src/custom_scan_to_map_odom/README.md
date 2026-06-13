# custom_scan_to_map_odom

Phase 5 LiDAR-only scan-to-map odometry package.

The node subscribes to `/custom/points_preprocessed`, maintains a local point cloud map, registers each incoming scan against that map with point-to-plane residuals, and publishes odometry, path, local map, and diagnostics.

## Topics

- Input: `/custom/points_preprocessed` (`sensor_msgs/msg/PointCloud2`)
- Default odometry: `/custom/scan_to_map_odom` (`nav_msgs/msg/Odometry`)
- Nav2 odometry in primary bringup: `/nav2_odom` (`nav_msgs/msg/Odometry`)
- Path: `/custom/scan_to_map_path` (`nav_msgs/msg/Path`)
- Local map: `/custom/local_map` (`sensor_msgs/msg/PointCloud2`)
- Diagnostics: `/custom/scan_to_map_diagnostics` (`diagnostic_msgs/msg/DiagnosticArray`)

Diagnostics are disabled by default while Phase 5 odometry is being smoke-tested on the Jetson.

## Shadow Mode

Use shadow mode first so another stable odometry source can continue publishing TF:

```bash
ros2 launch custom_scan_to_map_odom scan_to_map.launch.py publish_tf:=false
```

## Primary Nav2 Odometry Mode

Only enable TF after verifying no other node publishes `odom -> base_link`:

```bash
ros2 launch custom_scan_to_map_odom scan_to_map.launch.py publish_tf:=true
```

When TF lookup for `livox_frame -> base_link` succeeds, the node publishes odometry and TF as `odom -> base_link`. If that static transform is unavailable, odometry falls back to `odom -> livox_frame` and TF broadcasting is skipped.

For the existing Nav2 parameters in `bringup`, use the Phase 5 primary odometry launch so odometry is published on `/nav2_odom`:

```bash
ros2 launch bringup scan_to_map_primary_odom.launch.py
```

Before using this as Nav2's primary odometry, verify:

- Static test has little drift.
- Forward/backward motion has the correct sign.
- Yaw motion has the correct sign.
- The local map does not smear badly.
- There is exactly one `odom -> base_link` TF publisher.
