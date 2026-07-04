# Rover

Code to operate a homemade ROS 2 rover with an ESP32 motor controller, a
Raspberry Pi, and a Jetson processing Livox Mid-360 lidar data.

## System Overview

This repository is organized around a split compute stack:

- The ESP32 runs the low-level microcontroller firmware.
- The Raspberry Pi runs the main rover ROS 2 system on ROS 2 Kilted.
- The Jetson runs lidar point cloud processing on ROS 2 Humble.

The Jetson is currently used for Livox point cloud projection, custom Livox
preprocessing, FAST-LIO2 experiments, IMU point cloud deskewing,
error-state Kalman filter odometry, and shadow-mode scan-to-scan ICP odometry.
It also hosts local map management mode, where scan-to-map odometry and Livox
obstacle projection run on the Jetson while Nav2 runs on the Pi. It is
included in the architecture so heavier lidar point cloud processing and
future camera-based visual processing can be added there as the rover stack
grows.

The micro-ROS bridge must be started on the Pi before launching mapping or
autonomous navigation.

## Prerequisites

On the Raspberry Pi:

- ROS 2 Kilted
- micro-ROS agent
- Nav2
- slam_toolbox
- `joy` package for gamepad input
- Access to the ESP32 USB serial device. This project uses
`/dev/tty_rover_esp` as a configured stable device name.
- Access to the goBILDA Pinpoint v2 over I2C bus `1` at address `0x31`

On the Jetson:

- ROS 2 Humble
- Livox ROS driver for the Livox Mid-360
- `livox_cloud_to_scan`
- `custom_fastlio_preprocess`, when using filtered MID-360 clouds before scan
projection
- `custom_icp_odom`, when running custom scan-to-scan ICP odometry in shadow
mode
- `custom_scan_to_map_odom`, when using scan-to-map registration based
odometry as the Nav2 odometry source
- `custom_lidar_deskew`, when using IMU point cloud deskew mode
- `custom_imu_propagator`, when using IMU-assisted scan-to-map initial guesses
- `custom_lio_ekf`, when using the rover error-state Kalman filter odometry mode
- `custom_ikd_tree_backend`, when using the EKF ikd tree map backend mode
- `fastlio2_nav2_adapter`, when using FAST-LIO2 as the Nav2 odometry source
- `custom_livox_costmap_projection`, when using the upgraded costmap mode
- `fast_lio` from
  [Ericsii/FAST_LIO_ROS2](https://github.com/Ericsii/FAST_LIO_ROS2), when
  running FAST-LIO2 shadow mode or FAST-LIO2 Nav2 odometry

The Livox driver used to publish point clouds is:

[Livox-SDK/livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2)

## Build

Build the ROS 2 workspace on the machine that will run the packages:

```bash
cd ~/Documents/projects/rover/ros2_ws
colcon build
source install/setup.bash
```

The Pi and Jetson run different parts of the stack, so each machine should have
the packages it needs built and sourced in its own ROS 2 environment.

The Jetson should build the lidar-heavy packages:

```bash
colcon build --packages-select custom_fastlio_preprocess custom_livox_costmap_projection custom_icp_odom custom_imu_propagator custom_scan_to_map_odom custom_lidar_deskew custom_ikd_tree_backend custom_lio_ekf livox_cloud_to_scan bringup
source install/setup.bash
```

The Pi does not need `livox_ros_driver2`, `custom_fastlio_preprocess`,
`custom_livox_costmap_projection`, `custom_icp_odom`,
`custom_imu_propagator`, `custom_scan_to_map_odom`,
`custom_lidar_deskew`, `custom_ikd_tree_backend`, or `custom_lio_ekf` for normal
driving/mapping. If those Jetson-only packages are present in the Pi workspace
but their dependencies are not installed, skip them:

```bash
colcon build --packages-skip custom_fastlio_preprocess custom_livox_costmap_projection custom_icp_odom custom_imu_propagator custom_scan_to_map_odom custom_lidar_deskew custom_ikd_tree_backend custom_lio_ekf
source install/setup.bash
```

## Repository Structure

### ESP32 Firmware

The microcontroller code runs on an ESP32 from:

```text
ros2_ws/firmware/freertos_apps/apps/rover/app.c
```

This firmware is the low-level rover application that communicates with the ROS
2 system through micro-ROS.

### micro-ROS Agent

Once micro-ROS is installed, start the bridge on the Pi with the ESP32 USB
serial device path:

```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/tty_rover_esp
```

Run this before launching mapping or navigation so the Pi can communicate with
the ESP32.

There is also a helper script at:

```text
ros2_ws/start_micro_ros.sh
```

### ROS 2 Workspace

The main ROS 2 workspace is:

```text
ros2_ws/
```

Key directories:

- `ros2_ws/src/` contains the ROS 2 packages.
- `ros2_ws/maps/` stores saved maps from mapping sessions.
- `ros2_ws/rviz/` contains RViz configuration.
- `ros2_ws/firmware/` contains the ESP32 firmware and micro-ROS app code.

## Machine Responsibilities


| Machine      | Responsibility                                                                                                                                                                                                  |
| ------------ | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| ESP32        | Runs the low-level rover firmware and communicates with the ROS 2 system through micro-ROS.                                                                                                                     |
| Raspberry Pi | Runs the micro-ROS agent, gamepad control, mapping, localization, Nav2, and motor-command path. In Jetson-odometry Nav2 modes, it consumes `/nav2_odom` from the Jetson instead of starting `rover_odometry`. |
| Jetson       | Runs Livox point cloud processing, custom preprocessing, IMU point cloud deskewing, error-state Kalman filter odometry, scan-to-scan ICP shadow odometry, scan-to-map registration based odometry, FAST-LIO2, `/scan_from_livox`, and Jetson-side odometry adapters.      |


## ROS 2 Packages

### `bringup`

Main package for launch files used by the rover stack.

Important launch files:

- `jetson.launch.py` starts the Jetson-side processing that projects 3D point
cloud data into 2D scan data.
- `fastlio_shadow.launch.py` starts FAST-LIO2 beside the existing scan
projection path. Nav2 continues using stable rover odometry while FAST-LIO2
publishes `/fastlio_odom` for observation and bagging.
- `fast_lio2_nav2.launch.py` starts the Jetson-side FAST-LIO2 Nav2 odometry
path, including scan projection and the `/nav2_odom` adapter.
- `pointcloud_preprocess_fast_lio2_nav2.launch.py` starts the Jetson-side FAST-LIO2 Nav2
odometry path with the custom Livox preprocessing node feeding scan projection.
- `costmap_projection.launch.py` starts the Jetson-side upgraded costmap mode:
custom preprocessing, FAST-LIO2, `/nav2_odom`, `/scan_from_livox` for AMCL,
and `/custom/obstacle_cloud` for the Nav2 local costmap and collision monitor.
- `scan_to_map_costmap_projection.launch.py` starts local map management mode
on the Jetson: custom Livox preprocessing, IMU-assisted scan-to-map odometry,
`/scan_from_livox` for AMCL, and `/custom/obstacle_cloud` for the Nav2 local
costmap and collision monitor.
- `scan_to_map_deskew_costmap_projection.launch.py` starts IMU point cloud
deskew mode on the Jetson: custom Livox preprocessing, `custom_lidar_deskew`,
scan-to-map odometry from `/custom/deskewed_points`, `/scan_from_livox` for
AMCL, and `/custom/obstacle_cloud` for the Nav2 local costmap and collision
monitor.
- `lio_ekf_deskew_costmap_projection.launch.py` starts the rover error-state
Kalman filter mode on the Jetson: custom Livox preprocessing, IMU point cloud
deskewing, `custom_lio_ekf` odometry from `/custom/deskewed_points`, ikd tree
map backend mode for local-map KNN, `/scan_from_livox` for AMCL, and
`/custom/obstacle_cloud` for the Nav2 local costmap and collision monitor.
- `scan_to_scan_icp.launch.py` starts Jetson-side custom preprocessing and
`custom_icp_odom` in shadow mode. It publishes custom ICP odometry for
debugging but does not publish `odom -> base_link`.
- `pi_fast_lio2_nav2_main.launch.py` starts Pi-side Nav2 localization and
navigation using `/nav2_odom` from the Jetson with the FAST-LIO2 Nav2 parameter
baseline from the main branch.
- `pi_fast_lio2_costmap_projection_nav2.launch.py` starts Pi-side Nav2 using
`/nav2_odom` from the Jetson, `/scan_from_livox` for AMCL, and
`/custom/obstacle_cloud` for the Nav2 local costmap and collision monitoring.
- `mapping.launch.py` launches room mapping and saves maps into `ros2_ws/maps/`.
- `pi_nav2_livox.launch.py` launches autonomous navigation using Nav2.

### `gamepad_adapter`

Package used to control the rover with a gamepad. This is especially helpful
while manually driving the rover to map a room.

### `livox_cloud_to_scan`

Package that runs on the Jetson to process 3D point cloud data from the Livox
Mid-360 and project it into 2D scan data for the rest of the navigation stack.
The Jetson is intended to host more compute-heavy lidar point cloud processing
and camera-based visual processing as the rover stack grows.

For the current Jetson workflow, `/livox/lidar` is a Livox `CustomMsg` topic.
Derived `PointCloud2` topics such as `/custom/points_preprocessed`,
`/custom/points_for_nav2`, and `/livox/points` are the safe inputs for
visualization, scan projection, and scan-to-map registration.

The package can also build without `livox_ros_driver2`; in that case the
Livox CustomMsg converter is skipped. This is useful on the Pi, where the
driver and converter are not part of the runtime path.

### `custom_fastlio_preprocess`

Package that runs on the Jetson and subscribes directly to the Livox
`CustomMsg` stream on `/livox/lidar`. It publishes two filtered PointCloud2
streams:

```text
/custom/points_preprocessed
/custom/points_for_nav2
```

`/custom/points_preprocessed` is a downsampled cloud used by scan-to-map
registration based odometry and other scan-matching work. `/custom/points_for_nav2`
is an obstacle-focused cloud that feeds `livox_cloud_to_scan`, which still
publishes the Nav2 scan topic `/scan_from_livox`.

In IMU point cloud deskew mode, the preprocessor also publishes a timed cloud
for deskewing:

```text
/custom/points_for_deskew
```

That topic preserves per-point Livox `offset_time` so `custom_lidar_deskew`
can correct scan motion before scan-to-map registration.

The first implementation does not transform clouds internally. Nav2 height
filtering is tuned in `livox_frame` so the output remains compatible with the
existing `livox_cloud_to_scan` height slice after the cloud is transformed into
`base_link`.

### `custom_lidar_deskew`

Package that runs on the Jetson in IMU point cloud deskew mode. It subscribes
to the timed preprocessor cloud and Livox IMU:

```text
/custom/points_for_deskew
/livox/imu
```

It uses each point's relative `offset_time` and rotation integrated from the
Livox IMU to express the scan as if every point were captured at the scan end.
The current implementation performs rotation-only deskewing; translation
deskewing is reserved for future work.

Primary output:

```text
/custom/deskewed_points
```

Diagnostics are published on:

```text
/custom/deskew/diagnostics
```

The diagnostics report point counts, scan duration, IMU coverage, deskew
success, processing time, and maximum angular velocity. If point timing or IMU
coverage is not usable, the node can publish the raw cloud so downstream
scan-to-map processing keeps running.

### `custom_lio_ekf`

Package that runs on the Jetson in the rover error-state Kalman filter mode. It
subscribes to deskewed Livox points and the Livox IMU:

```text
/custom/deskewed_points
/livox/imu
```

The filter keeps an 18-state error covariance for orientation, position, velocity,
gyro bias, accel bias, and gravity. IMU propagation predicts the state between
accepted scans, and each deskewed LiDAR scan performs an iterated point-to-plane
correction against a bounded local map.

Primary Nav2 outputs are:

```text
/nav2_odom
odom -> base_link
```

Standalone/default inspection outputs include:

```text
/custom/lio_ekf_odom
/custom/lio_ekf_path
/custom/lio_ekf_local_map
/custom/lio_ekf_diagnostics
```

In the Nav2 bringup, the odometry output is remapped to `/nav2_odom`; local map
and diagnostics topics can be enabled for inspection when needed.

In the current rover bringup, Nav2 receives only LiDAR-corrected poses. The EKF
does not publish intermediate IMU-only odometry, which avoids stationary drift
from accelerometer bias or scale error.

The local map backend is selected in
`custom_lio_ekf/config/lio_ekf.yaml` with `local_map.backend_type`. The current
rover configuration uses `ikd_tree`. The `pcl_rebuild` and `voxel_hash` backend
types are kept for comparison and fallback testing.

### `custom_ikd_tree_backend`

Package that provides local map backend implementations for `custom_lio_ekf`.
The current ikd tree map backend mode keeps one representative point per voxel,
uses an incremental kd-tree for nearest-neighbor queries, and lazily marks
deleted tree nodes until a rebuild compacts and rebalances the index.

Supported backend types are:

```text
ikd_tree
pcl_rebuild
voxel_hash
```

The voxel hash remains the authoritative downsampled local map content. The
tree is the query index used during point-to-plane residual construction.
Backend timing and map-size counters are included in
`/custom/lio_ekf_diagnostics` with `map_backend_lidar_*`,
`map_backend_update_*`, and `map_backend_latest_*` keys.

### `custom_livox_costmap_projection`

Package that runs on the Jetson in upgraded costmap mode. It consumes the
preprocessed Livox point cloud:

```text
/custom/points_preprocessed
```

It transforms the cloud into `base_link`, filters obstacle points by range,
height, and rover self-body bounds, then publishes:

```text
/custom/obstacle_cloud
/custom/projected_occupancy_grid
/custom/costmap_projection_diagnostics
```

`/custom/obstacle_cloud` is the Nav2 obstacle source for the local costmap,
and collision monitor. `/custom/projected_occupancy_grid` is a debug view for
RViz, not the primary Nav2 costmap input. The global costmap remains map-based
in the current Nav2 configuration.

In this mode, `/scan_from_livox` is still published, but it is reserved for
AMCL localization. This keeps localization scan matching separate from obstacle
marking.

### `custom_icp_odom`

Package that runs on the Jetson and subscribes to the preprocessed odometry
cloud:

```text
/custom/points_preprocessed
```

It estimates scan-to-scan lidar odometry by registering each current scan
against the previous scan. The current default uses PCL GICP because it is much
more stable than point-to-point ICP in stationary tests with the MID-360.

Shadow-mode outputs are:

```text
/custom/icp_odom
/custom/icp_path
/custom/icp/aligned_cloud
/custom/icp/source_cloud
/custom/icp/target_cloud
```

`custom_icp_odom` does not publish TF in scan-to-scan ICP shadow mode. Nav2
should continue using rover odometry or official FAST-LIO2 odometry while the
custom ICP trajectory is inspected in RViz.

### `custom_scan_to_map_odom`

Package that runs on the Jetson and subscribes to the preprocessed odometry
cloud:

```text
/custom/points_preprocessed
```

It maintains a bounded local point cloud map, registers each incoming scan
against that map with point-to-plane residuals, and publishes rover odometry
for Nav2. The local map manager keeps the map inside a configurable cube around
the rover, recenters that cube only after movement thresholds are crossed,
clips new scan points to the active cube, downsamples the map, and rebuilds the
nearest-neighbor structure after each accepted update.

During the IMU integration phase, the node also subscribes to `/livox/imu` and
uses rotation-only IMU propagation to seed the scan-to-map optimizer. The final
Nav2 odometry still comes from accepted lidar scan-to-map registration; IMU
translation is not used in the published pose.

Primary Nav2 outputs are:

```text
/nav2_odom
odom -> base_link
```

Debug and inspection outputs are:

```text
/custom/scan_to_map_odom
/custom/imu_predicted_odom
/custom/scan_to_map_path
/custom/local_map
/custom/scan_to_map_diagnostics
```

The primary bringup publishes `/nav2_odom` and broadcasts `odom -> base_link`
from the latest accepted scan-to-map pose. If tracking is degraded for several
consecutive scans, odometry covariance is increased and TF refreshing is stopped
so Nav2 does not keep consuming a stale pose as if it were healthy.

Scan-to-map diagnostics are published on `/custom/scan_to_map_diagnostics`.
They include scan-matching status, IMU initial-guess status, timing, and local
map counters.

### `custom_imu_propagator`

Package containing the reusable IMU propagation code used by the scan-to-map
odometry node. It buffers Livox IMU samples, integrates angular velocity
between scan timestamps, and returns a rotation delta that can be used as an
optimizer initial guess.

The package also includes a standalone node for inspecting IMU propagation. The
normal Nav2 path embeds the propagator inside `custom_scan_to_map_odom`.

### `fastlio2_nav2_adapter`

Package that bridges official FAST-LIO2 odometry into the Nav2 odometry
contract. It subscribes to `/fastlio_odom`, republishes `/nav2_odom`, and by
default publishes the `odom -> base_link` transform.

The adapter keeps the Nav2-facing odometry planar by default:

```text
z = 0
roll = 0
pitch = 0
yaw preserved
```

This keeps the ground-robot Nav2 costmaps from inheriting small vertical,
roll, or pitch estimates from FAST-LIO2.

### `rover_odometry`

Package responsible for rover odometry. Odometry is broadcast over I2C to the
Pi from the goBILDA Pinpoint v2 odometry computer.

## Important Topics and Frames


| Name               | Type            | Notes                                                                     |
| ------------------ | --------------- | ------------------------------------------------------------------------- |
| `/livox/lidar`     | Topic           | Livox input from `livox_ros_driver2`.                                |
| `/livox/imu`       | Topic           | Livox IMU input used for scan-to-map optimizer initial guesses.       |
| `/livox/points`    | Topic           | PointCloud2 copy of Livox CustomMsg data used only in FAST-LIO2 shadow mode. |
| `/custom/points_preprocessed` | Topic | Filtered PointCloud2 stream used by scan-to-map registration and scan-matching work. |
| `/custom/points_for_nav2` | Topic | Filtered PointCloud2 stream projected into `/scan_from_livox` in the custom preprocessing mode. |
| `/custom/points_for_deskew` | Topic | Timed PointCloud2 stream with per-point `offset_time` for IMU point cloud deskewing. |
| `/custom/preprocess_diagnostics` | Topic | Runtime point counts, rejection counts, and processing time from `custom_fastlio_preprocess`. |
| `/custom/deskewed_points` | Topic | Deskewed PointCloud2 stream used by scan-to-map registration in IMU point cloud deskew mode. |
| `/custom/deskew/diagnostics` | Topic | Deskew health, IMU coverage, scan duration, point counts, and processing time. |
| `/custom/lio_ekf_odom` | Topic | Default error-state Kalman filter odometry output when not remapped to `/nav2_odom`. |
| `/custom/lio_ekf_path` | Topic | Accepted error-state Kalman filter pose path for RViz inspection. |
| `/custom/lio_ekf_local_map` | Topic | Local point cloud map maintained by the error-state Kalman filter odometry backend. |
| `/custom/lio_ekf_diagnostics` | Topic | EKF prediction, LiDAR correction, covariance, residual, timing, and map backend diagnostics. |
| `/custom/obstacle_cloud` | Topic | Filtered obstacle PointCloud2 stream used by the Nav2 local costmap and collision monitor in upgraded costmap mode. |
| `/custom/projected_occupancy_grid` | Topic | Debug occupancy grid built from `/custom/obstacle_cloud` for RViz inspection. |
| `/custom/costmap_projection_diagnostics` | Topic | Runtime projection health, TF status, point counts, and grid occupancy counts. |
| `/custom/icp_odom` | Topic | Shadow-mode scan-to-scan ICP odometry from `custom_icp_odom`. |
| `/custom/icp_path` | Topic | Accumulated custom ICP path for RViz comparison. |
| `/custom/icp/aligned_cloud` | Topic | Current scan transformed onto the previous scan by ICP/GICP. |
| `/custom/icp/source_cloud` | Topic | Current downsampled scan used as the registration source. |
| `/custom/icp/target_cloud` | Topic | Previous downsampled scan used as the registration target. |
| `/custom/scan_to_map_odom` | Topic | Default scan-to-map registration odometry output when not remapped to `/nav2_odom`. |
| `/custom/imu_predicted_odom` | Topic | Debug-only IMU predicted pose before lidar scan-to-map correction. |
| `/custom/scan_to_map_path` | Topic | Accepted scan-to-map pose path for RViz inspection. |
| `/custom/local_map` | Topic | Current local point cloud map maintained by scan-to-map registration. |
| `/custom/scan_to_map_diagnostics` | Topic | Scan-to-map status, correspondence counts, residuals, and timing. |
| `/scan_from_livox` | Topic           | 2D scan output from `livox_cloud_to_scan`; used by slam_toolbox and Nav2. |
| `/fastlio_odom`    | Topic           | FAST-LIO2 odometry output for observation, bagging, and adapter input.     |
| `/nav2_odom`       | Topic           | Nav2 odometry topic published by the active Jetson odometry source.        |
| `/joy`             | Topic           | Gamepad input from `joy_node`.                                            |
| `/cmd_vel`         | Topic           | Rover velocity command published by `gamepad_adapter`.                    |
| `odom`             | Topic and frame | Wheel odometry topic in the rover-odometry path; odom frame in both Nav2 paths. |
| `map`              | Frame           | Global map frame used by slam_toolbox and Nav2.                           |
| `base_footprint`   | Frame           | Odometry child frame from `rover_odometry`.                               |
| `base_link`        | Frame           | Main rover body frame.                                                    |
| `livox_frame`      | Frame           | Livox lidar frame; statically published from `base_link`.                 |


## Operating Modes

The completed rover stack can be run in these main ways:

- Raw Livox mapping and navigation with rover odometry.
- FAST-LIO2 running beside the existing rover stack for observation.
- FAST-LIO2 providing odometry to Nav2 while the raw converted cloud feeds
`/scan_from_livox`.
- FAST-LIO2 providing odometry to Nav2 while custom filtered clouds feed
`/scan_from_livox`.
- FAST-LIO2 providing odometry to Nav2 while the upgraded costmap mode feeds
the Nav2 local costmap and collision monitor with `/custom/obstacle_cloud` and
keeps `/scan_from_livox` for AMCL only.
- Scan-to-map registration based odometry providing `/nav2_odom` to Nav2 while
custom filtered clouds feed `/scan_from_livox`.
- Local map management mode using IMU-assisted scan-to-map odometry, a bounded
local map, `/custom/obstacle_cloud` for the Nav2 local costmap and collision
monitor, and `/scan_from_livox` for AMCL.
- IMU point cloud deskew mode using `/custom/deskewed_points` for scan-to-map
odometry while preserving `/custom/points_preprocessed` for obstacle
projection and `/scan_from_livox` for AMCL.
- Error-state Kalman filter mode using `/custom/deskewed_points` and
`/livox/imu` for Jetson-side odometry. In ikd tree map backend mode,
`custom_lio_ekf` uses `custom_ikd_tree_backend` for local-map KNN and updates
while preserving `/custom/points_preprocessed` for obstacle projection and
`/scan_from_livox` for AMCL.
- Custom scan-to-scan ICP odometry running in shadow mode while the rover is
driven using the normal Pi-side mapping or navigation stack.

### Manual Driving

Gamepad control is included in `mapping.launch.py`, which starts both
`joy_node` and `gamepad_adapter`.

The default joystick device is:

```text
/dev/input/js0
```

The gamepad adapter subscribes to `/joy` and publishes velocity commands on
`/cmd_vel`. It starts with E-stop active, so initialize the joystick and then
use the configured E-stop button to enable motion.

### Raw Livox Mapping

Before mapping, start the micro-ROS agent on the Pi and the Livox processing on
the Jetson. Then run mapping on the Pi:

```bash
ros2 launch bringup mapping.launch.py
```

This launch file starts gamepad control, rover odometry, the
`base_footprint -> base_link` static transform, and slam_toolbox.

### Raw Livox Navigation

Autonomous navigation with the saved map uses rover odometry and the standard
Livox scan projection path:

```text
/livox/lidar -> livox_cloud_to_scan -> /scan_from_livox
rover_odometry -> odom -> base_footprint
```

Start Jetson-side Livox scan projection:

```bash
ros2 launch bringup jetson.launch.py
```

Then start Nav2 on the Pi:

```bash
ros2 launch bringup pi_nav2_livox.launch.py map:=/home/rmahajani/Documents/projects/rover/ros2_ws/maps/rover_map.yaml
```

This launch starts rover odometry, the `base_footprint -> base_link` static
transform, Nav2 localization, and Nav2 navigation.

### FAST-LIO2 Shadow Observation

Shadow mode means FAST-LIO2 runs beside the existing rover navigation stack,
but does not control localization or navigation. Its odometry is published for
inspection, comparison, and bag recording while Nav2 continues to use the
stable `/odom` source.

FAST-LIO2 shadow mode runs on the Jetson while the Pi keeps using the stable
rover odometry and `/scan_from_livox` for mapping or Nav2. This is for
observing FAST-LIO2 odometry, checking frames, and recording comparison bags;
it does not replace `/odom`.

Start the Livox driver in CustomMsg mode:

```bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

Then start the shadow launch:

```bash
ros2 launch bringup fastlio_shadow.launch.py
```

The shadow launch keeps the normal scan path alive with:

```text
/livox/lidar  -> FAST-LIO2
/livox/lidar  -> livox_custom_msg_to_pointcloud2 -> /livox/points
/livox/points -> livox_cloud_to_scan             -> /scan_from_livox
```

Expected FAST-LIO2 frames are separate from the Nav2 tree:

```text
camera_init -> body
```

The normal rover tree should remain:

```text
map -> odom -> base_footprint -> base_link -> livox_frame
```

### FAST-LIO2 Odometry With Raw Scan Projection

In this mode, FAST-LIO2 becomes the odometry source used by Nav2. The Jetson
runs the Livox CustomMsg path, official FAST-LIO2, scan projection, and the
adapter. The Pi runs Nav2 and the motor-command path.

Start the Livox driver on the Jetson in CustomMsg mode:

```bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

Then start the Jetson FAST-LIO2/Nav2 odometry stack:

```bash
ros2 launch bringup fast_lio2_nav2.launch.py
```

This produces:

```text
/livox/lidar  -> FAST-LIO2                         -> /fastlio_odom
/livox/lidar  -> livox_custom_msg_to_pointcloud2   -> /livox/points
/livox/points -> livox_cloud_to_scan               -> /scan_from_livox
/fastlio_odom -> fastlio2_nav2_adapter             -> /nav2_odom
fastlio2_nav2_adapter                              -> odom -> base_link
```

On the Pi, start Nav2 with the saved map:

```bash
ros2 launch bringup pi_fast_lio2_nav2_main.launch.py map:=/home/rmahajani/Documents/projects/rover/ros2_ws/maps/rover_map.yaml
```

The Pi launch starts Nav2 localization and navigation, but intentionally does
not launch `rover_odometry`. In this mode the TF tree should be:

```text
map -> odom -> base_link -> livox_frame
```

Before sending a goal, verify:

```bash
ros2 topic hz /fastlio_odom
ros2 topic hz /nav2_odom
ros2 topic hz /scan_from_livox
ros2 topic echo /nav2_odom --once
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link livox_frame
```

`/nav2_odom` should use `header.frame_id: odom` and
`child_frame_id: base_link`. Exactly one node should publish
`odom -> base_link`.

### FAST-LIO2 Odometry With Custom Preprocessed Clouds

This mode keeps the same FAST-LIO2 odometry path as above, but replaces the
obstacle cloud path with `custom_fastlio_preprocess`.

Start the Livox driver on the Jetson in CustomMsg mode:

```bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

Then start the Jetson stack with custom preprocessing:

```bash
ros2 launch bringup pointcloud_preprocess_fast_lio2_nav2.launch.py
```

This produces:

```text
/livox/lidar                  -> FAST-LIO2                 -> /fastlio_odom
/fastlio_odom                 -> fastlio2_nav2_adapter     -> /nav2_odom
/livox/lidar                  -> custom_fastlio_preprocess -> /custom/points_preprocessed
/livox/lidar                  -> custom_fastlio_preprocess -> /custom/points_for_nav2
/custom/points_for_nav2       -> livox_cloud_to_scan       -> /scan_from_livox
fastlio2_nav2_adapter                                      -> odom -> base_link
```

On the Pi, start Nav2 with the same FAST-LIO2 odometry launch:

```bash
ros2 launch bringup pi_fast_lio2_nav2_main.launch.py map:=/home/rmahajani/Documents/projects/rover/ros2_ws/maps/rover_map.yaml
```

Before sending a goal, verify:

```bash
ros2 topic hz /custom/points_preprocessed
ros2 topic hz /custom/points_for_nav2
ros2 topic echo /custom/preprocess_diagnostics --once
ros2 topic hz /scan_from_livox
ros2 topic hz /nav2_odom
```

This mode is the preferred current setup for testing custom point cloud
preprocessing while still relying on official FAST-LIO2 for rover odometry.

### Upgraded Costmap Mode

This mode keeps FAST-LIO2 as the odometry source for Nav2, but changes the
obstacle pipeline used by the local costmap and collision monitor. AMCL still
receives `/scan_from_livox` for localization, while live Livox obstacles are
provided through `/custom/obstacle_cloud`.

This is an upgrade over using only the earlier scan projection because obstacle
marking no longer depends on compressing the Livox 3D cloud into a single 2D
LaserScan. The projection node filters the cloud in `base_link`, preserves the
obstacle points as PointCloud2 data for Nav2, and leaves `/scan_from_livox`
focused on AMCL where a 2D scan is still useful.

Start the Livox driver on the Jetson:

```bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

Then start the Jetson upgraded costmap stack:

```bash
ros2 launch bringup costmap_projection.launch.py
```

This produces:

```text
/livox/lidar                  -> FAST-LIO2                         -> /fastlio_odom
/fastlio_odom                 -> fastlio2_nav2_adapter             -> /nav2_odom
/livox/lidar                  -> custom_fastlio_preprocess         -> /custom/points_preprocessed
/custom/points_preprocessed   -> custom_livox_costmap_projection   -> /custom/obstacle_cloud
/custom/points_preprocessed   -> custom_livox_costmap_projection   -> /custom/projected_occupancy_grid
/livox/lidar                  -> custom_fastlio_preprocess         -> /custom/points_for_nav2
/custom/points_for_nav2       -> livox_cloud_to_scan               -> /scan_from_livox
fastlio2_nav2_adapter                                              -> odom -> base_link
static_transform_publisher                                         -> base_link -> livox_frame
```

Then start Nav2 on the Pi with the upgraded costmap parameters:

```bash
ros2 launch bringup pi_fast_lio2_costmap_projection_nav2.launch.py map:=/home/rmahajani/Documents/projects/rover/ros2_ws/maps/rover_map.yaml
```

Before sending a goal, verify the Jetson outputs:

```bash
ros2 topic hz /custom/points_preprocessed
ros2 topic hz /custom/obstacle_cloud
ros2 topic hz /custom/projected_occupancy_grid
ros2 topic echo /custom/costmap_projection_diagnostics --once
ros2 topic hz /scan_from_livox
ros2 topic hz /nav2_odom
```

Expected topic usage:

```text
/scan_from_livox      -> AMCL
/custom/obstacle_cloud -> Nav2 local costmap and collision monitor
```

Run order summary:

```bash
# Jetson
ros2 launch livox_ros_driver2 msg_MID360_launch.py
ros2 launch bringup costmap_projection.launch.py

# Pi
ros2 launch bringup pi_fast_lio2_costmap_projection_nav2.launch.py \
  map:=/home/rmahajani/Documents/projects/rover/ros2_ws/maps/rover_map.yaml
```

### Local Map Management Mode

Local map management mode uses scan-to-map odometry instead of FAST-LIO2 while
keeping the upgraded costmap obstacle pipeline. In the IMU integration phase,
the Jetson also consumes `/livox/imu` and uses rotation-only IMU propagation as
the initial guess for lidar registration. The Jetson owns the local map,
publishes `/nav2_odom`, broadcasts `odom -> base_link`, projects Livox obstacle
points into `/custom/obstacle_cloud`, and keeps `/scan_from_livox` available
for AMCL. The Pi runs Nav2 with the costmap-projection parameters and consumes
the Jetson topics.

Start the Livox driver on the Jetson:

```bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

Then start local map management mode on the Jetson:

```bash
ros2 launch bringup scan_to_map_costmap_projection.launch.py
```

This produces:

```text
/livox/lidar                  -> custom_fastlio_preprocess         -> /custom/points_preprocessed
/livox/lidar                  -> custom_fastlio_preprocess         -> /custom/points_for_nav2
/livox/imu                    -> custom_scan_to_map_odom           -> IMU initial guess
/custom/points_preprocessed   -> custom_scan_to_map_odom           -> /nav2_odom
/custom/points_preprocessed   -> custom_scan_to_map_odom           -> /custom/imu_predicted_odom
/custom/points_preprocessed   -> custom_livox_costmap_projection   -> /custom/obstacle_cloud
/custom/points_preprocessed   -> custom_livox_costmap_projection   -> /custom/projected_occupancy_grid
/custom/points_for_nav2       -> livox_cloud_to_scan               -> /scan_from_livox
custom_scan_to_map_odom                                            -> odom -> base_link
static_transform_publisher                                         -> base_link -> livox_frame
```

On the Pi, start Nav2 with the upgraded costmap parameters:

```bash
ros2 launch bringup pi_fast_lio2_costmap_projection_nav2.launch.py map:=/home/rmahajani/Documents/projects/rover/ros2_ws/maps/rover_map.yaml
```

Before sending a goal, verify:

```bash
ros2 topic hz /custom/points_preprocessed
ros2 topic hz /livox/imu
ros2 topic hz /custom/obstacle_cloud
ros2 topic hz /custom/local_map
ros2 topic hz /custom/imu_predicted_odom
ros2 topic hz /scan_from_livox
ros2 topic hz /nav2_odom
ros2 topic echo /nav2_odom --once
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link livox_frame
```

Expected topic usage:

```text
/nav2_odom           -> Nav2 odometry
/custom/local_map    -> RViz inspection of the scan-to-map backend map
/custom/imu_predicted_odom -> RViz/debug inspection of the IMU initial guess
/scan_from_livox     -> AMCL
/custom/obstacle_cloud -> Nav2 local costmap and collision monitor
```

Run order summary:

```bash
# Jetson
ros2 launch livox_ros_driver2 msg_MID360_launch.py
ros2 launch bringup scan_to_map_costmap_projection.launch.py

# Pi
ros2 launch bringup pi_fast_lio2_costmap_projection_nav2.launch.py map:=/home/rmahajani/Documents/projects/rover/ros2_ws/maps/rover_map.yaml
```

### IMU Point Cloud Deskew Mode

IMU point cloud deskew mode is the preferred scan-to-map testing path when
turning motion is expected. The Jetson still runs custom Livox preprocessing,
scan-to-map odometry, obstacle projection, `/scan_from_livox`, and
`odom -> base_link`, but scan-to-map registration consumes
`/custom/deskewed_points` instead of the raw preprocessed odometry cloud.
Obstacle projection continues to use `/custom/points_preprocessed` so the
costmap behavior remains compatible with previous modes.

Start the Livox driver on the Jetson:

```bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

Then start the Jetson deskew scan-to-map stack and capture logs:

```bash
ros2 launch bringup scan_to_map_deskew_costmap_projection.launch.py
```

This produces:

```text
/livox/lidar                  -> custom_fastlio_preprocess         -> /custom/points_preprocessed
/livox/lidar                  -> custom_fastlio_preprocess         -> /custom/points_for_nav2
/livox/lidar                  -> custom_fastlio_preprocess         -> /custom/points_for_deskew
/livox/imu                    -> custom_lidar_deskew               -> deskew rotation
/custom/points_for_deskew     -> custom_lidar_deskew               -> /custom/deskewed_points
/custom/deskewed_points       -> custom_scan_to_map_odom           -> /nav2_odom
/custom/deskewed_points       -> custom_scan_to_map_odom           -> /custom/imu_predicted_odom
/custom/points_preprocessed   -> custom_livox_costmap_projection   -> /custom/obstacle_cloud
/custom/points_preprocessed   -> custom_livox_costmap_projection   -> /custom/projected_occupancy_grid
/custom/points_for_nav2       -> livox_cloud_to_scan               -> /scan_from_livox
custom_scan_to_map_odom                                            -> odom -> base_link
static_transform_publisher                                         -> base_link -> livox_frame
```

On the Pi, start Nav2 with the upgraded costmap parameters and saved map:

```bash
ros2 launch bringup pi_fast_lio2_costmap_projection_nav2.launch.py map:=/home/rmahajani/Documents/projects/rover/ros2_ws/maps/rover_map.yaml
```

Before sending a goal, verify:

```bash
ros2 topic hz /custom/points_for_deskew
ros2 topic hz /custom/deskewed_points
ros2 topic echo /custom/deskew/diagnostics --once
ros2 topic hz /custom/obstacle_cloud
ros2 topic hz /scan_from_livox
ros2 topic hz /nav2_odom
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link livox_frame
```

Expected topic usage:

```text
/custom/deskewed_points -> scan-to-map odometry
/custom/points_preprocessed -> Livox obstacle projection
/custom/obstacle_cloud -> Nav2 local costmap and collision monitor
/scan_from_livox -> AMCL
```

Run order summary:

```bash
# Jetson
ros2 launch livox_ros_driver2 msg_MID360_launch.py
ros2 launch bringup scan_to_map_deskew_costmap_projection.launch.py

# Pi
ros2 launch bringup pi_fast_lio2_costmap_projection_nav2.launch.py map:=/home/rmahajani/Documents/projects/rover/ros2_ws/maps/rover_map.yaml
```

### Error-State Kalman Filter Mode

Error-state Kalman filter mode is the current rover odometry path for Nav2
testing with the Livox MID-360. The Jetson runs custom Livox preprocessing,
IMU point cloud deskewing, the EKF odometry backend, obstacle projection, and
`/scan_from_livox`. The Pi runs Nav2 with the costmap-projection parameters and
consumes `/nav2_odom` from the Jetson.

The EKF consumes `/custom/deskewed_points` and `/livox/imu`, maintains a local
LiDAR map, and publishes only LiDAR-corrected poses to Nav2. It does not
publish intermediate IMU-only odometry.

The current rover configuration uses ikd tree map backend mode:
`local_map.backend_type: ikd_tree` in `custom_lio_ekf/config/lio_ekf.yaml`.
This backend keeps a voxel-downsampled local map, serves point-to-plane
nearest-neighbor queries from an incremental kd-tree, and reports backend
timing in `/custom/lio_ekf_diagnostics`. Use `pcl_rebuild` only when comparing
against the rebuild-style baseline.

Start the Livox driver on the Jetson:

```bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

Then start the Jetson error-state Kalman filter stack:

```bash
ros2 launch bringup lio_ekf_deskew_costmap_projection.launch.py
```

This produces:

```text
/livox/lidar                  -> custom_fastlio_preprocess         -> /custom/points_preprocessed
/livox/lidar                  -> custom_fastlio_preprocess         -> /custom/points_for_nav2
/livox/lidar                  -> custom_fastlio_preprocess         -> /custom/points_for_deskew
/livox/imu                    -> custom_lidar_deskew               -> deskew rotation
/custom/points_for_deskew     -> custom_lidar_deskew               -> /custom/deskewed_points
/custom/deskewed_points       -> custom_lio_ekf + ikd tree backend -> /nav2_odom
/custom/points_preprocessed   -> custom_livox_costmap_projection   -> /custom/obstacle_cloud
/custom/points_for_nav2       -> livox_cloud_to_scan               -> /scan_from_livox
custom_lio_ekf                                                     -> odom -> base_link
static_transform_publisher                                         -> base_link -> livox_frame
```

On the Pi, start Nav2 with the upgraded costmap parameters and saved map:

```bash
ros2 launch bringup pi_fast_lio2_costmap_projection_nav2.launch.py map:=/home/rmahajani/Documents/projects/rover/ros2_ws/maps/rover_map.yaml
```

Before sending a goal, verify:

```bash
ros2 topic hz /custom/deskewed_points
ros2 topic hz /custom/obstacle_cloud
ros2 topic hz /scan_from_livox
ros2 topic hz /nav2_odom
ros2 topic echo /nav2_odom --once
ros2 topic echo /custom/lio_ekf_diagnostics --once
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link livox_frame
```

Expected topic usage:

```text
/nav2_odom -> Nav2 odometry from LiDAR-corrected EKF poses
/custom/deskewed_points -> EKF LiDAR correction input
/custom/points_preprocessed -> Livox obstacle projection
/custom/obstacle_cloud -> Nav2 local costmap and collision monitor
/scan_from_livox -> AMCL
/custom/lio_ekf_diagnostics -> EKF and map backend timing/status
```

Run order summary:

```bash
# Jetson
ros2 launch livox_ros_driver2 msg_MID360_launch.py
ros2 launch bringup lio_ekf_deskew_costmap_projection.launch.py

# Pi
ros2 launch bringup pi_fast_lio2_costmap_projection_nav2.launch.py map:=/home/rmahajani/Documents/projects/rover/ros2_ws/maps/rover_map.yaml
```

### Scan-To-Map Registration Based Odometry

In this mode, scan-to-map registration becomes the odometry source used by
Nav2. The current Jetson workflow uses the same launch as local map management:
custom Livox preprocessing, scan-to-map odometry, costmap obstacle projection,
and `/scan_from_livox` for AMCL. The IMU integration phase adds a rotation-only
IMU initial guess before lidar registration. The Pi runs Nav2 with the
costmap-projection parameters and the motor-command path.

Start the Livox driver on the Jetson in CustomMsg mode:

```bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

Then start the Jetson scan-to-map costmap projection stack:

```bash
ros2 launch bringup scan_to_map_costmap_projection.launch.py
```

This produces:

```text
/livox/lidar                  -> custom_fastlio_preprocess -> /custom/points_preprocessed
/livox/lidar                  -> custom_fastlio_preprocess -> /custom/points_for_nav2
/livox/imu                    -> custom_scan_to_map_odom   -> IMU initial guess
/custom/points_preprocessed   -> custom_scan_to_map_odom   -> /nav2_odom
/custom/points_preprocessed   -> custom_scan_to_map_odom   -> /custom/imu_predicted_odom
/custom/points_preprocessed   -> custom_livox_costmap_projection -> /custom/obstacle_cloud
/custom/points_for_nav2       -> livox_cloud_to_scan       -> /scan_from_livox
custom_scan_to_map_odom                                    -> odom -> base_link
static_transform_publisher                                 -> base_link -> livox_frame
```

On the Pi, start Nav2 with the upgraded costmap parameters and saved map:

```bash
ros2 launch bringup pi_fast_lio2_costmap_projection_nav2.launch.py \
  map:=/home/rmahajani/Documents/projects/rover/ros2_ws/maps/rover_map.yaml
```

The Pi launch name still refers to FAST-LIO2, but in this mode it consumes
scan-to-map `/nav2_odom` from the Jetson and does not start `rover_odometry`.

Before sending a goal, verify:

```bash
ros2 topic hz /nav2_odom
ros2 topic hz /scan_from_livox
ros2 topic hz /custom/points_preprocessed
ros2 topic hz /custom/obstacle_cloud
ros2 topic hz /livox/imu
ros2 topic hz /custom/imu_predicted_odom
ros2 topic echo /nav2_odom --once
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link livox_frame
```

`/nav2_odom` should use `header.frame_id: odom` and
`child_frame_id: base_link`. Exactly one node should publish
`odom -> base_link`.

### Custom Scan-to-Scan ICP Shadow Mode

This mode runs the custom ICP odometry node on the Jetson while the Pi continues
to drive, map, or navigate using the stable odometry source. It is for debugging
and comparison only.

Start the Livox driver on the Jetson in CustomMsg mode:

```bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

Then start the Jetson-side scan-to-scan ICP launch:

```bash
ros2 launch bringup scan_to_scan_icp.launch.py
```

This produces:

```text
/livox/lidar                  -> custom_fastlio_preprocess -> /custom/points_preprocessed
/custom/points_preprocessed   -> custom_icp_odom           -> /custom/icp_odom
/custom/points_preprocessed   -> custom_icp_odom           -> /custom/icp_path
/custom/points_preprocessed   -> custom_icp_odom           -> /custom/icp/* debug clouds
```

`scan_to_scan_icp.launch.py` forces `publish_tf: false`, so it should not
conflict with the odometry source used by mapping or Nav2.

On the Pi, run the normal mapping launch when you want to drive controlled test
patterns:

```bash
ros2 launch bringup mapping.launch.py
```

In RViz, set the fixed frame to `odom` and add:

```text
Path       /custom/icp_path
Odometry   /custom/icp_odom
PointCloud2 /custom/icp/aligned_cloud
PointCloud2 /custom/icp/source_cloud
PointCloud2 /custom/icp/target_cloud
```

Use controlled tests in this order:

```text
1. Still rover for 30-60 seconds.
2. Slow 0.5 m forward and backward.
3. Slow yaw left and right.
4. Small square path.
```

Expected behavior is directionally correct motion without large jumps. Drift is
normal for scan-to-scan odometry, especially during turns and in repetitive
geometry.

## Saving Maps

During mapping, once a usable map is visualized in RViz, save it with:

```bash
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '/home/rmahajani/Documents/projects/rover/ros2_ws/maps/rover_map'}}"
```

This creates the saved map files in `ros2_ws/maps/`. The generated YAML map file
is then passed into the navigation launch file.

## Hardware Notes

- The ESP32 runs the rover firmware from
`ros2_ws/firmware/freertos_apps/apps/rover/app.c`.
- The Pi talks to the ESP32 over USB serial using micro-ROS. In this setup, the
ESP32 device is configured as `/dev/tty_rover_esp`.
- The goBILDA Pinpoint v2 odometry computer is read over I2C bus `1` at address
`0x31`.
- The Livox Mid-360 publishes 3D point clouds through `livox_ros_driver2`.
- The Jetson projects Livox point clouds from `/livox/lidar` into
`/scan_from_livox`.
- In FAST-LIO2 shadow mode, `/livox/lidar` is a Livox CustomMsg topic and
`/livox/points` is the converted PointCloud2 topic used for scan projection.
- In FAST-LIO2 Nav2 odometry mode, the same CustomMsg conversion path is used,
and `fastlio2_nav2_adapter` publishes `/nav2_odom` plus `odom -> base_link`.
- In the custom preprocessing mode, `/livox/lidar` remains a Livox CustomMsg
topic, `custom_fastlio_preprocess` publishes `/custom/points_preprocessed` and
`/custom/points_for_nav2`, and `livox_cloud_to_scan` projects
`/custom/points_for_nav2` into `/scan_from_livox`.
- In IMU point cloud deskew mode, `custom_fastlio_preprocess` also publishes
`/custom/points_for_deskew`; `custom_lidar_deskew` consumes that cloud and
`/livox/imu`, then publishes `/custom/deskewed_points` for scan-to-map
odometry.
- In error-state Kalman filter mode, `custom_lio_ekf` consumes
`/custom/deskewed_points` and `/livox/imu`, publishes `/nav2_odom`, and
broadcasts `odom -> base_link` from LiDAR-corrected EKF poses. The current
configuration uses `local_map.backend_type: ikd_tree` for ikd tree map backend
mode.
- In upgraded costmap mode, `custom_livox_costmap_projection` consumes
`/custom/points_preprocessed`, publishes `/custom/obstacle_cloud` for Nav2
costmaps, and publishes `/custom/projected_occupancy_grid` for RViz/debugging.
`/scan_from_livox` remains available for AMCL localization.
- In local map management mode, `custom_scan_to_map_odom` consumes
`/custom/points_preprocessed`, maintains `/custom/local_map`, publishes
`/nav2_odom`, and broadcasts `odom -> base_link` while
`custom_livox_costmap_projection` publishes `/custom/obstacle_cloud` for Nav2.
- In the IMU integration phase, `custom_scan_to_map_odom` also consumes
`/livox/imu` and uses rotation-only propagation as the scan-to-map optimizer
initial guess. The debug topic `/custom/imu_predicted_odom` shows that
prediction before lidar correction.
- In custom scan-to-scan ICP shadow mode, `custom_icp_odom` consumes
`/custom/points_preprocessed` and publishes `/custom/icp_odom` plus
`/custom/icp_path` without publishing TF.
- In scan-to-map registration based odometry mode, `custom_scan_to_map_odom`
consumes `/custom/points_preprocessed`, publishes `/nav2_odom`, and broadcasts
`odom -> base_link`.
- In IMU point cloud deskew mode, `custom_scan_to_map_odom` consumes
`/custom/deskewed_points`, while `custom_livox_costmap_projection` continues
to consume `/custom/points_preprocessed` for obstacles.
- `jetson.launch.py` publishes a static transform from `base_link` to
`livox_frame` with the lidar mounted `0.3175 m` above `base_link`.

## Troubleshooting

- If mapping or navigation cannot command the rover, confirm the micro-ROS agent
is running and connected to the ESP32 USB serial device.
- If the configured ESP32 device path is missing, check the ESP32 USB connection
and any udev rule that creates the stable device name.
- If the rover does not respond to the gamepad, check that `/dev/input/js0`
exists and that `/joy` is publishing.
- If no laser scan appears, confirm Livox data is publishing on `/livox/lidar`
and that `livox_cloud_to_scan` is publishing `/scan_from_livox`.
- In FAST-LIO2 shadow mode, also confirm `/livox/points` is publishing. If a
bag recorder reports unknown type `livox_ros_driver2/msg/CustomMsg`, source
the Livox workspace before recording.
- In FAST-LIO2 Nav2 mode, confirm `/nav2_odom` is publishing and that no other
node is also publishing `odom -> base_link`.
- In custom preprocessing mode, confirm `/custom/points_for_nav2` is non-empty
before debugging `/scan_from_livox`.
- In upgraded costmap mode, confirm `/custom/obstacle_cloud`,
`/custom/projected_occupancy_grid`, `/custom/costmap_projection_diagnostics`,
`/scan_from_livox`, and `/nav2_odom` are publishing before sending a Nav2 goal.
- In local map management mode, confirm `/custom/local_map`,
`/custom/obstacle_cloud`, `/livox/imu`, `/custom/imu_predicted_odom`,
`/scan_from_livox`, and `/nav2_odom` are publishing before sending a Nav2
goal.
- In IMU point cloud deskew mode, confirm `/custom/points_for_deskew`,
`/custom/deskewed_points`, `/custom/deskew/diagnostics`,
`/custom/obstacle_cloud`, `/scan_from_livox`, and `/nav2_odom` are publishing
before sending a Nav2 goal.
- In error-state Kalman filter mode, confirm `/custom/deskewed_points`,
`/custom/obstacle_cloud`, `/scan_from_livox`, `/nav2_odom`, and
`/custom/lio_ekf_diagnostics` are publishing before sending a Nav2 goal.
- In ikd tree map backend mode, inspect `/custom/lio_ekf_diagnostics` keys such
as `map_backend_lidar_knn_time_ms`, `map_backend_update_total_time_ms`,
`map_backend_update_rebuild_time_ms`, and `consecutive_tracking_failures` if
Nav2 odometry slows or tracking looks unstable.
- To compare against the rebuild baseline, set `local_map.backend_type` to
`pcl_rebuild` in `custom_lio_ekf/config/lio_ekf.yaml`; set it back to
`ikd_tree` for the current rover run.
- If Nav2 starts but obstacles do not appear in the costmaps in upgraded
costmap mode, confirm the Pi was launched with
`pi_fast_lio2_costmap_projection_nav2.launch.py`, not one of the plain
FAST-LIO2 Nav2 launches.
- If `/custom/projected_occupancy_grid` is missing but `/custom/obstacle_cloud`
is publishing, check `/custom/costmap_projection_diagnostics` for
`tf_target_to_grid_success` and verify `odom -> base_link` is available at the
cloud timestamps.
- In scan-to-map registration based odometry mode, confirm
`/custom/points_preprocessed`, `/nav2_odom`, and `/scan_from_livox` are
publishing on the Jetson before starting Nav2 on the Pi.
- In local map management mode, use `scan_to_map_costmap_projection.launch.py`
on the Jetson and `pi_fast_lio2_costmap_projection_nav2.launch.py` on the Pi.
The Pi launch name still refers to FAST-LIO2, but in this mode it consumes
scan-to-map `/nav2_odom` from the Jetson.
- In IMU point cloud deskew mode, use
`scan_to_map_deskew_costmap_projection.launch.py` on the Jetson and
`pi_fast_lio2_costmap_projection_nav2.launch.py` on the Pi. The Pi launch name
still refers to FAST-LIO2, but it consumes `/nav2_odom` from the Jetson
scan-to-map stack.
- In error-state Kalman filter mode, use
`lio_ekf_deskew_costmap_projection.launch.py` on the Jetson and
`pi_fast_lio2_costmap_projection_nav2.launch.py` on the Pi. The Pi launch name
still refers to FAST-LIO2, but it consumes EKF `/nav2_odom` from the Jetson.
- If `/custom/deskewed_points` is missing, confirm `/custom/points_for_deskew`
and `/livox/imu` are publishing and inspect `/custom/deskew/diagnostics` for
`has_point_time`, `imu_coverage_ok`, and `deskew_success`.
- If scan-to-map odometry publishes but Nav2 reports TF problems, confirm
there is exactly one `odom -> base_link` publisher and that
`base_link -> livox_frame` is available.
- If the IMU predicted pose is missing, confirm the Livox driver is publishing
`/livox/imu` before starting the scan-to-map launch.
- If scan-to-map odometry degrades during turns, slow the rover and inspect
`/custom/local_map` and `/custom/scan_to_map_path` in RViz. Large odometry
jumps usually indicate that the scan registration has lost a reliable local map
match.
- In scan-to-scan ICP shadow mode, confirm `/custom/points_preprocessed`,
`/custom/icp_odom`, and `/custom/icp_path` are publishing. If the ICP node
starts but no odometry appears, check that the Jetson sourced the workspace
containing `livox_ros_driver2` before launching the preprocessor.
- If `/custom/icp_path` drifts while the rover is still, compare GICP and
point-to-point ICP with `use_gicp` in `custom_icp_odom/config/icp_odom.yaml`.
Some drift is expected from scan-to-scan odometry; GICP is the preferred current
mode.
- If `/custom/points_for_nav2` is visible but `/scan_from_livox` contains only
NaN ranges, check that the preprocessor height band still maps into the
existing `livox_cloud_to_scan` height slice after the `base_link -> livox_frame`
static transform.
- If Nav2 reports the sensor origin is out of costmap bounds, confirm
`force_planar_output` is true in `fastlio2_nav2_adapter.yaml` and that
`odom -> base_link` has `z = 0`.
- If slam_toolbox does not build a map, check that `/scan_from_livox`, `odom`,
and the `map -> odom -> base_footprint -> base_link` TF chain are valid.
- If map saving fails, make sure the `ros2_ws/maps/` directory exists and that
slam_toolbox is running.
- If Nav2 fails to start, confirm the `map:=...rover_map.yaml` path points to
an existing saved map YAML file.

## Typical Startup Order

1. Start the micro-ROS agent on the Pi:

   ```bash
   ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/tty_rover_esp
   ```

2. Start the Jetson lidar processing launch file:

   ```bash
   ros2 launch bringup jetson.launch.py
   ```

3. For FAST-LIO2 shadow mode instead of normal Jetson processing, start:

   ```bash
   ros2 launch livox_ros_driver2 msg_MID360_launch.py
   ros2 launch bringup fastlio_shadow.launch.py
   ```

4. For FAST-LIO2 odometry-based Nav2, start this on the Jetson:

   ```bash
   ros2 launch livox_ros_driver2 msg_MID360_launch.py
   ros2 launch bringup fast_lio2_nav2.launch.py
   ```

5. For FAST-LIO2 odometry with custom Livox preprocessing, start this on the Jetson:

   ```bash
   ros2 launch livox_ros_driver2 msg_MID360_launch.py
   ros2 launch bringup pointcloud_preprocess_fast_lio2_nav2.launch.py
   ```

6. For upgraded costmap mode:

   ```bash
   # Jetson
   ros2 launch livox_ros_driver2 msg_MID360_launch.py
   ros2 launch bringup costmap_projection.launch.py

   # Pi
   ros2 launch bringup pi_fast_lio2_costmap_projection_nav2.launch.py \
     map:=/home/rmahajani/Documents/projects/rover/ros2_ws/maps/rover_map.yaml
   ```

7. For IMU-integrated local map management mode:

   ```bash
   # Jetson
   ros2 launch livox_ros_driver2 msg_MID360_launch.py
   ros2 launch bringup scan_to_map_costmap_projection.launch.py

   # Pi
   ros2 launch bringup pi_fast_lio2_costmap_projection_nav2.launch.py \
     map:=/home/rmahajani/Documents/projects/rover/ros2_ws/maps/rover_map.yaml
   ```

8. For IMU point cloud deskew mode:

   ```bash
   # Jetson
   ros2 launch livox_ros_driver2 msg_MID360_launch.py
   ros2 launch bringup scan_to_map_deskew_costmap_projection.launch.py

   # Pi
   ros2 launch bringup pi_fast_lio2_costmap_projection_nav2.launch.py map:=/home/rmahajani/Documents/projects/rover/ros2_ws/maps/rover_map.yaml
   ```

9. For error-state Kalman filter mode with ikd tree map backend mode:

   ```bash
   # Jetson
   ros2 launch livox_ros_driver2 msg_MID360_launch.py
   ros2 launch bringup lio_ekf_deskew_costmap_projection.launch.py

   # Pi
   ros2 launch bringup pi_fast_lio2_costmap_projection_nav2.launch.py map:=/home/rmahajani/Documents/projects/rover/ros2_ws/maps/rover_map.yaml
   ```

10. For custom scan-to-scan ICP shadow mode, start this on the Jetson:

   ```bash
   ros2 launch livox_ros_driver2 msg_MID360_launch.py
   ros2 launch bringup scan_to_scan_icp.launch.py
   ```

11. For mapping, launch:

   ```bash
   ros2 launch bringup mapping.launch.py
   ```

12. For autonomous navigation with wheel odometry, provide the saved map:

   ```bash
   ros2 launch bringup pi_nav2_livox.launch.py \
     map:=/home/rmahajani/Documents/projects/rover/ros2_ws/maps/rover_map.yaml
   ```

13. For autonomous navigation with Jetson odometry on `/nav2_odom`, provide the saved map:

   ```bash
   ros2 launch bringup pi_fast_lio2_nav2_main.launch.py \
     map:=/home/rmahajani/Documents/projects/rover/ros2_ws/maps/rover_map.yaml
   ```
