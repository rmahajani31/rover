# Rover

Code to operate a homemade ROS 2 rover with an ESP32 motor controller, a
Raspberry Pi, and a Jetson processing Livox Mid-360 lidar data.

## System Overview

This repository is organized around a split compute stack:

- The ESP32 runs the low-level microcontroller firmware.
- The Raspberry Pi runs the main rover ROS 2 system on ROS 2 Kilted.
- The Jetson runs lidar point cloud processing on ROS 2 Humble.

The Jetson is currently used for Livox point cloud projection. It is included
in the architecture so heavier lidar point cloud processing and future
camera-based visual processing can be added there as the rover stack grows.

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
- `fastlio2_nav2_adapter`, when using FAST-LIO2 as the Nav2 odometry source
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
| Raspberry Pi | Runs the micro-ROS agent, gamepad control, mapping, localization, Nav2, and motor-command path. In the FAST-LIO2 Nav2 mode, it consumes `/nav2_odom` from the Jetson instead of starting `rover_odometry`.      |
| Jetson       | Runs Livox point cloud processing, FAST-LIO2, `/scan_from_livox`, and the FAST-LIO2-to-Nav2 odometry adapter.                                                                                                  |


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
- `pi_fast_lio2_nav2.launch.py` starts Pi-side Nav2 localization and navigation
using `/nav2_odom` from the Jetson.
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

In normal Jetson mode, `/livox/lidar` is expected to be a `PointCloud2` topic.
In FAST-LIO2 shadow mode, `/livox/lidar` is kept as a Livox `CustomMsg` for
FAST-LIO2, and a small converter publishes `/livox/points` as `PointCloud2` so
the existing scan projection path can still publish `/scan_from_livox`.

The package can also build without `livox_ros_driver2`; in that case the
Livox CustomMsg converter is skipped. This is useful on the Pi, where the
driver and converter are not part of the runtime path.

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
| `/livox/points`    | Topic           | PointCloud2 copy of Livox CustomMsg data used only in FAST-LIO2 shadow mode. |
| `/scan_from_livox` | Topic           | 2D scan output from `livox_cloud_to_scan`; used by slam_toolbox and Nav2. |
| `/fastlio_odom`    | Topic           | FAST-LIO2 odometry output for observation, bagging, and adapter input.     |
| `/nav2_odom`       | Topic           | Nav2 odometry topic republished by `fastlio2_nav2_adapter`.                |
| `/joy`             | Topic           | Gamepad input from `joy_node`.                                            |
| `/cmd_vel`         | Topic           | Rover velocity command published by `gamepad_adapter`.                    |
| `odom`             | Topic and frame | Wheel odometry topic in the rover-odometry path; odom frame in both Nav2 paths. |
| `map`              | Frame           | Global map frame used by slam_toolbox and Nav2.                           |
| `base_footprint`   | Frame           | Odometry child frame from `rover_odometry`.                               |
| `base_link`        | Frame           | Main rover body frame.                                                    |
| `livox_frame`      | Frame           | Livox lidar frame; statically published from `base_link`.                 |


## Operating Modes

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

### Mapping

Before mapping, start the micro-ROS agent on the Pi and the Livox processing on
the Jetson. Then run mapping on the Pi:

```bash
ros2 launch bringup mapping.launch.py
```

This launch file starts gamepad control, rover odometry, the
`base_footprint -> base_link` static transform, and slam_toolbox.

### FAST-LIO2 Shadow Mode

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

### FAST-LIO2 Nav2 Odometry

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
ros2 launch bringup pi_fast_lio2_nav2.launch.py map:=/home/rmahajani/Documents/projects/rover/ros2_ws/maps/rover_map.yaml
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

### Navigation

Autonomous navigation uses Nav2 with a saved map:

```bash
ros2 launch bringup pi_nav2_livox.launch.py map:=/home/rmahajani/Documents/projects/rover/ros2_ws/maps/rover_map.yaml | tee out/output.txt
```

This launch file starts rover odometry, the `base_footprint -> base_link`
static transform, Nav2 localization, and Nav2 navigation.

For FAST-LIO2 odometry-based navigation, use:

```bash
ros2 launch bringup pi_fast_lio2_nav2.launch.py map:=/home/rmahajani/Documents/projects/rover/ros2_ws/maps/rover_map.yaml
```

This launch uses `/nav2_odom` from the Jetson-side adapter and does not start
the wheel odometry node.

## Saving Maps

During the mapping phase, once a usable map is visualized in RViz, save it with:

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
5. For mapping, launch:
  ```bash
   ros2 launch bringup mapping.launch.py
  ```
6. For autonomous navigation with wheel odometry, provide the saved map:
  ```bash
   ros2 launch bringup pi_nav2_livox.launch.py map:=/home/rmahajani/Documents/projects/rover/ros2_ws/maps/rover_map.yaml
  ```
7. For autonomous navigation with FAST-LIO2 odometry, provide the saved map:
  ```bash
   ros2 launch bringup pi_fast_lio2_nav2.launch.py map:=/home/rmahajani/Documents/projects/rover/ros2_ws/maps/rover_map.yaml
  ```
