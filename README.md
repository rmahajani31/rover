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
- `fast_lio` from
  [Ericsii/FAST_LIO_ROS2](https://github.com/Ericsii/FAST_LIO_ROS2), when
  running FAST-LIO2 shadow mode

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
| Raspberry Pi | Runs the micro-ROS agent, odometry, gamepad control, mapping, localization, and Nav2.                                                                                                                           |
| Jetson       | Runs Livox point cloud processing and projects 3D lidar data into 2D laser scan data. The Jetson is used here to leave room for heavier lidar processing and a future camera-based visual processing component. |


## ROS 2 Packages

### `bringup`

Main package for launch files used by the rover stack.

Important launch files:

- `jetson.launch.py` starts the Jetson-side processing that projects 3D point
cloud data into 2D scan data.
- `fastlio_shadow.launch.py` starts FAST-LIO2 beside the existing scan
projection path. Nav2 continues using stable rover odometry while FAST-LIO2
publishes `/fastlio_odom` for observation and bagging.
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

### `rover_odometry`

Package responsible for rover odometry. Odometry is broadcast over I2C to the
Pi from the goBILDA Pinpoint v2 odometry computer.

## Important Topics and Frames


| Name               | Type            | Notes                                                                     |
| ------------------ | --------------- | ------------------------------------------------------------------------- |
| `/livox/lidar`     | Topic           | Livox input from `livox_ros_driver2`.                                |
| `/livox/points`    | Topic           | PointCloud2 copy of Livox CustomMsg data used only in FAST-LIO2 shadow mode. |
| `/scan_from_livox` | Topic           | 2D scan output from `livox_cloud_to_scan`; used by slam_toolbox and Nav2. |
| `/fastlio_odom`    | Topic           | FAST-LIO2 odometry output for shadow-mode observation and bagging.         |
| `/joy`             | Topic           | Gamepad input from `joy_node`.                                            |
| `/cmd_vel`         | Topic           | Rover velocity command published by `gamepad_adapter`.                    |
| `odom`             | Topic and frame | Odometry output from `rover_odometry`.                                    |
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

### Navigation

Autonomous navigation uses Nav2 with a saved map:

```bash
ros2 launch bringup pi_nav2_livox.launch.py map:=/home/rmahajani/Documents/projects/rover/ros2_ws/maps/rover_map.yaml | tee out/output.txt
```

This launch file starts rover odometry, the `base_footprint -> base_link`
static transform, Nav2 localization, and Nav2 navigation.

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
4. For mapping, launch:
  ```bash
   ros2 launch bringup mapping.launch.py
  ```
5. For autonomous navigation with Nav2, provide the saved map:
  ```bash
   ros2 launch bringup pi_nav2_livox.launch.py map:=/home/rmahajani/Documents/projects/rover/ros2_ws/maps/rover_map.yaml
  ```
