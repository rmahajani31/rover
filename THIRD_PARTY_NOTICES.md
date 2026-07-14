# Third-Party Notices

This repository's original code is licensed under the MIT License. Some rover
operating modes integrate with third-party ROS 2 packages that have their own
licenses.

## FAST_LIO_ROS2

FAST-LIO2 operating modes use `fast_lio` from
[Ericsii/FAST_LIO_ROS2](https://github.com/Ericsii/FAST_LIO_ROS2) as an
external ROS 2 package.

- Upstream license: GNU General Public License v2.0
- Upstream license file:
  [FAST_LIO_ROS2/LICENSE](https://github.com/Ericsii/FAST_LIO_ROS2/blob/ros2/LICENSE)
- Used by launch/configuration paths such as FAST-LIO2 shadow mode,
  FAST-LIO2 Nav2 odometry mode, FAST-LIO2 Nav2 odometry with custom Livox
  preprocessing, and upgraded costmap mode with FAST-LIO2 odometry.
- This repository does not vendor the FAST_LIO_ROS2 source code. Users should
  obtain FAST_LIO_ROS2 from the upstream project and comply with its license.

The file `ros2_ws/src/bringup/config/mid360_fastlio2.yaml` is adapted from
FAST_LIO_ROS2 MID-360-style configuration parameters for this rover's Livox
Mid-360 setup.

If distributing a combined workspace, image, or binary artifact that includes
FAST_LIO_ROS2, include the applicable GPLv2 license notices and corresponding
source for the GPLv2-covered FAST_LIO_ROS2 components.
