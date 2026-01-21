# HERON CONTROLLER

**H**ydrodynamic **C**ommand **A**llocation and **D**ynamics **C**ontrol.

[![ROS](https://img.shields.io/badge/ROS-Noetic-blue)](http://wiki.ros.org/noetic)
[![License](https://img.shields.io/badge/license-BSD-lightgrey)]()

## Abstract

The **Heron Controller** provides the low-level C++ control loops for the Heron USV. It is responsible for mapping high-level wrench commands (Force X, Torque Z) into individual thruster setpoints, accounting for the differential drive geometry and hydrodynamic drag.

## Features

*   **Force Allocation**: Differential mixing for non-holonomic thrust.
*   **Helm Interface**: Accepts standard `geometry_msgs/Twist` or `cmd_helm` inputs.
*   **Status Reporting**: Publishes battery, temperature, and link status.

## Node: `controller`

### Subscribed Topics
*   `cmd_helm` (heron_msgs/Helm)
*   `cmd_wrench` (geometry_msgs/Wrench)
*   `imu/data` (sensor_msgs/Imu)

### Published Topics
*   `cmd_drive` (heron_msgs/Drive): Raw left/right thruster commands.
*   `status` (heron_msgs/Status): System health.

## Usage

This node is typically launched via `heron_base/launch/base.launch` or included in the simulation backend.

```bash
rosrun heron_controller controller
```

## License

BSD License.
