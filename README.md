# HERON Controller

`heron_controller` contains low-level control and command-allocation support for
the Heron USV. It converts higher-level motion or wrench requests into the
left/right drive commands understood by the platform.

## Responsibilities

- helm and wrench command handling
- thruster allocation
- platform command publication
- force-compensation math used by allocation and recovery logic

## Main Node

```bash
ros2 launch heron_controller controller.launch.py
```

The node is normally launched by a larger bringup or simulation profile rather
than by hand.

## Interfaces

Typical inputs:

- `cmd_vel`
- `cmd_helm`
- `cmd_wrench`
- `cmd_course`
- `state/odometry` (feedback; relative to the node namespace)

Typical outputs:

- `cmd_drive`
- `eff_wrench`
- `fwd_vel_debug`, `yaw_rate_debug`, `yaw_debug`

## Workspace Role

ORACLE selects missions, MARINER asks for motion, and the controller turns
low-level platform requests into drive commands. In the integrated GRANDE
runtime, MARINER's drive bridge owns the normal `/cmd_vel` to `/cmd_drive`
navigation path.

## License

BSD.

# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| .gitignore | Excludes local environments, generated files, and robot recordings. | Git | Contributors |
| CHANGELOG.rst | Records controller package releases. | None | Package users |
| CMakeLists.txt | Builds and installs the ROS 2 Heron controller executable. | CMake, ament_cmake, ROS 2 | colcon build |
| LICENSE | Defines the inherited Clearpath BSD license terms. | None | Package users |
| package.xml | Declares controller package metadata and ROS dependencies. | ROS 2 Jazzy | colcon, rosdep |
