# HERON Controller

Low-level control and command allocation for the Heron USV.

This package converts higher-level motion or wrench requests into the concrete
left/right drive commands the platform understands.

## Responsibilities

- wrench or helm command handling
- thruster allocation
- low-level vehicle interface publishing
- platform-status reporting
- static force-compensation math used by allocation and recovery logic

## Main Node

`controller`

### Typical inputs

- `cmd_helm`
- `cmd_wrench`
- `imu/data`

### Typical outputs

- `cmd_drive`
- `status`

## Role In The Workspace

This package sits below navigation and mission logic:

- ORACLE asks for missions
- MARINER asks for motion
- the Heron controller turns that into vehicle drive commands

## Usage

```bash
rosrun heron_controller controller
```

In practice this is usually launched as part of a larger bringup or simulation
launch file rather than by itself.

## Tests

The package includes a focused C++ regression test for the force compensator in
`tests/test_force_compensator.cpp`. Run it with:

```bash
catkin test heron_controller --no-status --summarize
```

## License

BSD.
