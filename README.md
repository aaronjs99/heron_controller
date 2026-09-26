# HERON Controller

`heron_controller` contains low-level control and command-allocation support for
the Heron USV. It converts higher-level motion or wrench requests into the
left/right drive commands understood by the platform.

## Responsibilities

- helm and wrench command handling
- thruster allocation
- platform command publication
- status publication
- force-compensation math used by allocation and recovery logic

## Main Node

```bash
rosrun heron_controller controller
```

The node is normally launched by a larger bringup or simulation profile rather
than by hand.

## Interfaces

Typical inputs:

- `cmd_helm`
- `cmd_wrench`
- `imu/data`

Typical outputs:

- `cmd_drive`
- `status`

## Workspace Role

ORACLE selects missions, MARINER asks for motion, and the controller turns
low-level platform requests into drive commands. In the integrated GRANDE
runtime, MARINER's drive bridge owns the normal `/cmd_vel` to `/cmd_drive`
navigation path.

## Tests

```bash
catkin test heron_controller --no-status --summarize
```

The focused C++ regression test for force compensation is in
`tests/test_force_compensator.cpp`.

## License

BSD.
