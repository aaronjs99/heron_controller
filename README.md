# HERON Controller: Hydrodynamic Command Allocation and Dynamics Control

## Abstract

The **Heron Controller** provides the low-level Real-Time control infrastructure for the Heron USV. The system is responsible for the synthesis of multi-variable wrench-to-thrust mappings, translating high-level force and torque vectors (Force X, Torque Z) into discrete thruster setpoints. This process accounts for the underlying differential drive topology and non-linear hydrodynamic drag manifolds.

## Methodological Features

*   **Dynamic Force Allocation**: Adaptive differential mixing for non-holonomic thrust distribution.
*   **Advanced Helm Interface**: Orchestrates standard `geometry_msgs/Twist` and `heron_msgs/Helm` input streams.
*   **System Diagnostics**: Asynchronous publication of battery metrics, thermal states, and communication link integrity.

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
