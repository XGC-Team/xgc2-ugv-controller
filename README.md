# XGC2 UGV Controller

ROS1 unicycle UGV controller product repository for XGC2 robots.

Packages:

- `unicycle_reference_trajectory`: planar reference trajectory messages,
  generation runtime, and ROS publishers.
- `unicycle_ugv_controller`: unicycle chassis controller, unique `cmd_vel`
  publisher. CONTROL states: SelfCheck / Ready / Reset / Custom1. Reset executes leased commands from `ugv_reset_safety`. Custom1 selects
  `nmpc` or `flatness` by rosparam. Remote I/O is canonical `{ns}/pose` only.
- `mecanum_ugv_controller`: reusable holonomic chassis modules. Health /
  SelfCheck, `Reset` to an Experiment `initialPose`, and first-order Custom1
  (world ENU velocity to body FLU, heading P to east). Algorithms publish
  `{ns}/alg/reference/twist` only; they do not publish `cmd_vel`.
- `ugv_fleet_host`: single native fleet owner, bounded XRPC start/cancel/held observe,
  embedded mixed-chassis Coordinator and fixed shared NMPC workers.
- `ugv_reset_safety`: ROS-free swarm Reset coordinator, geometric path planning,
  DWA with obstacle/inter-vehicle footprint checks, command limits and slew limits.

Reset requires an explicit target, a complete UGV roster, fresh canonical poses
and controller states, and an authoritative static obstacle snapshot. Missing
inputs refuse motion. See [Reset design and limits](ugv_reset_safety/README.md).

## Install

```bash
sudo apt update
sudo apt install ros-noetic-xgc2-ugv-controller
```

## Smoke Test

```bash
source /opt/ros/noetic/setup.bash
roslaunch --files unicycle_reference_trajectory ugv_unicycle_reference_trajectory.launch
# UGV launch requires explicit XRPC bootstrap and scene references.
# See ugv_fleet_host/README.md.
```

## NMPC tracking

The runtime model keeps yaw rate as a state and commands angular acceleration:

```text
x = [x, y, yaw, speed, omega]
u = [linear_accel, angular_accel]
```

This makes `omega` continuous between shooting stages. Both its magnitude and
its rate of change are part of the optimization; `max_angular_acceleration`
also prevents a one-stage full-lock sign flip.

NMPC stage cost is nonlinear LS. Weights and limits are defined in the selected
controller YAML and read once when the node starts. Edit that file, then restart
the node. The launch file accepts the namespace and configuration file.

The fleet launch loads each controller YAML into its original private namespace;
start/cancel/observe use the native XRPC contract in [ugv_fleet_host](ugv_fleet_host/README.md).

## Control-state modes

The controller supports two configured state providers:

- `state_source: state_estimator` consumes `RigidStateEstimate` and projects it
  to SE2 (`x, y, yaw, speed, yaw_rate`) at the control boundary. It requires
  `STATE_RUNNING` and no `FLAG_FAULT`.
- `state_source: platform_pose` consumes canonical `{ns}/pose`. The supplied
  `scout_flatness.yaml` selects this provider with `tracking_strategy: flatness`.

`unicycle_ugv_controller.yaml` selects estimator-backed NMPC.
`unicycle_nmpc_active.yaml` additionally enables automatic tracking for the
corresponding process definition. Each chassis controller owns its `cmd_vel`.

The old target/shuttle replanner is retired. Analytic and sampled references,
fixed-time seventh-order waypoint interpolation, UGV polynomial and PVA
inputs, NMPC/flatness tracking, Reset and DWA remain available. The waypoint
producer and consumer require the 1.4 message contract and a coordinated rebuild.
