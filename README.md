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
- `ugv_reset_safety`: shared swarm Reset coordinator, geometric path planning,
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
roslaunch --files unicycle_ugv_controller ugv_unicycle_nmpc_controller.launch
roslaunch --files mecanum_ugv_controller ugv_mecanum_reset_controller.launch
```

## Shuttle rail (no U-turn)

`unicycle_target_replanner` can hold a fixed `X` and shuttle along `Y`.
Rail heading is the nearer of `+Y` / `-Y` to the current body yaw. Body
speed is `vy * sin(yaw)`, so a nose already on `-Y` goes forward toward
`-Y` instead of spinning 180° onto `+Y`. Off-rail poses within
`shuttle_capture_radius` (default 30 m of the finite rail segment) get a
geometric SE2 plan onto a rail entry pose if MINCO fails. Reverse Y-legs
are published only after the robot is on the rail (X and a rail-axis yaw).
Poses farther than the capture radius are refused.

```bash
roslaunch unicycle_reference_trajectory ugv_unicycle_target_replanner.launch \
  ns:=ugv1 \
  config_file:=$(rospack find unicycle_reference_trajectory)/config/unicycle_shuttle.yaml
```

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

```bash
roslaunch unicycle_ugv_controller ugv_unicycle_nmpc_controller.launch \
  ns:=ugv1 config_file:=/absolute/path/to/controller.yaml
```

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
