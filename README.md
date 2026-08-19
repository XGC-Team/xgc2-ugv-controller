# XGC2 UGV Controller

ROS1 unicycle UGV controller product repository for XGC2 robots.

Packages:

- `unicycle_reference_trajectory`: planar reference trajectory messages,
  generation runtime, and ROS publishers.
- `unicycle_ugv_controller`: unicycle-model UGV NMPC controller publishing
  `geometry_msgs/Twist`.

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
```

## Shuttle rail (no U-turn)

`unicycle_target_replanner` can hold a fixed `X` and shuttle along `Y`.
Yaw stays `+Y`; negative speed covers `-Y`.

```bash
roslaunch unicycle_reference_trajectory ugv_unicycle_target_replanner.launch \
  ns:=ugv1 random_targets:=false shuttle_mode:=true \
  shuttle_x:=0.0 shuttle_y_min:=-2.0 shuttle_y_max:=2.0 shuttle_speed:=0.5
```

## Control-state modes

The same controller executable supports both state providers:

- `state_source:=state_estimator` consumes `RigidStateEstimate` and projects
  to SE2 (`x, y, yaw, speed, yaw_rate`) at the control boundary.
  Healthy means `estimator_state == STATE_RUNNING` (**3**, not the planar 2)
  and no `FLAG_FAULT`.
- `state_source:=vrpn_direct` consumes trusted VRPN pose and twist directly,
  without launching an estimator. The product NMPC remains the sole
  `cmd_vel` publisher for the nonholonomic vehicle.

Example:

```bash
roslaunch unicycle_ugv_controller ugv_unicycle_nmpc_controller.launch \
  ns:=ugv1 state_source:=vrpn_direct
```
