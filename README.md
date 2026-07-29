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

## Control-state modes

The same controller executable supports both state providers:

- `state_source:=state_estimator` consumes `PlanarStateEstimate`.
- `state_source:=vrpn_direct` consumes trusted VRPN pose and twist directly,
  without launching an estimator. The product NMPC remains the sole
  `cmd_vel` publisher for the nonholonomic vehicle.

Example:

```bash
roslaunch unicycle_ugv_controller ugv_unicycle_nmpc_controller.launch \
  ns:=ugv1 state_source:=vrpn_direct
```
