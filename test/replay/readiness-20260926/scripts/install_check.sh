#!/usr/bin/env bash
set -euo pipefail
set +u
source /opt/ros/noetic/setup.bash
source /review/.review/head/devel/setup.bash
set -u
export LD_LIBRARY_PATH=/opt/xgc2/acados/lib:${LD_LIBRARY_PATH:-}
cd /review/.review/head
DESTDIR=/review/.review/install-root catkin_make -j4 -l4 install -DCMAKE_INSTALL_PREFIX=/opt/ros/noetic -DCMAKE_BUILD_TYPE=Release
test "$(rospack find unicycle_reference_trajectory)" = "$PWD/src/ugv/unicycle_reference_trajectory"
test "$(rospack find unicycle_ugv_controller)" = "$PWD/src/ugv/unicycle_ugv_controller"
test "$(rospack find mecanum_ugv_controller)" = "$PWD/src/ugv/mecanum_ugv_controller"
roslaunch --files unicycle_reference_trajectory ugv_unicycle_reference_trajectory.launch
roslaunch --files unicycle_ugv_controller ugv_unicycle_nmpc_controller.launch
roslaunch --files mecanum_ugv_controller ugv_mecanum_reset_controller.launch
