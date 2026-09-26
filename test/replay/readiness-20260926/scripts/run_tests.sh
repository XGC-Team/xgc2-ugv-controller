#!/usr/bin/env bash
set -euo pipefail
set +u
source /opt/ros/noetic/setup.bash
source /review/.review/head/devel/setup.bash
set -u
export LD_LIBRARY_PATH=/opt/xgc2/acados/lib:${LD_LIBRARY_PATH:-}
export ROS_HOME=/review/.review/ros-home
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=127.0.0.1
cd /review/.review/head
PYTHONPATH="$PWD/src/ugv/unicycle_ugv_controller/tools" python3 -B -m unittest discover -s src/ugv/unicycle_ugv_controller/tools/unicycle_nmpc/tests -v
PYTHONPATH="$PWD/src/ugv/mecanum_ugv_controller/tools" python3 -B -m unittest discover -s src/ugv/mecanum_ugv_controller/tools/holonomic_tracker/tests -v
catkin_make -j4 -l4 run_tests_unicycle_reference_trajectory run_tests_unicycle_ugv_controller run_tests_mecanum_ugv_controller run_tests_ugv_reset_safety
catkin_test_results
