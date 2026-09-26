#!/usr/bin/env bash
set -euo pipefail
set +u
source /opt/ros/noetic/setup.bash
set -u
export LD_LIBRARY_PATH=/opt/xgc2/acados/lib:${LD_LIBRARY_PATH:-}
cd /review/.review/"$1"
catkin_make -j4 -l4 -DCMAKE_BUILD_TYPE=RelWithDebInfo
catkin_make -j4 -l4 tests
