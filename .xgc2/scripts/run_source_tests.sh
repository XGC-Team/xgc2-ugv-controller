#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
work_dir="${RUNNER_TEMP:-/tmp}/xgc2-ugv-controller-compliance"
install_root="${RUNNER_TEMP:-/tmp}/xgc2-ugv-controller-install-root"

rm -rf "$work_dir" "$install_root"
mkdir -p "$work_dir/src/xgc2-ugv-controller"
rsync -a --delete "$REPO_ROOT/" "$work_dir/src/xgc2-ugv-controller/"

cd "$work_dir"
set +u
source /opt/ros/noetic/setup.bash
set -u
parallel_jobs="$(nproc)"
PYTHONPATH="$work_dir/src/xgc2-ugv-controller/unicycle_ugv_controller/tools" \
  python3 -B -m unittest discover \
    -s "$work_dir/src/xgc2-ugv-controller/unicycle_ugv_controller/tools/unicycle_nmpc/tests" \
    -v
catkin_make -j"${parallel_jobs}" -l"${parallel_jobs}" \
  run_tests_unicycle_reference_trajectory \
  run_tests_unicycle_ugv_controller
catkin_test_results
DESTDIR="$install_root" catkin_make -j"${parallel_jobs}" -l"${parallel_jobs}" install \
  -DCMAKE_INSTALL_PREFIX=/opt/ros/noetic \
  -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
test "$(rospack find unicycle_reference_trajectory)" = "$work_dir/src/xgc2-ugv-controller/unicycle_reference_trajectory"
test "$(rospack find unicycle_ugv_controller)" = "$work_dir/src/xgc2-ugv-controller/unicycle_ugv_controller"
roslaunch --files unicycle_reference_trajectory ugv_unicycle_reference_trajectory.launch >/tmp/xgc2-unicycle-reference-files.txt
roslaunch --files unicycle_ugv_controller ugv_unicycle_nmpc_controller.launch >/tmp/xgc2-unicycle-controller-files.txt
