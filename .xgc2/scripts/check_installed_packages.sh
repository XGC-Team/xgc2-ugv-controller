#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-noetic}"
set +u
# shellcheck source=/dev/null
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u

dpkg -s "ros-${ROS_DISTRO}-xgc2-ugv-controller" >/dev/null
dpkg -s "ros-${ROS_DISTRO}-xgc2-estimator-rigid-state" >/dev/null
dpkg -s "ros-${ROS_DISTRO}-xgc2-ros1-utils" >/dev/null
dpkg -s libxgc2-state-machine-dev >/dev/null
dpkg -s libxgc2-math-dev >/dev/null
dpkg -s xgc2-acados >/dev/null
xgc2_acados_version="$(dpkg-query -W -f='${Version}' xgc2-acados)"
dpkg --compare-versions "${xgc2_acados_version}" ge "0.1.0-3~focal"
test "$(rospack find unicycle_reference_trajectory)" = "/opt/ros/${ROS_DISTRO}/share/unicycle_reference_trajectory"
test "$(rospack find unicycle_ugv_controller)" = "/opt/ros/${ROS_DISTRO}/share/unicycle_ugv_controller"
test "$(rospack find estimator_vrpn_ugv_state)" = "/opt/ros/${ROS_DISTRO}/share/estimator_vrpn_ugv_state"
test -f "/opt/ros/${ROS_DISTRO}/share/unicycle_reference_trajectory/config/unicycle_reference_trajectory.yaml"
test -f "/opt/ros/${ROS_DISTRO}/share/unicycle_reference_trajectory/launch/ugv_unicycle_reference_trajectory.launch"
test -f "/opt/ros/${ROS_DISTRO}/include/unicycle_reference_trajectory/unicycle_reference_trajectory_runtime.h"
test -f "/opt/ros/${ROS_DISTRO}/include/unicycle_reference_trajectory/AnalyticReference.h"
test -f "/opt/ros/${ROS_DISTRO}/include/unicycle_reference_trajectory/PlanarReferencePoint.h"
test -f "/opt/ros/${ROS_DISTRO}/share/unicycle_ugv_controller/config/unicycle_ugv_controller.yaml"
test -f "/opt/ros/${ROS_DISTRO}/share/unicycle_ugv_controller/launch/ugv_unicycle_nmpc_controller.launch"
test -f "/opt/ros/${ROS_DISTRO}/include/unicycle_ugv_controller/unicycle_ugv_controller.h"
test -x "/opt/ros/${ROS_DISTRO}/lib/unicycle_ugv_controller/unicycle_ugv_controller_node"
test -f "/opt/ros/${ROS_DISTRO}/lib/libunicycle_ugv_controller_nmpc_runtime.so"
roslaunch --files unicycle_reference_trajectory ugv_unicycle_reference_trajectory.launch >/tmp/xgc2-unicycle-reference-files.txt
roslaunch --files unicycle_ugv_controller ugv_unicycle_nmpc_controller.launch >/tmp/xgc2-unicycle-controller-files.txt

while IFS= read -r file; do
  if ! file -b "${file}" | grep -q '^ELF'; then
    continue
  fi
  if ! ldd "${file}" | awk '/not found/ {missing=1} END {exit missing ? 1 : 0}'; then
    echo "missing shared library dependency in ${file}" >&2
    ldd "${file}" >&2 || true
    exit 1
  fi
done < <(find "/opt/ros/${ROS_DISTRO}/lib/unicycle_ugv_controller" \
  "/opt/ros/${ROS_DISTRO}/lib/unicycle_reference_trajectory" \
  "/opt/ros/${ROS_DISTRO}/lib/libunicycle_ugv_controller_nmpc_runtime.so" -type f 2>/dev/null | sort -u)

echo "Installed package check passed"
