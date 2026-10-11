#!/usr/bin/env bash
# Compile-only check of the ROS-free sources of the Scout chain with the Ubuntu 18.04 g++ 7.5
# (glibc 2.27 sysroot), the oldest compiler the modules have to build with.
#
# What it covers: the cores (reference trajectory, unicycle and Mecanum controller, the Reset path and
# geometry mathematics), the two ROS-free modules, the replays and the module tests. What it cannot
# cover: the sources that include ROS (the nodes, the input producers and output consumers, the
# conversions, the ROS client of the Reset lease, the ROS edge) because Melodic headers are not in the
# sysroot, and linking and running, because the sysroot has none of the libraries (acados, the XGC2
# math and state machine libraries). The headers of Eigen, jsoncpp and googletest are the host's own;
# they are text and compile with g++ 7.5.
#
# The unicycle controller includes the NMPC solver that the build of unicycle_ugv_controller generates
# (unicycle_ugv_controller/generated/nmpc/unicycle_nmpc): configure that package once before the check.
#
#   XGC2_BIONIC_TOOLCHAINS  directory with bin/bionic-g++ and bionic-sysroot
#                           (default /home/node/workspace/toolchains, as in xgc2-module's scripts)
#   XGC2_UNDERLAY           prefix with include/xgc2_math, include/state_machine and the acados headers
#   XGC2_MODULE_INCLUDE     directory with xgc2/module.h (the SDK of xgc2-module)
set -uo pipefail

repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
toolchains="${XGC2_BIONIC_TOOLCHAINS:-/home/node/workspace/toolchains}"
underlay="${XGC2_UNDERLAY:-/home/node/workspace/underlay/install}"
module_include="${XGC2_MODULE_INCLUDE:-/usr/include}"
gxx="${toolchains}/bin/bionic-g++"
work="$(mktemp -d)"
trap 'rm -rf -- "${work}"' EXIT
[[ -x "${gxx}" ]] || { echo "no Bionic g++ at ${gxx} (set XGC2_BIONIC_TOOLCHAINS)" >&2; exit 2; }

# The host's text headers, behind their own directory so that its libc headers do not shadow the sysroot's.
mkdir -p "${work}/headers/jsoncpp_root"
ln -s /usr/include/eigen3 "${work}/headers/eigen3"
ln -s /usr/include/jsoncpp "${work}/headers/jsoncpp_root/jsoncpp"
ln -s /usr/include/gtest "${work}/headers/jsoncpp_root/gtest"

includes=(
  -I"${repo}/unicycle_reference_trajectory/include"
  -I"${repo}/unicycle_ugv_controller/include"
  -I"${repo}/unicycle_ugv_controller/generated/nmpc/unicycle_nmpc"
  -I"${repo}/mecanum_ugv_controller/include"
  -I"${repo}/ugv_reset_safety/include"
  -I"${repo}/ugv_reset_client/include"
  -I"${repo}/ugv_modules/include" -I"${repo}/ugv_modules/src" -I"${repo}/ugv_modules/test"
  -I"${repo}/unicycle_reference_trajectory/test/replay"
  -isystem "${module_include}"
  -isystem "${underlay}/include" -isystem "${underlay}"
  -isystem "${underlay}/include/blasfeo/include" -isystem "${underlay}/include/hpipm/include"
  -isystem "${underlay}/interfaces" -isystem "${underlay}/interfaces/acados_template"
  -isystem "${work}/headers/eigen3" -isystem "${work}/headers/jsoncpp_root"
)
flags=(-std=c++17 -O1 -Wall -Wextra -Werror -fPIC -DACADOS_WITH_OSQP -DACADOS_WITH_QPOASES
       -DUGV_MODULES_VERSION=\"0\" -DREFERENCE_MODULE_PATH=\"x\" -DCONTROLLER_MODULE_PATH=\"x\")

sources=(
  unicycle_reference_trajectory/src/unicycle_reference_trajectory_runtime.cpp
  unicycle_reference_trajectory/src/reference_driver.cpp
  unicycle_reference_trajectory/src/default_analytic.cpp
  unicycle_reference_trajectory/src/state_machine/active_state.cpp
  unicycle_reference_trajectory/src/state_machine/planning_state.cpp
  unicycle_reference_trajectory/src/state_machine/ready_state.cpp
  unicycle_reference_trajectory/src/state_machine/self_check_state.cpp
  unicycle_ugv_controller/src/common_types.cpp
  unicycle_ugv_controller/src/core_log.cpp
  unicycle_ugv_controller/src/reference_cache.cpp
  unicycle_ugv_controller/src/unicycle_ugv_controller.cpp
  unicycle_ugv_controller/src/nmpc/nmpc_execution.cpp
  unicycle_ugv_controller/src/nmpc/nmpc_tracking_backend.cpp
  unicycle_ugv_controller/src/nmpc/unicycle_nmpc_solver.cpp
  unicycle_ugv_controller/src/state_machine/custom1_state.cpp
  unicycle_ugv_controller/src/state_machine/health_monitor_state.cpp
  unicycle_ugv_controller/src/state_machine/ready_state.cpp
  unicycle_ugv_controller/src/state_machine/reset_state.cpp
  unicycle_ugv_controller/src/state_machine/self_check_state.cpp
  mecanum_ugv_controller/src/common_types.cpp
  mecanum_ugv_controller/src/core_log.cpp
  mecanum_ugv_controller/src/mecanum_ugv_controller.cpp
  mecanum_ugv_controller/src/state_machine/states.cpp
  ugv_reset_safety/src/reset_geometry.cpp
  ugv_reset_safety/src/reset_path.cpp
  ugv_modules/src/ugv_unicycle_reference.cpp
  ugv_modules/src/ugv_unicycle_controller.cpp
  unicycle_reference_trajectory/test/replay/reference_replay.cpp
  unicycle_ugv_controller/test/replay/unicycle_replay.cpp
  ugv_reset_client/test/reset_lease_test.cpp
  ugv_modules/test/reference_module_test.cpp
  ugv_modules/test/controller_module_test.cpp
  ugv_modules/test/payload_conversion_test.cpp
  ugv_modules/test/chain_parity_test.cpp
)

status=0
for source in "${sources[@]}"; do
  if "${gxx}" "${flags[@]}" "${includes[@]}" -c "${repo}/${source}" -o "${work}/object.o" 2>"${work}/log"; then
    echo "ok   ${source}"
  else
    echo "FAIL ${source}"
    head -20 "${work}/log"
    status=1
  fi
done
"${gxx}" --version | head -1
exit "${status}"
