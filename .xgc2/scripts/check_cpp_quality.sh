#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
ROS_DISTRO="${ROS_DISTRO:-noetic}"

require_command() {
  local command_name="$1"
  if ! command -v "${command_name}" >/dev/null 2>&1; then
    echo "missing required command: ${command_name}" >&2
    exit 1
  fi
}

if [[ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  echo "missing ROS setup: /opt/ros/${ROS_DISTRO}/setup.bash" >&2
  exit 1
fi

set +u
# shellcheck source=/dev/null
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u

require_command clang-format
require_command clang-tidy
require_command catkin_make
require_command rsync

mapfile -d '' CXX_FILES < <(
  cd "${REPO_ROOT}"
  find unicycle_ugv_controller/include unicycle_ugv_controller/src unicycle_ugv_controller/test \
       unicycle_reference_trajectory/include unicycle_reference_trajectory/src unicycle_reference_trajectory/test \
    -type f \
    \( -name "*.cpp" -o -name "*.cc" -o -name "*.cxx" -o \
      -name "*.h" -o -name "*.hpp" -o -name "*.hh" -o -name "*.hxx" \) \
    -print0 | sort -z
)

if [[ "${#CXX_FILES[@]}" -eq 0 ]]; then
  echo "no C++ files found" >&2
  exit 1
fi

echo "Running clang-format..."
(
  cd "${REPO_ROOT}"
  clang-format --dry-run --Werror "${CXX_FILES[@]}"
)

WORK_DIR="${RUNNER_TEMP:-${TMPDIR:-/tmp}}/xgc2-ugv-controller-cpp-quality"
rm -rf "${WORK_DIR}"
mkdir -p "${WORK_DIR}/src/xgc2-ugv-controller"
rsync -a --delete --exclude ".git" "${REPO_ROOT}/" "${WORK_DIR}/src/xgc2-ugv-controller/"

echo "Generating compile_commands.json..."
(
  cd "${WORK_DIR}"
  catkin_make \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
    -DCMAKE_BUILD_TYPE=Debug
)

echo "Running clang-tidy..."
mapfile -d '' TIDY_FILES < <(
  find "${WORK_DIR}/src/xgc2-ugv-controller/unicycle_ugv_controller/src" \
       "${WORK_DIR}/src/xgc2-ugv-controller/unicycle_ugv_controller/test" \
       "${WORK_DIR}/src/xgc2-ugv-controller/unicycle_reference_trajectory/src" \
       "${WORK_DIR}/src/xgc2-ugv-controller/unicycle_reference_trajectory/test" \
    -type f \( -name "*.cpp" -o -name "*.cc" -o -name "*.cxx" \) \
    -print0 | sort -z
)
clang-tidy \
  -p "${WORK_DIR}/build" \
  -header-filter="^${WORK_DIR}/src/xgc2-ugv-controller/(unicycle_ugv_controller|unicycle_reference_trajectory)/(src|test)/" \
  -quiet \
  "${TIDY_FILES[@]}"

echo "C++ quality check passed"
