#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

cd "${REPO_ROOT}"

bash -n .xgc2/scripts/*.sh

nested_git="$(
  find . \
    -path ./.git -prune -o \
    -path './*/build' -prune -o \
    -path './*/devel' -prune -o \
    -path './*/install' -prune -o \
    -path ./.work -prune -o \
    -path ./debs -prune -o \
    -name .git -print
)"
if [[ -n "${nested_git}" ]]; then
  echo "Nested .git directory found." >&2
  echo "${nested_git}" >&2
  exit 1
fi

if git ls-files | grep -E '(^|/)(build|devel|install|\.catkin_tools|\.work|debs)(/|$)' >/dev/null; then
  echo "Generated build artifacts are tracked." >&2
  git ls-files | grep -E '(^|/)(build|devel|install|\.catkin_tools|\.work|debs)(/|$)' >&2
  exit 1
fi

required_files=(
  .clang-format
  .clang-tidy
  .github/workflows/build-debs.yml
  .xgc2/product.yml
  .xgc2/scripts/build_debs_in_docker.sh
  .xgc2/scripts/check_core_libraries.sh
  .xgc2/scripts/check_cpp_quality.sh
  .xgc2/scripts/check_installed_packages.sh
  .xgc2/scripts/check_package_compliance.sh
  .xgc2/scripts/check_version_bump.sh
  .xgc2/scripts/package_debs.sh
  .xgc2/scripts/publish_apt_repo.sh
  .xgc2/scripts/setup_xgc2_apt_source.sh
  unicycle_ugv_controller/CMakeLists.txt
  unicycle_ugv_controller/package.xml
  unicycle_ugv_controller/launch/ugv_unicycle_nmpc_controller.launch
  unicycle_ugv_controller/config/unicycle_ugv_controller.yaml
  unicycle_reference_trajectory/CMakeLists.txt
  unicycle_reference_trajectory/package.xml
  unicycle_reference_trajectory/msg/AnalyticReference.msg
  unicycle_reference_trajectory/msg/WaypointReferenceRequest.msg
  unicycle_reference_trajectory/msg/SampledReference.msg
  unicycle_reference_trajectory/msg/ActivePolynomialReference.msg
  unicycle_reference_trajectory/msg/PlanarReferencePoint.msg
  unicycle_reference_trajectory/msg/ReferenceStatus.msg
  unicycle_reference_trajectory/include/unicycle_reference_trajectory/unicycle_reference_trajectory_runtime.h
  unicycle_reference_trajectory/launch/ugv_unicycle_reference_trajectory.launch
)

for file in "${required_files[@]}"; do
  if [[ ! -f "${file}" ]]; then
    echo "Missing required file: ${file}" >&2
    exit 1
  fi
done

grep -q "id: xgc2-ugv-controller" .xgc2/product.yml
grep -Eq '^version: [0-9]+\.[0-9]+\.[0-9]+-[0-9]+$' .xgc2/product.yml
grep -q "<name>unicycle_ugv_controller</name>" unicycle_ugv_controller/package.xml
grep -q "<name>unicycle_reference_trajectory</name>" unicycle_reference_trajectory/package.xml
grep -q "run_tests_unicycle_reference_trajectory" .github/workflows/build-debs.yml
grep -q "run_tests_unicycle_ugv_controller" .github/workflows/build-debs.yml
grep -q "check_version_bump.sh --ci" .github/workflows/build-debs.yml
grep -q "PACKAGE=\"ros-\${ROS_DISTRO}-xgc2-ugv-controller\"" .xgc2/scripts/package_debs.sh
grep -q "unicycle_ugv_controller" .xgc2/scripts/package_debs.sh
grep -q "unicycle_reference_trajectory" .xgc2/scripts/package_debs.sh
grep -q "xgc2-acados (>= 0.1.0-3~focal)" .xgc2/product.yml
grep -q "xgc2-acados (>= 0.1.0-3~focal)" .xgc2/scripts/package_debs.sh

if grep -R --exclude='check_package_compliance.sh' "ros-noetic-xgc2-reference" \
  .github .xgc2 README.md unicycle_ugv_controller unicycle_reference_trajectory >/dev/null; then
  echo "Deprecated ros-noetic-xgc2-reference dependency found." >&2
  exit 1
fi

if grep -R --exclude='check_package_compliance.sh' \
  -E "UavFlatTrajectory|UavBsplineTrajectory|nmpc_reference_trajectory|uav_reference_circle_entry|publish_uav_reference_trajectory|alg/reference_trajectory/(flat|bspline|activate)|core/trajectory_core|core/nmpc_reference" \
  .github .xgc2 README.md unicycle_ugv_controller unicycle_reference_trajectory >/dev/null; then
  echo "Deprecated reference trajectory interface found." >&2
  exit 1
fi

echo "Package compliance checks passed."
