#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

DOCKER_IMAGE="${DOCKER_IMAGE:-ros:noetic-ros-base-focal}"
WORK_DIR="${WORK_DIR:-${REPO_ROOT}/.work/docker}"
OUTPUT_DIR="${OUTPUT_DIR:-${REPO_ROOT}/debs}"
INSTALL_CHECK="${INSTALL_CHECK:-true}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --image)
      DOCKER_IMAGE="$2"
      shift 2
      ;;
    --work-dir)
      WORK_DIR="$2"
      shift 2
      ;;
    --output-dir)
      OUTPUT_DIR="$2"
      shift 2
      ;;
    --skip-install-check)
      INSTALL_CHECK=false
      shift
      ;;
    *)
      echo "unknown argument: $1" >&2
      exit 1
      ;;
  esac
done

mkdir -p "${WORK_DIR}" "${OUTPUT_DIR}"

docker pull "${DOCKER_IMAGE}"
docker run --rm \
  -e DEBIAN_FRONTEND=noninteractive \
  -e INSTALL_CHECK="${INSTALL_CHECK}" \
  -e XGC2_APT_COMPONENT="${XGC2_APT_COMPONENT:-}" \
  -e XGC2_APT_DISTRIBUTION="${XGC2_APT_DISTRIBUTION:-}" \
  -e XGC2_APT_SOURCE_URL="${XGC2_APT_SOURCE_URL:-}" \
  -v "${REPO_ROOT}:/workspace/xgc2-ugv-controller:ro" \
  -v "${WORK_DIR}:/workspace/work" \
  -v "${OUTPUT_DIR}:/workspace/out" \
  "${DOCKER_IMAGE}" \
  bash -lc '
    set -euo pipefail

    export DEBIAN_FRONTEND=noninteractive
    /workspace/xgc2-ugv-controller/.xgc2/scripts/setup_xgc2_apt_source.sh
    apt-get install -y --no-install-recommends \
      build-essential \
      ca-certificates \
      cmake \
      dpkg-dev \
      fakeroot \
      file \
      git \
      libeigen3-dev \
      libxgc2-math-dev \
      libxgc2-state-machine-dev \
      python3-numpy \
      rsync \
      xgc2-acados \
      ros-noetic-geometry-msgs \
      ros-noetic-message-generation \
      ros-noetic-roscpp \
      ros-noetic-roslaunch \
      ros-noetic-rospack \
      ros-noetic-rospy \
      ros-noetic-rostest \
      ros-noetic-rosunit \
      ros-noetic-std-msgs \
      ros-noetic-xgc2-estimator-rigid-state \
      ros-noetic-xgc2-ros1-utils

    rm -rf /workspace/work/src /workspace/work/build /workspace/work/devel /workspace/work/install-root
    mkdir -p /workspace/work/src/xgc2-ugv-controller
    rsync -a --delete /workspace/xgc2-ugv-controller/ /workspace/work/src/xgc2-ugv-controller/

    cd /workspace/work
    set +u
    source /opt/ros/noetic/setup.bash
    set -u
    parallel_jobs="$(nproc)"
    PYTHONPATH=/workspace/work/src/xgc2-ugv-controller/unicycle_ugv_controller/tools \
      python3 -B -m unittest discover \
        -s /workspace/work/src/xgc2-ugv-controller/unicycle_ugv_controller/tools/unicycle_nmpc/tests \
        -v
    catkin_make -j"${parallel_jobs}" -l"${parallel_jobs}" \
      run_tests_unicycle_reference_trajectory \
      run_tests_unicycle_ugv_controller
    catkin_test_results
    DESTDIR=/workspace/work/install-root catkin_make -j"${parallel_jobs}" -l"${parallel_jobs}" install \
      -DCMAKE_INSTALL_PREFIX=/opt/ros/noetic \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_CXX_FLAGS_RELEASE="-O3 -DNDEBUG" \
      -DCMAKE_C_FLAGS_RELEASE="-O3 -DNDEBUG"

    /workspace/xgc2-ugv-controller/.xgc2/scripts/package_debs.sh \
      --install-root /workspace/work/install-root \
      --output-dir /workspace/out

    if [[ "${INSTALL_CHECK}" == "true" ]]; then
      apt-get install -y /workspace/out/*.deb
      /workspace/xgc2-ugv-controller/.xgc2/scripts/check_installed_packages.sh
    fi
  '

echo "Debian package output:"
find "${OUTPUT_DIR}" -maxdepth 1 -type f -name "*.deb" -print | sort
