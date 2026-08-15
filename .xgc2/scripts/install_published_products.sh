#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

export DEBIAN_FRONTEND=noninteractive
"${SCRIPT_DIR}/setup_xgc2_apt_source.sh"
apt-get install -y --no-install-recommends \
  libxgc2-math-dev \
  libxgc2-state-machine-dev \
  xgc2-acados \
  ros-noetic-xgc2-estimator-rigid-state-msgs \
  ros-noetic-xgc2-unicycle-reference-trajectory-msgs \
  ros-noetic-xgc2-ros1-utils
