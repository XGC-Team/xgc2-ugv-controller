#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

export DEBIAN_FRONTEND=noninteractive
ROS_DISTRO="${ROS_DISTRO:-noetic}"
[[ "${ROS_DISTRO}" == noetic ]] || { echo "UGV controller release requires Noetic/Focal" >&2; exit 1; }
if [[ -d "${XGC2_LOCAL_DEB_DIR:-}" ]]; then
  shopt -s nullglob
  local_debs=("${XGC2_LOCAL_DEB_DIR}"/*.deb)
  shopt -u nullglob
  if [[ ${#local_debs[@]} -gt 0 ]]; then
    dpkg -i "${local_debs[@]}" || apt-get -f install -y --no-install-recommends
  fi
fi
"${SCRIPT_DIR}/setup_xgc2_apt_source.sh"
apt-get install -y --no-install-recommends \
  libxgc2-math-dev \
  libxgc2-state-machine-dev \
  xgc2-acados \
  "ros-${ROS_DISTRO}-xgc2-estimator-rigid-state-msgs" \
  "ros-${ROS_DISTRO}-xgc2-unicycle-reference-trajectory-msgs" \
  "ros-${ROS_DISTRO}-xgc2-ros1-utils" \
  "ros-${ROS_DISTRO}-xgc2-geometry-msgs"

require_version() {
  local package="$1" minimum="$2" installed
  installed="$(dpkg-query -W -f='${Version}' "${package}")"
  echo "Build dependency: ${package} ${installed} (minimum ${minimum})"
  dpkg --compare-versions "${installed}" ge "${minimum}" || {
    echo "Build dependency ${package} requires >= ${minimum}; installed ${installed}" >&2
    exit 1
  }
}
require_version libxgc2-math-dev '0.5.12-1~'
require_version "ros-${ROS_DISTRO:-noetic}-xgc2-unicycle-reference-trajectory-msgs" 1.4.0-1

# The generated header fixes the ROS1 wire MD5 in the compiled consumer.
message_header="/opt/ros/${ROS_DISTRO:-noetic}/include/unicycle_reference_trajectory_msgs/WaypointReferenceRequest.h"
dpkg-query -S "${message_header}"
python3 - "${message_header}" '52c351ce747add594dfc7134ad3b9241' <<'PY_MD5'
import pathlib
import re
import sys

header, expected = sys.argv[1:]
text = pathlib.Path(header).read_text()
trait = text.split("struct MD5Sum<", 1)[1].split("struct DataType<", 1)[0]
match = re.search(r'return "([0-9a-f]{32})";', trait)
actual = match.group(1) if match else None
print(f"Build message header: {header} MD5={actual} (required {expected})")
if actual != expected:
    raise SystemExit("Installed message header does not match the required ROS1 contract")
PY_MD5
