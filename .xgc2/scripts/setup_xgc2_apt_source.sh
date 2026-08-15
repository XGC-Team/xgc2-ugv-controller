#!/usr/bin/env bash
set -euo pipefail

source_url="${XGC2_APT_SOURCE_URL:-https://xgc2.apt.xiaokang.ink}"
overlay_url="${XGC2_APT_OVERLAY_URL:-}"
if [[ -z "${XGC2_APT_DISTRIBUTION:-}" && -r /etc/os-release ]]; then
  # shellcheck disable=SC1091
  . /etc/os-release
  XGC2_APT_DISTRIBUTION="${VERSION_CODENAME:-}"
fi
distribution="${XGC2_APT_DISTRIBUTION:-focal}"
component="${XGC2_APT_COMPONENT:-main}"
list_file="${XGC2_APT_LIST_FILE:-/etc/apt/sources.list.d/xgc2.list}"
arch="$(dpkg --print-architecture)"

if [[ -z "${source_url}" || -z "${distribution}" || -z "${component}" ]]; then
  echo "XGC2 APT source is disabled because source, distribution, or component is empty" >&2
  exit 0
fi

if ! dpkg -s ca-certificates >/dev/null 2>&1; then
  echo "image is missing ca-certificates; use xgc2-build-focal-ros-noetic" >&2
  exit 1
fi
apt-get update
echo "deb [trusted=yes arch=${arch}] ${source_url} ${distribution} ${component}" > "${list_file}"
if [[ -n "${overlay_url}" ]]; then
  echo "deb [trusted=yes arch=${arch}] ${overlay_url%/} ${distribution} ${component}" \
    > /etc/apt/sources.list.d/00-xgc2-release-train.list
fi
apt-get update
