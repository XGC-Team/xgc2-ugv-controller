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
[[ "${distribution}" == focal ]] || { echo "UGV controller release requires Focal" >&2; exit 1; }
install -m 0755 -d /etc/apt/keyrings
curl -fsSL --retry 5 https://xgc2.apt.xiaokang.ink/xgc2-archive-keyring.gpg \
  -o /etc/apt/keyrings/xgc2-archive-keyring.gpg
chmod 0644 /etc/apt/keyrings/xgc2-archive-keyring.gpg
if [[ -n "${overlay_url}" ]]; then source_url="${overlay_url%/}"; fi
rm -f /etc/apt/sources.list.d/00-xgc2-release-train.list
printf 'deb [arch=%s signed-by=/etc/apt/keyrings/xgc2-archive-keyring.gpg] %s %s %s\n' \
  "${arch}" "${source_url}" "${distribution}" "${component}" > "${list_file}"
apt-get update -o Dir::Etc::sourcelist="${list_file}" -o Dir::Etc::sourceparts="-" -o APT::Get::List-Cleanup="0"
