#!/usr/bin/env bash
set -euo pipefail

source_url="${XGC2_APT_SOURCE_URL:-https://xgc2.apt.xiaokang.ink}"
distribution="${XGC2_APT_DISTRIBUTION:-bionic}"
component="${XGC2_APT_COMPONENT:-main}"
list_file="${XGC2_APT_LIST_FILE:-/etc/apt/sources.list.d/xgc2.list}"
arch="$(dpkg --print-architecture)"

if [[ -z "${source_url}" || -z "${distribution}" || -z "${component}" ]]; then
  echo "XGC2 APT source is disabled because source, distribution, or component is empty" >&2
  exit 0
fi

apt-get update
apt-get install -y --no-install-recommends ca-certificates
echo "deb [trusted=yes arch=${arch}] ${source_url} ${distribution} ${component}" > "${list_file}"
apt-get update
