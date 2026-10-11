#!/usr/bin/env bash
# A module exports its entry point and nothing else; the ones that are not the ROS edge link no ROS.
#   check_module_linkage.sh --ros-free LIB... --ros LIB...
set -euo pipefail
mode=""
status=0
for argument in "$@"; do
  case "$argument" in
    --ros-free) mode=free; continue ;;
    --ros) mode=ros; continue ;;
  esac
  library="$argument"
  failed=0
  exported="$(nm -D --defined-only "$library" | awk '{print $3}')"
  if [[ "$exported" != "xgc2_module_entry" ]]; then
    echo "FAIL: $library exports: $exported" >&2
    failed=1
  fi
  needed="$(readelf -d "$library" | awk '/NEEDED/ {gsub(/[\[\]]/, "", $5); print $5}')"
  if grep -Eq '^lib(ros|roscpp|rosconsole|rostime|cpp_common|xmlrpcpp|tf2?|boost_)' <<<"$needed"; then
    if [[ "$mode" == free ]]; then
      echo "FAIL: $library links ROS or Boost: $(grep -E '^lib(ros|roscpp|rosconsole|rostime|cpp_common|xmlrpcpp|tf2?|boost_)' <<<"$needed" | tr '\n' ' ')" >&2
      failed=1
    fi
  elif [[ "$mode" == ros ]]; then
    echo "FAIL: $library does not link roscpp" >&2
    failed=1
  fi
  if [[ "$mode" == ros ]]; then
    # roscpp cannot be restarted: the edge is never unmapped
    if ! readelf -d "$library" | grep -q 'NODELETE\|FLAGS_1.*NODELETE'; then
      echo "FAIL: $library is not linked -z nodelete" >&2
      failed=1
    fi
  fi
  if [[ "$failed" == 0 ]]; then
    echo "ok: $library ($mode)"
  else
    status=1
  fi
done
exit $status
