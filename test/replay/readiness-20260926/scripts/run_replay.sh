#!/usr/bin/env bash
set -euo pipefail
set +u
source /opt/ros/noetic/setup.bash
source /review/.review/"$1"/devel/setup.bash
set -u
export LD_LIBRARY_PATH=/opt/xgc2/acados/lib:${LD_LIBRARY_PATH:-}
for name in unicycle mecanum; do
  exe="/review/.review/$1/devel/lib/${name}_ugv_controller/${name}_ugv_controller_replay"
  for n in 1 2; do
    "$exe" "/review/.review/$1-${name}-$n.txt" > "/review/.review/$1-${name}-$n.stdout" 2> "/review/.review/$1-${name}-$n.stderr"
  done
  cmp "/review/.review/$1-${name}-1.txt" "/review/.review/$1-${name}-2.txt"
done
