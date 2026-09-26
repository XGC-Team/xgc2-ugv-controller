#!/usr/bin/env bash
# Rebuild the reviewed revisions in a new isolated workspace; no host packages installed.
set -euo pipefail
if [[ $# -lt 1 || $# -gt 2 ]]; then
  echo "usage: $0 NEW_OUTPUT_DIRECTORY [RUNTIME_IMAGE]" >&2
  exit 2
fi
scripts="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo="$(git -C "$scripts" rev-parse --show-toplevel)"
output="$1"
runtime_image="${2:-sha256:0dddefada71477f19154c9d2aaf8ec6c6b922ed58ec4bc5ad9b15229ca649d2c}"
if [[ -e "$output" ]]; then
  echo "output directory must not already exist: $output" >&2
  exit 2
fi
docker image inspect "$runtime_image" >/dev/null
mkdir -p "$output/.review/base/src/ugv" "$output/.review/head/src/ugv"
output="$(cd "$output" && pwd)"
base=13b5ed98c57709737990695cd3544801cf8885f2
head=d1859696b878b65595a2d00bf1168d09c96d5a75
git -C "$repo" archive "$base" | tar -x -C "$output/.review/base/src/ugv"
git -C "$repo" archive "$head" | tar -x -C "$output/.review/head/src/ugv"
for spec in unicycle:cd500c9386d24c6b5633e60da21bd49ab96dd82d mecanum:c8aa9fd463a8e4070b676680bfa2fd161e9892ad; do
  name="${spec%%:*}"
  revision="${spec#*:}"
  package="${name}_ugv_controller"
  mkdir -p "$output/.review/base/src/ugv/$package/test/replay"
  for file in CMakeLists.txt "test/replay/${name}_replay.cpp"; do
    git -C "$repo" show "$revision:$package/$file" > "$output/.review/base/src/ugv/$package/$file"
  done
done
cp "$scripts/build.sh" "$scripts/run_replay.sh" "$scripts/run_tests.sh" "$scripts/install_check.sh" "$output/.review/"
cp "$scripts/../time/"*.cpp "$output/.review/"
run() {
  docker run --rm --label com.docker.compose.project=pr-validation \
    --label com.docker.compose.service=validation --entrypoint /bin/bash \
    -v "$output:/review" -w /review "$runtime_image" "$@"
}
for profile in base head; do
  run /review/.review/build.sh "$profile" > "$output/.review/$profile-build.log" 2>&1
  run /review/.review/run_replay.sh "$profile" > "$output/.review/$profile-replay.log" 2>&1
done
for name in unicycle mecanum; do
  cmp "$output/.review/base-$name-1.txt" "$output/.review/head-$name-1.txt"
  sha256sum "$output/.review/"*"-$name-"*.txt
  run -c 'set -e; name="$1"; cd /review/.review/head/src/ugv; g++ -std=c++17 -O2 -I"${name}_ugv_controller/include" -I/opt/ros/noetic/include "/review/.review/${name}_time_equivalence.cpp" -L/opt/ros/noetic/lib -Wl,-rpath,/opt/ros/noetic/lib -lrostime -lcpp_common -o "/review/.review/${name}_time_equivalence"; "/review/.review/${name}_time_equivalence" 2000000' bash "$name" > "$output/.review/${name}_time_equivalence.log" 2>&1
done
run /review/.review/run_tests.sh > "$output/.review/head-tests.log" 2>&1
run -c 'set -e; cd /review/.review/head/src/ugv; python3 unicycle_ugv_controller/test/test_scout_fence_contract.py; python3 ugv_reset_safety/test/world_boundary_launch_test.py; python3 unicycle_ugv_controller/test/nmpc_result_ownership_test.py; .xgc2/scripts/check_package_compliance.sh' > "$output/.review/extra-tests.log" 2>&1
run /review/.review/install_check.sh > "$output/.review/install-check.log" 2>&1
printf 'Results saved in %s/.review\n' "$output"
