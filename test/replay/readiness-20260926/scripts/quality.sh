#!/usr/bin/env bash
# Re-run the unchanged full quality gate using real Clang 10 from the CI build image.
# First create OUTPUT_DIRECTORY with reproduce.sh. This script does not install host packages.
set -euo pipefail
if [[ $# -lt 1 || $# -gt 2 ]]; then
  echo "usage: $0 OUTPUT_DIRECTORY [RUNTIME_IMAGE]" >&2
  exit 2
fi
output="$(cd "$1" && pwd)"
runtime_image="${2:-sha256:0dddefada71477f19154c9d2aaf8ec6c6b922ed58ec4bc5ad9b15229ca649d2c}"
quality_image=ghcr.io/xgc-team/xgc2-images/xgc2-build-focal-full-noetic@sha256:2d0ab240a669e59dc6e86e41806041a7f5d46751c787b5cbb225b10e1faed39b
test -d "$output/.review/head/src/ugv"
quality_tools="$output/.review/quality-tools"
if [[ -e "$quality_tools" ]]; then
  echo "quality-tools directory already exists; use a fresh reproduction workspace" >&2
  exit 2
fi
mkdir -p "$quality_tools/bin" "$quality_tools/lib"
container="$(docker create --label com.docker.compose.project=pr-validation --label com.docker.compose.service=validation --entrypoint /bin/bash "$quality_image" -c ':')"
trap 'docker rm "$container" >/dev/null' EXIT
docker cp "$container:/usr/lib/llvm-10" "$quality_tools/llvm-10"
for library in libclang-cpp.so.10 libLLVM-10.so.1 libpopt.so.0; do
  docker cp -L "$container:/lib/x86_64-linux-gnu/$library" "$quality_tools/lib/"
done
docker cp "$container:/usr/bin/rsync" "$quality_tools/bin/rsync"
docker run --rm --label com.docker.compose.project=pr-validation \
  --label com.docker.compose.service=validation --cpuset-cpus=0-7 --entrypoint /bin/bash \
  -v "$output:/review" -w /review/.review/head/src/ugv "$runtime_image" \
  -c 'set -e; export PATH=/review/.review/quality-tools/llvm-10/bin:/review/.review/quality-tools/bin:$PATH; export LD_LIBRARY_PATH=/review/.review/quality-tools/lib:/opt/xgc2/acados/lib; clang-tidy --version; clang-format --version; RUNNER_TEMP=/review/.review/quality XGC2_CLANG_TIDY_SCOPE=full .xgc2/scripts/check_cpp_quality.sh' \
  > "$output/.review/cpp-quality-runtime.log" 2>&1
