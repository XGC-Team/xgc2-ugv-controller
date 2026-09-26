# UGV controller PR #21 independent computational review

Review date: 2026-09-26. Isolated shared clone: `ugv21-review`; no canonical tree or remote state was modified.

## Exact revisions and repairs

- Inspected PR head: `0cb7c891153cb2c5b0002c931343f29234e8c6f8`, branch `feat/ros-free-core`.
- Current target: `13b5ed98c57709737990695cd3544801cf8885f2` (`origin/noetic`). Actual common ancestor was `328a0e3f33516c101a169889bb49a91defa821f1`.
- Local merge of target: `007a966c2f6535cde25ec1879fd1b2dbcd9fe2d0`. This preserved the three newer Scout candidate/contract files without conflict; the candidate is optional and Session bounds remain authoritative.
- Repair commit / candidate head: `d1859696b878b65595a2d00bf1168d09c96d5a75`.
- Changed only five files in the repair: removed 28 obsolete `ros::Time::init()` calls from the mecanum core runtime tests, changed two test timestamps to core `Time`, added required braces in both core log sinks and replays, and made replay exceptions print an error and return nonzero. Normal control calculations and replay output are unchanged.

The original head was not ready: GitHub source build failed on the remaining ROS references in the mecanum runtime test; C++ quality failed on unbraced statements and replay exceptions escaping `main`. Logs are saved as `ci-source-build-failure.log` and `ci-cpp-quality-failure.log`. The PR's blanket historical claim that every commit passed all non-rostest binaries is therefore not certified by this review. Only the fresh checks below are certified.

## Same-toolchain baseline/current replay

Build/runtime image pinned by immutable local image ID:
`sha256:0dddefada71477f19154c9d2aaf8ec6c6b922ed58ec4bc5ad9b15229ca649d2c`.
It contains actual ROS Noetic and acados, not stubs. Relevant packages: state-machine `0.1.3-11~focal`, xgc2-math `0.5.8-1~focal`, acados `0.1.0-18~focal`, reference-trajectory messages `1.3.0-13`. Compiler is GCC 9.4.0 on Ubuntu 20.04 amd64.

The baseline source snapshot was `git archive 13b5ed98...`; only the two replay sources and their CMake wiring were added from their first pre-refactor commits (`cd500c9386d24c6b5633e60da21bd49ab96dd82d` for unicycle, `c8aa9fd463a8e4070b676680bfa2fd161e9892ad` for mecanum). Production sources were unchanged. Both normal builds regenerated the NMPC generated sources with the installed CasADi 3.7.2/acados template stack. All generated baseline and current files were byte-identical to each other. These differ from some checked-in generator outputs (CasADi 3.6.7+), so this is explicitly a fresh normal-build gate, not a reproduction of the old RoboStack binary/hash.

Both snapshots used `catkin_make -j4 -l4 -DCMAKE_BUILD_TYPE=RelWithDebInfo` followed by `catkin_make -j4 -l4 tests`. Each replay ran twice, all four per-controller outputs were compared with `cmp`, including the final repaired source.

| Replay | Lines | SHA-256, equal baseline/current/repeats |
| --- | ---: | --- |
| unicycle | 13478 | `9ba2d377d1932100a90538b40b76e53d068c988230a33e5de7b552a9d8827b7f` |
| mecanum | 5532 | `446c71651bbff780429f1718eed20d356ded7832f4d23df1d783da8537ad3f0a` |

The unicycle replay contains exactly 1682 actual acados solve records; every record has `ok 1 status 0 cmd 1`. Both replays reach control states 1, 2, 3, 5 with health state 100. Scenarios include tracking, stop/restart, missing/stale pose, fence exit, and reset timeout. Output includes raw double bits, control states/events, commands, plant state and NMPC predicted states. Files: `base-{unicycle,mecanum}-{1,2}.txt`, `head-{unicycle,mecanum}-{1,2}.txt`; scripts `build.sh`, `run_replay.sh`.

Additional time differential check reused the existing multirotor `time_equivalence.cpp` test, changing only the header/namespace for each UGV copy. It compiled against real `librostime` and `libcpp_common`. Each copy passed 24,571,432 comparisons with 0 mismatches (`unicycle_time_equivalence.log`, `mecanum_time_equivalence.log`), using 2,000,000 randomized trials plus edge cases. This is finite differential evidence, not a universal mathematical proof.

## Builds, tests, isolation and packaging

- Baseline and repaired current production builds and test targets completed successfully.
- Current actual catkin test execution: `run_tests_unicycle_reference_trajectory run_tests_unicycle_ugv_controller run_tests_mecanum_ugv_controller run_tests_ugv_reset_safety`; `catkin_test_results`: **334 tests, 0 errors, 0 failures, 0 skipped**. Includes gtest and isolated ROS integration/rostest. Log: `head-final-tests.log`.
- Python tools: 2 NMPC config/model tests and 8 holonomic tracker tests passed.
- Preserved merged Scout contract: 7 tests passed (`scout-fence-contract.log`). Session/world-boundary launch tests: 10 passed (`world-boundary.log`).
- `nmpc_result_ownership_test.py` passed with actual production worker behavior; stale/invalid/duplicate/mismatched results are rejected, and a late worker cannot overwrite Reset.
- `check_package_compliance.sh` passed; `git diff --check` passed.
- Release build/install to isolated `DESTDIR=.review/install-root` passed. The three source `rospack find` checks and `roslaunch --files` for unicycle reference, unicycle NMPC, and mecanum Reset passed. Log: `install-check.log`.
- Core compile flags have no ROS include directories for unicycle core, mecanum core or reset math. `readelf -d` shows no ROS `NEEDED` entries and `nm -D -C` finds no `ros::` symbols. Unicycle links state-machine and acados components plus language/system runtime; mecanum links state-machine plus language/system runtime; reset math links C/C++ runtime only. Their links use `--no-undefined`.
- Original exact-head GitHub four deb matrices (amd64/arm64, Focal/Noetic and Bionic/Melodic) passed. They are historical checks of the original head, not new remote checks of the repair. New remote checks of the repaired head were not available when this archive was captured; only the listed local results and original-head remote checks are asserted here.
- Full local C++ quality result: **PASS** (all-file clang-format 10 check, Debug build, and all 59 clang-tidy translation units; unchanged script exit 0 and `C++ quality check passed`). The initial full-build-image dependency download was too slow and was canceled before the gate ran. We extracted actual Clang 10 tools and their dependencies from `ghcr.io/xgc-team/xgc2-images/xgc2-build-focal-full-noetic@sha256:2d0ab240a669e59dc6e86e41806041a7f5d46751c787b5cbb225b10e1faed39b`, used them in the same pinned runtime image with real installed products, and ran the unchanged script with `XGC2_CLANG_TIDY_SCOPE=full`. Log: `cpp-quality-runtime.log`.

## Reproduction

The original `.review` build workspace is local and ignored. Its raw replay and text logs are preserved in tracked `evidence.tar.xz`, with `evidence-sha256.json` for every member; build products and dependency binaries are excluded. Exact helper scripts and time test sources are included alongside this report. `scripts/reproduce.sh` reconstructs a fresh workspace at the source revisions below. From the clone, mount it at `/review` in the pinned runtime image, override the image's entrypoint to `/bin/bash`, and set Docker labels `com.docker.compose.project=pr-validation`, `com.docker.compose.service=validation` so validation is not mistaken for the active station:

```
/review/.review/build.sh base
/review/.review/build.sh head
/review/.review/run_replay.sh base
/review/.review/run_replay.sh head
cmp /review/.review/base-unicycle-1.txt /review/.review/head-unicycle-1.txt
cmp /review/.review/base-mecanum-1.txt /review/.review/head-mecanum-1.txt
/review/.review/run_tests.sh
/review/.review/install_check.sh
```

`install_check.sh` switches the current build profile to Release, as CI does after tests. Re-run `build.sh head` before reproducing the reported RelWithDebInfo replay hashes. Source setup scripts require temporarily disabling `set -u`; `LD_LIBRARY_PATH` includes `/opt/xgc2/acados/lib`.

## Scope and readiness

**READY for parent review, push, and exact-head GitHub checks.** All local required checks passed on the repaired source; full clang-format/clang-tidy gate completed with exit 0. No remote push or merge was performed here.

The replay proves equality only for the finite deterministic synchronous-core scenarios and this build stack. It does not validate asynchronous live ROS timing, transport faults beyond the integration tests, hardware driving, arbitrary reference inputs or floating-point behavior on every architecture. It does not certify the PR author's per-commit historical hash claims. The new plain structs/time conversion are compiled through the real ROS edge and covered by the passing integration tests; no control gains, default fence, MPC model or controller algorithm was changed by the repair. The replay is an explicitly rerun review gate; the existing CI does not automatically compare two revisions' replay output.
