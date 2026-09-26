# PR #21 readiness evidence (2026-09-26)

Reviewed and repaired source: `d1859696b878b65595a2d00bf1168d09c96d5a75`, including target `13b5ed98c57709737990695cd3544801cf8885f2`. This evidence commit changes documentation/artifacts only.

[Report](report.md) records initial CI failures, the five-file repair, exact build environment and limitations. `evidence.tar.xz` contains raw text logs and all eight baseline/current repeated replay outputs. It contains no build products or dependency binaries. `evidence-sha256.json` records every archive member; `SHA256SUMS` records the archive and the two exact time comparison sources.

Verified results:

- Same-toolchain unicycle replay: SHA-256 `9ba2d377d1932100a90538b40b76e53d068c988230a33e5de7b552a9d8827b7f`, 1682 actual acados solves, all successful.
- Same-toolchain mecanum replay: SHA-256 `446c71651bbff780429f1718eed20d356ded7832f4d23df1d783da8537ad3f0a`.
- Baseline/current and both repeats match byte for byte for each controller.
- Each core Time copy: 24,571,432 comparisons against real ROS Time, zero mismatches.
- 334 catkin/rostest tests, 10 Python model/tracker tests, 7 Scout contract tests and 10 Session boundary tests passed.
- Release install, three launch entrypoint parses, package compliance and NMPC result ownership passed.
- Clang 10 formatting plus the full 59-translation-unit clang-tidy gate passed.

Reproduce from a repository clone that has the recorded revisions, with Docker and the real runtime dependency image available locally:

```sh
./test/replay/readiness-20260926/scripts/reproduce.sh /absolute/path/to/new-output
./test/replay/readiness-20260926/scripts/quality.sh /absolute/path/to/new-output
```

The original runtime image is pinned by local image ID `sha256:0dddefada71477f19154c9d2aaf8ec6c6b922ed58ec4bc5ad9b15229ca649d2c`; no immutable registry digest was available for that locally assembled image. Scripts accept a second argument for an equivalent runtime image. The report lists dependency versions. Changing the toolchain may change output hashes, so compare baseline and current within the same image. Quality tooling has an immutable registry digest. `quality.sh` uses the original eight-core CPU selection (`0-7`); adjust it on smaller hosts.

`scripts/build.sh`, `run_replay.sh`, `run_tests.sh` and `install_check.sh` are the exact helpers used for this review. `reproduce.sh` and `quality.sh` package the manual orchestration; they were syntax checked when archived, without repeating the already completed numerical/build checks. The two time tests derive from the existing multirotor time-equivalence harness with only the include and namespace adapted.

The original build regenerated NMPC code with CasADi 3.7.2 in both snapshots, producing identical generated files. This verifies the finite deterministic core replay on that actual build stack; it does not certify every historical commit, arbitrary inputs, asynchronous live ROS operation or physical driving. Final repair-head remote CI had not run at evidence capture; the original head's four deb matrix checks passed, while its source and quality failures were repaired here.
