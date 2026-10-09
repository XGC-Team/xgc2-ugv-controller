# Native UGV fleet owner

One native process owns the existing Unicycle and Mecanum controller cores,
user ROS inputs, and sole `cmd_vel` publishers. It preserves each controller's
configured update rate, the 50 Hz fleet Coordinator, exact generation/stamp
matching, 150 ms world/monotonic leases, 90 s reset deadline, 5 cm / 5 degree
arrival tolerances, measured stop criteria, and the previous actually published
command. Fleet path, DWA, collision, NMPC and user tracking laws are unchanged.

`ResetClient`, reset request/response ROS messages, the external ROS Coordinator
and per-robot controller executables are retired. Reference trajectory publishers
and user `command`, `pose`, `reset_pose`, estimator and reference ROS data inputs
retain their existing interfaces. Controller private YAML parameters retain
`/<robot>/<unicycle|mecanum>_ugv_controller` namespaces.

The host accepts only `--bootstrap-input /absolute/owner/input.json` plus standard
ROS remappings. The common C++ SDK validates the private BootstrapInput and
retains the resolved runtime directory through shutdown. Its binding declares
`ugv-reset`, API `v1`, HTTP `http.v1`, Unix endpoint and `local_private`
authentication. Optional owner authorization is enforced by the shared loader.
No endpoint, target, instance or readiness probe is inferred.

`application` has exactly two fields: `sceneServiceRef` is the complete actual
`xgc2.simulation` / `v1` / `http.v1` reference returned by this Session's completed
world startup, and `worldBoundary` is the original frozen Session boundary JSON
(or explicit `null`). The scene cache binds that supplied incarnation directly;
it never calls describe or discovers a replacement. Controller fleet YAML and
per-robot tuning still come from their original launch/YAML namespaces.

Existing fleet launch files transport only `bootstrap_input` to the host. The
combined UAV/UGV wrappers retain their original `world_boundary_json` argument
for UAV products; the UGV owner consumes its frozen boundary from BootstrapInput.
`GET /v1/describe` exposes the newly created UGV ServiceRef. All business calls
bind the instance and use finite SDK deadlines.

## Reset contract

- `POST /v1/extensions/reset/start`: exactly
  `{"expected_revision":0,"robots":["ugv1"],"expected_scene":{"epoch":"world-boot","revision":1,"simulation_time_epoch":2}}`.
  The robot set is a nonempty unique subset of the frozen native roster. Targets
  are the original controller's configured/cached reset targets. Start checks
  current scene identity and actual freshness; `202 accepted` is admission,
  followed by `running` only after the native Reset session exists.
- `POST /v1/extensions/reset/cancel`: exactly
  `{"expected_revision":3,"operation":1}`. Cancellation reaches the original
  Stop event. `cancelled` requires native exit and a new actual zero publication.
- `GET /v1/extensions/reset`: current revision, operation, state and each robot's
  exact native generation/result, plus the observed `scene` fence and
  `scene.operational` admission evidence. It is false for stale science data or
  a failed instance-bound provider.
- `POST /v1/extensions/reset/observe`: exactly
  `{"operation":1,"after_revision":3}`. Holds until revision advances under the
  caller's SDK deadline. Future revisions and replaced operations conflict.

Results distinguish `arrived`, `rejected`, `expired`, and `cancelled`. Ready alone
is never success. Terminal native events survive `onExit`, and a fresh sole-owner
zero publication is required before publishing terminal completion. Replaced
native generations cannot complete an older operation. World rewind and either
90 s clock deadline stop the native owners before exposing expiry.

The endpoint has one SDK IO thread, 16 fixed management command slots and 16
held observer slots. The native turn uses nonblocking handoff; 50 Hz proposals,
feedback, commands and fleet math never make network calls. NMPC has 1..4 fixed
workers (default 2), one registered coalescing slot per robot (maximum 128);
original busy-solve rejection and correlation checks remain.

## Scene contract and execution fences

One fixed SDK scene IO owner consumes the complete frozen `xgc2.simulation`
ServiceRef, binds its incarnation, and caches only the latest snapshot. `GET /v1/extensions/scene` followed by bounded held
`POST /v1/extensions/scene/observe {"after_serial":N}` uses the frozen Gazebo
contract: top-level authoring epoch/revision/frame/stamp, definition, actual
state, applied/playing, and explicit `simulation_time {epoch,nanoseconds}`.
Nanoseconds parse as canonical decimal u64 strings, with no double conversion.
Pose positions and vector geometry use the provider's exact coordinate arrays
of three values; orientation uses four. The provider's existing geometry bounds
(256 obstacles, 128 parts per obstacle, 4096 total parts/vertices) remain.
A quiet held-observe deadline causes one GET of the same bound provider. Equal
serial is permitted only for that actual latest read; identical scientific
stamp/clock epoch/time does not republish cached evidence. Serial remains an
observation cursor. Both scene freshness clocks remain <=0.5 s;
paused clocks do not renew the native request/feedback lease. Authoring or
scientific epoch/revision changes reject the active operation and wait for actual
native zero output. A failed/mismatched provider does not permit a new reset.

## Isolated verification

`cmake -S ugv_fleet_host -B target/native -DCMAKE_BUILD_TYPE=Release`, build and
`ctest --test-dir target/native --output-on-failure` run ROS-free fleet math,
SDK host negative controls, native handoff, strict JSON/time, fixed worker and
original session/scenario suites. All builds require the installed SDK; there is no source fallback. Production
ROS builds require the installed SDK and
its C++20 toolchain, JsonCpp, existing Noetic controller dependencies and acados.

Optional `UGV_NATIVE_ACADOS_PREFIX`, `UGV_NATIVE_COMMON_ROOT`, and
`UGV_NATIVE_ROS_PREFIX` select existing isolated dependency evidence for genuine
controller core/edge/host validation; they do not install or manufacture a
solver. Current message headers were generated with official `gen_cpp` from
read-only current message sources into `target/generated/include`.

`test/native/verify.py` is an explicit opt-in workflow for an existing Classic
11.15.1 executable and native plugins, the built fleet host, and an existing
acados prefix. It starts a separate ROS master and headless world, deletes both
display variables, supplies explicit private sockets and target identities, and
records exact PID/start ticks/executable plus wait cleanup. It exercises the
actual mixed native controllers, instance-bound scene cache, arrived/cancel,
held observation, stale scientific admission and world reset rejection with new
sole-owner zero publications. `--static-scene` retains a stationary authoring
scene while the world runs. Its evidence certifies native semantics only.
