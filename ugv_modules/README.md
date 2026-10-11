# ugv_modules

The AgileX Scout unicycle chain as [xgc2-module](https://github.com/XGC-Team/xgc2-module) modules: the
planar reference generator, the NMPC / flatness controller and the vehicle's ROS1 edge run in one
`xgc2-module-host` process, handing typed samples to each other in memory instead of through ROS topics.
The cores are the ones the ROS nodes run: `unicycle_reference_trajectory_core` and
`unicycle_ugv_controller_core`. A module is a port layout, a configuration and a step around a core.

```text
  ROS side                         one xgc2-module-host process (entity "scout")
  ---------                        -------------------------------------------------------------
  state estimate / pose  ->  edge --state-->      controller --cmd_vel------>  edge  -> cmd_vel
  /command, command      ->       --command-->    (NMPC on its own thread)      |    -> custom/statustext
  reset_pose             ->       --reset_target->     |  \---status----------->  |
  reset/response         ->       --reset_clearance->  |  \---reset_session---->  |--> reset/request
  alg/reference/pva      ->       --pva------------>   ^
  .../request/{analytic,sampled,waypoint}, .../reset                              |
                         ->       --*_request, reference_reset--> reference --active_*, status--> controller, edge
  /clock (simulation)    ->       --clock--> the host's clock                      -> .../status, .../active/*
```

The drivers, the state estimator and the Reset coordinator stay separate ROS nodes. The entity replaces
`unicycle_reference_trajectory_node` and `unicycle_ugv_controller_node` with the same topics, parameters
(see Configuration) and operator commands.

## Run it

```bash
source /opt/ros/noetic/setup.bash          # the edge is a ROS node: master, libraries, ROS_HOME
xgc2-module-host --manifest /opt/ros/noetic/share/ugv_modules/manifests/scout_unicycle.toml --check
xgc2-module-host --manifest /opt/ros/noetic/share/ugv_modules/manifests/scout_unicycle.toml
```

`scout_unicycle_sim.toml` is the same entity on simulation time (Gazebo or xsim with `/use_sim_time`): the
host's clock is the `/clock` topic, which the edge writes to the host; the entity is not ready before the
simulator publishes time, and the host's timers run at the simulator's speed. It differs from the
vehicle's manifest in `[clock]`, in the edge's `sim_time = true` and in the edge's `clock` binding, and a
test keeps it so.

`[control] socket` names the XRPC control plane (`GET /v1/describe`, `/v1/health`, live
`instances/configure`, `replace`, ...). Its directory has to exist, belong to the user that runs the host
and have mode 0700; `--control-socket` overrides it. The control plane is documented with xgc2-module.

The manifests carry the values of `config/unicycle_reference_trajectory.yaml` and
`config/unicycle_ugv_controller.yaml` of the two nodes, minus the topic names (the edge keeps the nodes'
topics) and minus `main_frequency` and `control_rate_hz`: the period of an instance is its loop rate
(`period_ms` = 10 for the generator, 2 for the controller, 10 for the edge). A test compares the manifest
with both YAML files, so a key that is added to a node configuration and not here fails it.

## Modules

All payloads are in [`xgc2_ugv/payloads.h`](include/xgc2_ugv/payloads.h): plain data, C11 and C++, schema
ids `xgc2.ugv.*.v1`, size and alignment asserted. Times are nanoseconds in the host clock (0 = not set),
lengths metres, angles radians, the world frame ENU. A list that does not fit its payload is refused
where it enters, never truncated. A module exports `xgc2_module_entry` and nothing else.

**`ugv_unicycle_reference`** (period 10 ms)

| port | dir | kind | payload |
|---|---|---|---|
| `analytic_request` | in | event, 8 | analytic reference |
| `sampled_request` | in | event, 4 | sampled reference (up to 512 points) |
| `waypoint_request` | in | event, 4 | waypoints (up to 64), fixed-time septic interpolation |
| `reset` | in | event, 2 | restart the generator |
| `status` | out | state | state, flags, active trajectory id / revision / type |
| `active_analytic`, `active_polynomial`, `active_sampled` | out | state | the active reference |

Requests are served in the order of their commit stamps. A live `configure` restarts the generator, as a
start of the node does.

**`ugv_unicycle_controller`** (period 2 ms; the NMPC solve runs on a thread of the module, which wakes
the instance when it has a result)

| port | dir | kind | payload |
|---|---|---|---|
| `state` (required) | in | state | planar state: estimate (`state_source: state_estimator`) or pose |
| `command` | in | event, 8 | start tracking / stop / reset |
| `reset_target`, `reset_clearance` | in | state | the Reset target; the coordinator's response after the lease validated it |
| `active_analytic`, `active_polynomial`, `active_sampled` | in | state | the NMPC strategy's reference |
| `pva` | in | state | the flatness strategy's world PVA reference |
| `cmd_vel` | out | state | the twist for the chassis, after saturation; two decisions of one step are one commit, the later one |
| `status` | out | state | control state name and id (at `status_publish_rate_hz`) |
| `reset_session` | out | state | the Reset session: generation, frozen target, active, healthy |

The state of a pose source carries no velocities; the controller's own pose filter makes them. A state of
the other source than the configured one is refused with a warning, not silently ignored.

**`ugv_ros_edge`** (period 10 ms: the Reset lease asks for a request about every 20 ms). The one module
that links ROS. ROS callbacks run on one thread of the module and are the only writers of its output
ports (all are `ASYNC_WRITER`); the step reads the input ports and publishes. Stamps are taken from the
receipt time of the message (`ros::MessageEvent`) and converted between ROS time and the host clock; a
time that is zero stays zero. With `sim_time` they are the same clock and nothing is converted, and the
module refuses to start when `sim_time` and `/use_sim_time` disagree. The edge takes the topics of the
nodes, a `namespace` (the vehicle's, as the launch files' `ns`) and the configuration keys listed in
`src/ugv_ros_edge.cpp`; `reset_request_topic` and `reset_response_topic` replace the launch file's remap.

## Configuration

JSON objects (TOML tables in the manifests). A key that is absent keeps the default of the node; a key a
module does not know, or a value of the wrong type, is an error that `create` and a live `configure`
refuse, and a refused `configure` changes nothing. `control_rate_hz` is refused by the controller module:
set `period_ms`.

## What differs from the nodes

- No `reference_path` visualization topic of the generator, no NMPC `predicted_path` topics: they are
  visualization of the cores' internals and the module chain does not publish them.
- The active references the edge publishes carry the frame `world`; a payload has no frame or sequence.
- `rigid_state_estimator_msgs` stays in the edge (the projection to the plane); the cores know none.
- The generator and the controller write to the log of the host (`controller: ...`), not to rosconsole.

## Threads and the host

The state machine of the cores belongs to the thread that built it and refuses every other one
(`operation called from non-owner thread`), while the host runs the calls of an instance on whichever
worker is free. The generator and the controller therefore build their cores on a thread of their own
(`OwnerThread` in `src/module_support.h`) and run every step there; the reads and writes of the ports stay
on the host's thread. A step costs a thread hand-off (some tens of microseconds). Without this the
generator stopped in `SelfCheck` with the invalid-input flag in the real host. A host that keeps an
instance on one thread would make it unnecessary.

roscpp can be started once in a process. The first edge instance starts it (node name and master from its
configuration), later ones share it, and it ends with the process. The library is linked `-z nodelete`, so
unloading or replacing it leaves ROS running. Without a reachable master `start` fails at once.

## Tests

| | |
|---|---|
| `reference_module_test`, `controller_module_test` | the modules as shared libraries, loaded the way the host loads them, in an in-test host (`test/test_host.h`): ports, strict configuration, health, the Reset session and lease, NMPC and flatness tracking against a kinematic unicycle, whole outputs, refused writes, calls from a ring of threads |
| `chain_parity_test` | the module chain against the two nodes' chain without ROS, on the replay scenario of the generator: every output of the generator, every twist of the controller and the vehicle's path equal bit for bit |
| `ros_edge_test`, `ros_edge_sim_test`, `ros_edge_no_master_test` | the edge against the master that rostest starts: wall-clock and simulation-time masters, the Reset lease, stamps, refused requests, reconnects |
| `payload_conversion_test`, `ugv_modules_manifests`, `ugv_modules_linkage` | the payload layout in C and C++, the conversions, the manifests against the node configurations (and the host's `--check`), what the libraries export and link |
| `ugv_integration_tests` | the real `xgc2-module-host`: the Reset coordinator test with the module entity as the Scout, the chain against a kinematic Scout over ROS and the control socket, the simulation-time manifest at twice real time |

The tests that need the host binary find it in `$XGC2_MODULE_HOST` or on the `PATH`; without one they are
not registered, and the configure step says so. `tools/bionic_compile_check.sh` compiles the ROS-free
sources and the module tests with the Ubuntu 18.04 g++ 7.5 (compile only; see the script for its scope).

## Build and install

`find_package(Xgc2Module CONFIG REQUIRED)` (the package `libxgc2-module-dev` of xgc2-module) and
`libjsoncpp-dev` at build time. The libraries go to `lib/ugv_modules`, the manifests to
`share/ugv_modules/manifests`, whose relative paths find them, and the payload header to
`include/xgc2_ugv`. The host (`xgc2-module-host`) is a recommended package, not a dependency of the ROS
nodes.
