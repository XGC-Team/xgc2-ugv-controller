# Coordinated UGV Reset

This package is the station side of Reset for Scout and Wheeltec. It does not change UAV control or Custom1 tracking.
One coordinator per Session grants vehicles permission to move; each chassis controller remains the sole publisher of
native `cmd_vel` and owns its motion, its 90 s deadline and its fail-stop (`ugv_reset_client`).

## Who decides what

* A **vehicle** declares itself in its `reset/session` message (5 Hz, also while idle, and at once on every change):
  footprint, limits, target, pose, health and, for a unicycle, the rotation-centre offset and the spread it still
  needs covered. There is no roster file and no fleet configuration. A vehicle that never sends a session does not
  exist for the coordinator; one that falls silent stays a parked body at its last pose.
* The **coordinator** (`ugv_reset_coordinator_node`) turns the sessions into **grants**: a frozen, exactly verified
  route and the prefix of it that is free (`reset/grant`, HOLD / GO / BRAKE / REJECT with a reason and the names of the
  blockers). It repeats an unchanged grant every 0.4 s as the vehicles' heartbeat (ttl 1 s). It never publishes
  `cmd_vel` and never subscribes `/command`.
* Every session ends with exactly one typed `ResetResult` from the vehicle (`reset/result`, latched): Arrived, Timeout,
  Stopped, a rejection (no target, no coordinator, by the coordinator), Infeasible (with the reason), or a vehicle
  abort (route tube, stalled, coordinator lost, unhealthy). The coordinator also publishes the fleet's state at 5 Hz on
  `/ugv/reset/fleet` (`ResetFleet`).
* **Fault isolation:** a fault is the faulty vehicle's alone. An invalid declaration, a lost vehicle, a vehicle without
  target or pose refuses only that vehicle; the others keep their grants and treat it as a parked body. There is no
  fleet-wide rejection.

The coordinator is a Session-managed process (`xgc.system.ground-control`, process definition
`xgc2-ugv-reset-coordinator-ros1` 2.x). Configuration is the environment of the process, because a Session-managed
definition fills every argument and an empty argument is not a parameter:

| variable | meaning |
|---|---|
| `XGC_RESET_FENCE` | the Session's frozen world control bounds as `x_min,x_max,y_min,y_max`; empty: no fence |
| `XGC_RESET_SCENE_NAMESPACE` | the Scene adapter's ROS namespace (for example `/xgc_scene`); empty: no Scene authored |

A misconfigured value is the node's own non-zero exit, so a Session-managed instance fails its Run instead of idling.
`ugv_reset_coordinator.launch` starts the same node for launches that still compose it (it accepts and ignores
`config_file`; takes `scene_namespace` and `fence`).

## The planner

`ResetPlanner` (ROS-free; `reset_planner.h`) plans a route, verifies it exactly, and grants the free prefix of it.

* **Route:** a visibility graph around the statics and the parked bodies (`planVisibilityPath`), as the polyline of the
  vehicle's rotation centre, with in-place turns at the vertices and forward or reverse travel per segment. A
  unicycle's route is verified for the three offsets `{ell - spread, ell, ell + spread}` (the declared bound when the
  offset is not yet known); an omnidirectional footprint is the circumscribed polygon of its rectangle, which is sound
  for any heading schedule.
* **Exact verification:** the swept footprint is sampled (2 cm, 1.5 degrees) and checked with a separating-axis test
  against every static, the fence and every other body: parked bodies and the pose of movers, and the remaining
  reserved sweep of movers. Each margin (pair 0.10 m, static 0.12 m, fence 0.05 m) carries the sampling error of both
  bodies explicitly, so the numbers hold between the samples as well. Margins are between the declared envelopes, which
  already include 0.14 m a side around the chassis.
* **Rolling prefix:** a vehicle is granted the part of its route that is free, less a hold margin (0.45 m) for braking
  and tracking slack; the prefix is extended every 0.2 s as the movers ahead pass. A parked body that blocks a held
  mover's remaining route (or a Scene change under it) revokes the grant (BRAKE) and re-plans from where the vehicle
  stands. Reservation is of space, not of time, so it is insensitive to transport delay and plant lag.
* **Ordering:** a waiting vehicle yields to another whose static route passes its goal ("that one passes first") or
  whose route runs through its start. A cycle of such deferrals is broken in id order, at once if the route gets 2 m of
  free prefix, else when nobody moves; if nobody can move, one parking move that makes progress is granted; else the
  vehicles are refused with the blockers' names (Deadlock) and a stall of 10 s of the whole fleet rejects the held
  movers (Stalled). A route that needs more than 1.5 times the limit is refused before any motion (TooSlow).
* **Not complete:** coordinating rectangles is PSPACE-hard; an instance the planner cannot solve is reported with
  reasons, never held silently. A body outside any session that moves is a parked body at its pose and no new route is
  granted while it moves.

## Scene and admissibility

The coordinator consumes `xgc2_geometry_msgs/SceneSnapshot` and `SceneState` under the Scene namespace. An explicit
empty snapshot is valid; absent or stale scene data is not an empty scene: with a Scene authored and not live, nobody
is granted a new route and each waiting vehicle is told why (SceneUnavailable). Epoch/revision changes under a held
route revoke it. Each compound part projects independently into 32 supporting XY halfspaces; curves and oriented
meshes use conservative planar outer approximations; compound openings are not filled by a shared hull. With no height
envelope, overhead geometry also occupies XY. This does not certify 3D passage. Live Scene occupancy includes the
configured 2 s translational and rotational motion envelope; unsupported motion or geometry reports a capability gap.

## Limits

* The rotation-centre offset of a skid-steer unicycle is learned from its own turns. Until it is known (three samples
  and half a radian of turning) the planner covers the declared bound, which makes tight layouts slower; after that the
  estimate and its spread are covered. A wrong estimate shifts the true centre of a turn away from the planned vertex
  by the error; the margins and the route tube (0.10 m plus twice the covered spread) absorb it, and a larger
  deviation aborts the vehicle (AbortedDeviation) instead of continuing.
* The tracker's deviation from the route consumes part of the 0.10 m design margin; in the closed-loop harness it stays
  within 0.14 m and the physical chassis (0.14 m a side smaller than the envelope) never touched anything.
* Footprint, rotation-centre bound and lag are simulation values; they must be measured on the real Scout and Wheeltec.
* Hardware safety is not certified by these tests.

## Verification

* `reset_planner_test`: soundness of every grant against an independent 2 ms separating-axis audit of the followed
  routes, determinism, revocation, parking, early refusal, stall, scene change, the Scout family of 80 scenarios with
  ideal followers (0 collisions) and the omnidirectional footprint.
* `reset_coordinator_test`: sessions in, grants out, heartbeat, isolation of every kind of single-vehicle fault.
* `reset_path_test`, `scene_projection_test`: the route and the Scene projection.
* `ugv_integration_tests`: the closed-loop scenario harness (real coordinator, real executors, a plant with lag, offset,
  delay, noise, bus loss and a coordinator outage, a 10 ms physical-chassis audit) on the 235 TASE scenarios
  (`ResetScenarios.QuickFamily` with the package tests, `FullFamily` with `RESET_SCENARIOS_FULL=1`), and rostests with
  the real nodes against synthetic vehicles.
* The slot pose and world boundary launch tests cover the launch-time helpers of the controllers
  (`launch_with_yaml_context.py`, `launch_control_with_slot_poses.py`), which are installed with this package for
  historical reasons and are not part of the coordinator.
