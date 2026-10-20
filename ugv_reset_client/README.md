# ugv_reset_client

The vehicle's side of the UGV Reset protocol with the station's coordinator (`ugv_reset_safety`). The vehicle owns
its motion, its 90 s deadline and its fail-stop; the coordinator only grants permission to move. Everything here is
header-only and ROS-free except `reset_ros.h`, so the controllers, the module entity and the scenario harness
exercise one definition.

| header | what it is |
|---|---|
| `reset_protocol.h` | The vocabulary as plain C++: `Session` (what a vehicle declares: footprint, limits, target, health, phase), `Grant` (HOLD, GO, BRAKE, REJECT; the frozen route and the free prefix of it), `Result` (the one typed outcome), `Reason`, `Outcome`, and the numbers of the requirement (90 s, 5 cm, 5 degrees). |
| `reset_route.h` | `Route`: the polyline of the rotation centre from the pose the route was planned from, through the via points, to the goal, with in-place turns at the vertices and forward or reverse per segment. Both sides sample it the same way. |
| `reset_tracker.h` | `VelocityEstimator`, `EllEstimator` (windowed rotation-centre offset of a skid-steer unicycle, learned only from turns in place) and `RouteTracker`: follows the segment direction, stops and turns in place at every vertex, stops inside the granted prefix with a lag-aware stopping profile, finishes with a latched final stage. |
| `reset_executor.h` | `ResetExecutor`: one session of one vehicle. Begin, grants in, command out, `Session` messages out, one `Result`. Its fail-stops: 90 s on the wall and the ROS clock; the heartbeat of the grant (a second after the last message the vehicle brakes to a stop inside its prefix and waits, and goes on if the coordinator is back); no message from the coordinator for 2 s (typed result AbortedCoordinatorLost, or RejectedNoCoordinator if none ever answered); no target (2 s); health lost; the route tube (cross-track); no progress for 10 s while allowed to move. |
| `reset_ros.h` | `ResetRosEdge`, the only ROS part: publishes `reset/session`, the latched `reset/result`, takes `reset/grant`. |

The controller owns the executor as a member: it feeds the measured pose and health every control cycle in every
CONTROL state (the offset is learned during the mission, before any Reset), asks for the command only in Reset, and maps
the one `Result` to its CONTROL transitions (`Arrived` to RESET_ARRIVED, `Timeout` to RESET_TIMEOUT, everything else to
RESET_REJECTED). The shared messages `ResetSession`, `ResetGrant`, `ResetResult` and `ResetFleet` are in `ugv_reset_msgs`
(ros1-msgs).

Used by `unicycle_ugv_controller`, `mecanum_ugv_controller` and the ROS edge of `ugv_modules`. A vehicle package knows one
vehicle and nothing of the others.

## What a vehicle declares

The footprint is the chassis plus a margin per side (Scout 0.62 x 0.52 m plus 0.14 m a side: 0.90 x 0.80 m; Wheeltec
0.41 x 0.36 m plus the same margin: 0.69 x 0.64 m), its speed and acceleration limits (0.35 m/s, 0.5 rad/s, 0.35 m/s^2,
0.6 rad/s^2 for the simulation profiles; not a physical certification), and for a unicycle the bound of its
rotation-centre offset (Scout 0.25 m, Wheeltec as a unicycle 0.05 m). The declared values are in each controller's YAML
(`reset/footprint/*`, `reset/limits/*`, `reset/lateral_offset_bound`) and in the module manifests.

## Tests

`reset_executor_test` drives the executor against a plant with lag, delay and a rotation-centre offset, without the
coordinator: arrival, tracking, the stopping profile, every fail-stop, the offset estimator, the Session message. The
closed loop with the real coordinator is in `ugv_integration_tests` (`reset_scenario_harness_test`).
