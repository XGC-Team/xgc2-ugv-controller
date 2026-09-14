# Consumed Flatness trace (harness #93)

This change fixes a diagnostic ambiguity, not the vehicle dynamics or the
low-speed controller. The numerical law and receipt-time PVA lift in
`src/common_types.cpp` are unchanged (git blob
`6101dab5e1a0d87f807f88533de39f0c0a030baa`). No estimator, switch, additional
input lag, future-reference queue, or Reset behavior is introduced.

## Enable from the node's YAML only

```yaml
flatness_trace_enabled: true
# Optional; otherwise the resolved command topic followed by /flatness_trace.
# flatness_trace_topic: /ugv1/cmd_vel/flatness_trace
```

The publisher is `std_msgs/String`, nonlatched, queue size 10, and generates
JSON only when subscribed. Record it alongside the original command, PVA,
state/pose, mission-clock, and source-planner diagnostics. Queue loss is a
coverage failure, not a zero-error sample. The command publication itself is
not retried or duplicated by diagnostics.

## Timing and frames

`reference_source_stamp_ns` and `reference_sequence` are copied from the PVA
header for identity. The header may be the planner's future stage epoch;
**it is not reinterpreted as communication latency**.
`reference_receive_stamp_ns` is the real callback reception epoch already
used by the tracker. At evaluation time t the existing lift is

    tau = max(0, t - t_receive)
    q_p = p_sample + v_sample*tau + 0.5*a_sample*tau^2
    q_v = v_sample + a_sample*tau
    q_a = a_sample

The trace stores that exact lifted value, the exact state passed to the
same evaluation, its measurement stamp, and the command integrator's value
before integration. The command and snapshot are committed under the same
existing mutex. A subsequent input callback cannot change the recorded
consumption. At command output, the computed and actually clamped published
(v,w) are both recorded. `publication_stamp_ns` precedes the ROS publish call;
it is not an acknowledgment that the vehicle executed the command.

All nanosecond epochs are decimal strings to avoid double-precision timestamp
rounding. x,y and vx,vy are in the existing world frame, yaw is body heading,
v is m/s, w is rad/s, acceleration is m/s^2. Parameter ordering is
[kp (s^-2), kv (s^-1), epsilon (m/s), L (m), zeta (dimensionless), vmax (m/s),
wmax (rad/s)]. Schemas and finite-value checks are in `common/flatness_trace.h`.

## Separate r-q from q-p

The consumed q-to-p error is computed directly from `consumed_q_*` and
`state_*` in the SAME snapshot, not a nearest-neighbor bag reference. The
physical pose/ground-truth error is a separate diagnostic because the
controller state may be filtered or observed at another rigid-body point.
The r-to-q comparison needs the planner's desired reference evaluated using
the same reference rule, stage, source stamp, and mission clock as the sent
PVA. A leader-position-only reconstruction must not be labeled desired r.
An old bag without this new snapshot can support an observer-receipt proxy,
which must remain labeled as such; it cannot be retroactively promoted to an
exact consumed-reference trace.

## Numerical regression (no ROS)

```sh
python3 unicycle_ugv_controller/test/flatness_estimated_speed_test.py
python3 unicycle_ugv_controller/test/flatness_consumption_trace_test.py
```

The second test compiles the actual production C++ numerical and tick bodies
and the production JSON formatter, substituting dependency/message structs.
It tests signed measured speed, command-state independence, receipt/source
identity, exact lifted q, immutable snapshots, repeated simulated time,
invalid-reference trace clearing, finite serialization, and uint64 precision.
It also preserves and demonstrates the exact zero-speed purely lateral
constant-jerk invariant. That counterexample is an applicability limitation,
not a new controller switch or a passing motion test.

CI run 34826748477 at commit 744ae5ff0eee88a1f29d5058f2db2ecdae61968c
passed both numerical tests and the unchanged-core hash check. Full catkin
ABI/linking, ROS topic coverage, scheduler latency, and actual four-vehicle
motion remain local integration/Experiment tests, not established here.

No merge, deployment, package publication, or closure of harness #93 is implied.
