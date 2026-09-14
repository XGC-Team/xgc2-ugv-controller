# Flatness evaluation and publication diagnostic, version 1

Related to XGC-Team/xgc2-harness#93. Observation only; no controller gain, state
estimator, low-speed mode, reference timestamp, Reset logic or NMPC change.
The existing signed estimated-speed inverse in `common_types.cpp` is unchanged.

A valid flatness command now carries an immutable snapshot of the exact state,
lifted PVA, command-integrator input and result used in its evaluation. The output
consumer reads ONE ControlCommand snapshot, computes its bounded Twist and
publishes both that Twist and the corresponding observation at
`<cmd_vel_topic>/flatness_diagnostic`. It does not resample controller state or
reference at publication. At the default configuration this is bounded by the
existing 30 Hz command publication gate. Idle/invalid/NMPC/Reset commands carry
no valid flatness observation; this diagnostic is not a command heartbeat.

Wire type: `std_msgs/Float64MultiArray`, 25 doubles. `layout.dim[0].label` is the
versioned CSV-like schema in `flatnessDiagnosticSchema()`, `size=stride=25`.
ROS timestamps use seconds. Position is metres, velocity m/s, acceleration m/s^2,
yaw radians and yaw rate rad/s. World-frame x/y values are unchanged from the
feedback and reference used by production. Body projections are

    v_parallel = cos(yaw)*vx + sin(yaw)*vy
    v_perp     = -sin(yaw)*vx + cos(yaw)*vy.

Array indices (zero-based):
0 evaluation time; 1 state source stamp; 2 PVA RECEIPT stamp (not planner header);
3 integration dt; 4 command-integrator speed before the evaluation;
5/6 estimated signed body longitudinal/lateral speed;
7/8 feedback x/y; 9 yaw; 10/11 estimated world vx/vy; 12 estimated yaw rate;
13/14 lifted q x/y; 15/16 q vx/vy; 17/18 q ax/ay;
19/20 solved bounded v/yaw-rate; 21 longitudinal acceleration;
22 time sampled immediately before the Twist publish invocation;
23/24 final published bounded v/yaw-rate.

The receipt-time constant-acceleration reference lift remains intentional.
`liftWorldPva` retains the receipt stamp, so its age is index0-index2; state age
is index0-index1. These are NOT assumed zero or aligned by an optimized shift.
The instantaneous feedback error is (index13-index7,index14-index8). It is the
exact error input used in this evaluation, not proof of simultaneous physical
truth: the state source stamp may be older. For true q-p at evaluation time,
interpolate independently recorded physics truth only within its support at
index0 and report state age separately. For field motion retain the marker-point
and derivative-filter scope. Do not compare a marker-origin velocity with a
base_link velocity without a known transform.

r-q remains a separate planner-side stage comparison: associate the PVA source
header minus dt with the recorded mission trigger, reconstruct r through the
production formation/stadium reference with frozen within-cycle offsets, and
compare to q at that same stage. The diagnostic's receipt stamp must not be used
to manufacture that source-stage association. No new source-header queue is added.

## Tests and local acceptance

`python3 unicycle_ugv_controller/test/issue93/test_snapshot_diagnostic.py` compiles
the actual independent header under ASan/UBSan and checks signed speed versus
command speed, immutable state/reference snapshots, publication clipping versus
solved output, invalid observations and rotated frames. A source hash check proves
`common_types.cpp` is byte-for-byte the baseline; the existing actual-function
flatness regression is run separately. These are not a catkin build or ROS
publisher/state-machine integration test.

Before local acceptance, build the entire source package, record its binary hash,
record this topic with /pose, /cmd_vel, PVA/source ticks and physics truth, and
check one diagnostic row per valid published flatness command with the same
values. Include repeated /clock stamps, clock discontinuities, saturation, source
state aging and final stop. Do not introduce diagnostic traffic into control
feedback or declare full four-vehicle acceptance from these unit tests.
