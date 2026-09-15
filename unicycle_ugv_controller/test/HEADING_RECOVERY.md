# Continuous heading-centred damped inverse

The existing longitudinal PVA feedback, measured signed speed, speed-dependent transverse bandwidth, bounded command-speed integrator, command boxes and Stop/Reset semantics are retained. The optional change replaces the **zero target** in the scalar regularised inverse:

`min_w (s*w-a_perp)^2 + eps^2*(w-w_h)^2`

whose unique minimiser is `(s*a_perp + eps^2*w_h)/(s^2+eps^2)`. At zero speed this uses `w_h`, not an unbounded inverse or an angle-threshold controller switch.

Set `h = v_ref + (kp/kv)*(p_ref-p)`, `d=h/sqrt(dot(h,h)+eps^2)`, `r=|d|`, `c=t(theta)'d`, `e=n(theta)'d`. The heading potential is

`U = 0.5*e^2 + beta*r*(r-c)`, `-dU/dtheta = e*(c+beta*r)`.

For `0<beta<1`, forward and reverse axes are both local minima; the forward bias removes the stationary point at exactly 90 degrees without a forward/reverse mode switch. It does not remove every critical point. For a frozen nonzero h and ideal yaw actuation, `w = gain*(-dU/dtheta)` yields `U_dot = -gain*(dU/dtheta)^2`. This limited identity does **not** prove full PVA tracking stability, global convergence, or absence of oscillation with a moving target, skid, delay or saturation.

The target used by the controller is

`w_h = w_ref_reg + gain*(-dU/dtheta) - rate_damping*(w_measured-w_ref_reg)`

with `w_ref_reg=(vx_ref*ay_ref-vy_ref*ax_ref)/(vx_ref^2+vy_ref^2+eps^2)`. The rate feedback adds damping around the regularised feedforward value. It requires a finite measured yaw rate when enabled. All normalisations are scaled to avoid squared-norm overflow; invalid outputs are rejected before saturation. No `atan2(0,0)`, signed-epsilon denominator, extra integrator, angle trigger, or stop/go phase is introduced.

## Configuration and tests

Strict private-node map for the flatness strategy:

```yaml
flatness:
  heading_recovery:
    gain: 0.0
    axis_bias: 0.5
    rate_damping: 0.5
```

The default gain is zero: no production behaviour is changed merely by updating the source. Nonzero gain under NMPC and invalid/unknown parameter fields are rejected. For an isolated candidate trial choose a nonzero gain explicitly; `1.0` in the unit tests is a mathematical test value, not a calibrated Scout recommendation. Keep the existing complete controller profile and add this nested map, rather than replacing it with this fragment.

Run `python3 test/run_heading_recovery_test.py` from the package for the standalone C++17 kernel checks. CMake also registers `heading_recovery_kernel_test` and six GTests in `heading_recovery_runtime_test`, which call the production `computeFlatnessCommand` function. The latter require the normal ROS/catkin dependencies. Mathematical unit tests do not substitute for the existing skid replay, source build, fixed PVA, command-rate/Stop tests and full multi-vehicle Gazebo A/B comparison. In particular, position RMS and yaw sign reversals must both be evaluated before enabling a production gain.
