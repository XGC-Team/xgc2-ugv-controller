/* The payload header and the module header compile as C11, and C sees the same layout as C++. */
#include <stddef.h>
#include <stdint.h>
#include <xgc2/module.h>

#include "xgc2_ugv/payloads.h"

/* sizeof, alignof and the offset of the last member of each payload, in a fixed order. */
size_t ugv_payload_layout_in_c(size_t* out, size_t capacity) {
  const size_t layout[] = {
      sizeof(xgc2_ugv_planar_state),         _Alignof(xgc2_ugv_planar_state),
      offsetof(xgc2_ugv_planar_state, reserved),
      sizeof(xgc2_ugv_command),              _Alignof(xgc2_ugv_command),
      offsetof(xgc2_ugv_command, source),
      sizeof(xgc2_ugv_reset_target),         _Alignof(xgc2_ugv_reset_target),
      offsetof(xgc2_ugv_reset_target, yaw),
      sizeof(xgc2_ugv_reset_clearance),      _Alignof(xgc2_ugv_reset_clearance),
      offsetof(xgc2_ugv_reset_clearance, status),
      sizeof(xgc2_ugv_reset_session),        _Alignof(xgc2_ugv_reset_session),
      offsetof(xgc2_ugv_reset_session, flags),
      sizeof(xgc2_ugv_cmd_vel),              _Alignof(xgc2_ugv_cmd_vel),
      offsetof(xgc2_ugv_cmd_vel, reserved),
      sizeof(xgc2_ugv_controller_status),    _Alignof(xgc2_ugv_controller_status),
      offsetof(xgc2_ugv_controller_status, control_state_name),
      sizeof(xgc2_ugv_planar_pva),           _Alignof(xgc2_ugv_planar_pva),
      offsetof(xgc2_ugv_planar_pva, ay),
      sizeof(xgc2_ugv_analytic_reference),   _Alignof(xgc2_ugv_analytic_reference),
      offsetof(xgc2_ugv_analytic_reference, reserved),
      sizeof(xgc2_ugv_planar_point),         _Alignof(xgc2_ugv_planar_point),
      offsetof(xgc2_ugv_planar_point, jy),
      sizeof(xgc2_ugv_sampled_reference),    _Alignof(xgc2_ugv_sampled_reference),
      offsetof(xgc2_ugv_sampled_reference, points),
      sizeof(xgc2_ugv_polynomial_reference), _Alignof(xgc2_ugv_polynomial_reference),
      offsetof(xgc2_ugv_polynomial_reference, coeff_yaw),
      sizeof(xgc2_ugv_waypoint),             _Alignof(xgc2_ugv_waypoint),
      offsetof(xgc2_ugv_waypoint, qw),
      sizeof(xgc2_ugv_waypoint_request),     _Alignof(xgc2_ugv_waypoint_request),
      offsetof(xgc2_ugv_waypoint_request, segment_times),
      sizeof(xgc2_ugv_reference_status),     _Alignof(xgc2_ugv_reference_status),
      offsetof(xgc2_ugv_reference_status, reserved),
      sizeof(xgc2_ugv_reference_reset),      _Alignof(xgc2_ugv_reference_reset),
      offsetof(xgc2_ugv_reference_reset, stamp_ns),
  };
  const size_t count = sizeof layout / sizeof layout[0];
  for (size_t i = 0; i < count && i < capacity; ++i) {
    out[i] = layout[i];
  }
  return count;
}
