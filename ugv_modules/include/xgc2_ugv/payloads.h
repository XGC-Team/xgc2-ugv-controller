/*
 * xgc2_ugv/payloads.h - the payloads of the ground-vehicle modules of xgc2-ugv-controller.
 *
 * The ports of ugv_unicycle_reference, ugv_unicycle_controller and ugv_ros_edge carry these
 * fixed-size structs (the module ABI of xgc2-module: plain data, no pointers, one schema id each).
 * Every schema id, size and alignment is checked by the host when a port is bound to a channel, and
 * by the static assertions below when a producer or consumer is compiled. Layout changes need a new
 * schema version.
 *
 * Conventions:
 * - Times named *_ns are nanoseconds in the clock domain of the host (steady, or external in
 * simulation); 0 means "not set". The ROS edge converts every ROS stamp into this domain.
 * - Lengths in m, angles in rad, the world frame is ENU. The vehicle frame is x forward, y left.
 * - Variable-length lists of the ROS messages have a count and a fixed capacity. A request that
 * exceeds a capacity is refused by the module that receives it from ROS, never truncated.
 * - Reserved fields are zero.
 *
 * C11 and C++11 compatible.
 */
#ifndef XGC2_UGV_PAYLOADS_H
#define XGC2_UGV_PAYLOADS_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------------------------------------
 */
/* Schema ids */

#define XGC2_UGV_SCHEMA_PLANAR_STATE "xgc2.ugv.planar_state.v1"
#define XGC2_UGV_SCHEMA_COMMAND "xgc2.ugv.command.v1"
#define XGC2_UGV_SCHEMA_RESET_TARGET "xgc2.ugv.reset_target.v1"
#define XGC2_UGV_SCHEMA_RESET_GRANT "xgc2.ugv.reset_grant.v1"
#define XGC2_UGV_SCHEMA_RESET_SESSION "xgc2.ugv.reset_session.v2"
#define XGC2_UGV_SCHEMA_RESET_RESULT "xgc2.ugv.reset_result.v1"
#define XGC2_UGV_SCHEMA_CMD_VEL "xgc2.ugv.cmd_vel.v1"
#define XGC2_UGV_SCHEMA_CONTROLLER_STATUS "xgc2.ugv.controller_status.v1"
#define XGC2_UGV_SCHEMA_PLANAR_PVA "xgc2.ugv.planar_pva.v1"
#define XGC2_UGV_SCHEMA_ANALYTIC_REFERENCE "xgc2.ugv.unicycle_reference.analytic.v1"
#define XGC2_UGV_SCHEMA_SAMPLED_REFERENCE "xgc2.ugv.unicycle_reference.sampled.v1"
#define XGC2_UGV_SCHEMA_POLYNOMIAL_REFERENCE "xgc2.ugv.unicycle_reference.polynomial.v1"
#define XGC2_UGV_SCHEMA_WAYPOINT_REQUEST "xgc2.ugv.unicycle_reference.waypoint_request.v1"
#define XGC2_UGV_SCHEMA_REFERENCE_STATUS "xgc2.ugv.unicycle_reference.status.v1"
#define XGC2_UGV_SCHEMA_REFERENCE_RESET "xgc2.ugv.unicycle_reference.reset.v1"

/* ------------------------------------------------------------------------------------------------
 */
/* Capacities of the reference payloads */

#define XGC2_UGV_MAX_ANALYTIC_PARAMS 8u
#define XGC2_UGV_MAX_SAMPLED_POINTS 512u
#define XGC2_UGV_MAX_POLYNOMIAL_SEGMENTS 64u
/* Order 7: one segment has order + 1 coefficients per axis. */
#define XGC2_UGV_POLYNOMIAL_COEFFS 8u
#define XGC2_UGV_MAX_WAYPOINTS 64u

/* ------------------------------------------------------------------------------------------------
 */
/* Vehicle state (edge -> controller) */

#define XGC2_UGV_STATE_SOURCE_ESTIMATE 0u /* a rigid state estimate projected to the plane */
#define XGC2_UGV_STATE_SOURCE_POSE 1u     /* a canonical pose; velocities are not provided */

/* State port. Planar projection of the vehicle state, the plain form of the controller's UgvState.
 */
typedef struct xgc2_ugv_planar_state {
    int64_t stamp_ns;         /* measurement time (the message stamp, else the receipt time) */
    double x, y, yaw;         /* world pose */
    double vx, vy;            /* world velocity; 0 for a pose source */
    double speed;             /* forward speed in the body frame; 0 for a pose source */
    double yaw_rate;          /* 0 for a pose source */
    uint32_t estimator_state; /* RigidStateEstimate.estimator_state; 0 for a pose source */
    uint32_t estimator_flags; /* RigidStateEstimate.flags; 0 for a pose source */
    uint8_t source;           /* XGC2_UGV_STATE_SOURCE_* */
    uint8_t velocity_valid;   /* vx and vy are finite estimator values */
    uint8_t reserved[6];
} xgc2_ugv_planar_state;

/* ------------------------------------------------------------------------------------------------
 */
/* Commands and Reset (edge <-> controller) */

#define XGC2_UGV_COMMAND_CUSTOM1 1u           /* start tracking */
#define XGC2_UGV_COMMAND_STOP 2u              /* stop / hold */
#define XGC2_UGV_COMMAND_RESET 3u             /* drive to the Reset target */
#define XGC2_UGV_COMMAND_SOURCE_NAMESPACED 0u /* "command" in the vehicle's namespace */
#define XGC2_UGV_COMMAND_SOURCE_PUBLIC 1u     /* "/command" */

/* Event port. One operator command. */
typedef struct xgc2_ugv_command {
    int64_t stamp_ns; /* receipt time */
    uint32_t kind;    /* XGC2_UGV_COMMAND_* */
    uint32_t source;  /* XGC2_UGV_COMMAND_SOURCE_* */
} xgc2_ugv_command;

/* State port. The pose the Reset session drives to (world frame). */
typedef struct xgc2_ugv_reset_target {
    double x, y, yaw;
} xgc2_ugv_reset_target;

/* Reset sessions: the numbers are those of ugv_reset_client/reset_protocol.h and of ugv_reset_msgs. */
#define XGC2_UGV_RESET_PHASE_IDLE 0u
#define XGC2_UGV_RESET_PHASE_REQUESTING 1u
#define XGC2_UGV_RESET_PHASE_WAITING 2u
#define XGC2_UGV_RESET_PHASE_MOVING 3u
#define XGC2_UGV_RESET_PHASE_SETTLING 4u
#define XGC2_UGV_RESET_PHASE_DONE 5u
#define XGC2_UGV_RESET_MODEL_UNICYCLE 0u
#define XGC2_UGV_RESET_MODEL_OMNI 1u

/*
 * State port (controller -> edge). The vehicle's side of its Reset session: what the edge publishes as
 * ResetSession at 5 Hz and on change (the controller's executor decides when it is due). Times are on
 * the host clock.
 */
typedef struct xgc2_ugv_reset_session {
    int64_t stamp_ns;
    int64_t pose_stamp_ns;
    double pose_x, pose_y, pose_yaw;
    double target_x, target_y, target_yaw;
    double progress, cross_track;
    double half_length, half_width, offset_x, offset_y; /* the declared footprint */
    double max_v, max_vy, max_w, acc_v, acc_w;           /* the limits of the Reset tracker */
    double ell_hat, ell_spread, ell_bound;               /* rotation-centre offset; spread < 0: unknown */
    uint32_t session;
    uint32_t phase; /* XGC2_UGV_RESET_PHASE_* */
    uint32_t model; /* XGC2_UGV_RESET_MODEL_* */
    uint32_t healthy;
} xgc2_ugv_reset_session;

#define XGC2_UGV_RESET_GRANT_HOLD 0u
#define XGC2_UGV_RESET_GRANT_GO 1u
#define XGC2_UGV_RESET_GRANT_BRAKE 2u
#define XGC2_UGV_RESET_GRANT_REJECT 3u
#define XGC2_UGV_MAX_RESET_VIA 24u
#define XGC2_UGV_RESET_TEXT 96u

/*
 * Event port (edge -> controller, queue 16). One message of the coordinator for the vehicle's session:
 * a frozen route and the prefix of it that may be driven, or a hold, a brake, a rejection. The ttl is
 * counted by the controller on the wall clock from the moment it takes the message.
 */
typedef struct xgc2_ugv_reset_grant {
    int64_t stamp_ns; /* receipt time on the host clock */
    double start_x, start_y, start_yaw;
    double goal_x, goal_y, goal_yaw;
    double ell_plan;
    double s_limit;
    double ttl;
    double via[XGC2_UGV_MAX_RESET_VIA][2]; /* interior vertices of the rotation-centre polyline */
    uint32_t session;
    uint32_t state; /* XGC2_UGV_RESET_GRANT_* */
    uint32_t grant_id;
    uint32_t via_count;
    uint32_t reason;     /* ResetGrant.REASON_* */
    uint32_t final_move; /* 1: the route ends at the target */
    char detail[XGC2_UGV_RESET_TEXT]; /* the coordinator's text, with the blockers' names appended */
} xgc2_ugv_reset_grant;

/* Event port (controller -> edge, queue 4). The one outcome of a session. */
typedef struct xgc2_ugv_reset_result {
    int64_t stamp_ns;
    double final_planar_error, final_yaw_error, duration;
    uint32_t session;
    uint32_t outcome; /* ResetResult.OUTCOME_* */
    uint32_t reason;
    uint32_t reserved;
    char detail[XGC2_UGV_RESET_TEXT];
} xgc2_ugv_reset_result;

/* ------------------------------------------------------------------------------------------------
 */
/* Controller outputs (controller -> edge) */

#define XGC2_UGV_CMD_VEL_COMMAND 0u /* the tracking or Reset command */
#define XGC2_UGV_CMD_VEL_ZERO 1u    /* an idle or stop zero */

/* State port. The twist for the chassis, after saturation. The controller is the only writer. */
typedef struct xgc2_ugv_cmd_vel {
    int64_t stamp_ns; /* host clock when the controller emitted it */
    double linear_x, linear_y, angular_z;
    uint32_t kind; /* XGC2_UGV_CMD_VEL_* */
    uint32_t reserved;
} xgc2_ugv_cmd_vel;

#define XGC2_UGV_CONTROL_SELF_CHECK 1u
#define XGC2_UGV_CONTROL_READY 2u
#define XGC2_UGV_CONTROL_CUSTOM1 3u
#define XGC2_UGV_CONTROL_RESET 5u

/* State port. The controller's control state, for operators and for the Reset coordinator. */
typedef struct xgc2_ugv_controller_status {
    int64_t stamp_ns;
    uint32_t control_state; /* XGC2_UGV_CONTROL_*, 0 unknown */
    uint32_t health_state;
    char control_state_name[24]; /* "SelfCheck", "Ready", "Custom1", "Reset"; NUL terminated */
} xgc2_ugv_controller_status;

/* State port. World-frame position, velocity and acceleration reference of the flatness strategy.
 */
typedef struct xgc2_ugv_planar_pva {
    int64_t stamp_ns; /* receipt time */
    double x, y, yaw, vx, vy, ax, ay;
} xgc2_ugv_planar_pva;

/* ------------------------------------------------------------------------------------------------
 */
/* Reference (edge -> reference -> controller) */

#define XGC2_UGV_ANALYTIC_HOLD 0u
#define XGC2_UGV_ANALYTIC_CIRCLE 1u
#define XGC2_UGV_ANALYTIC_CIRCLE_ENTRY 3u
#define XGC2_UGV_ANALYTIC_FIGURE_EIGHT 4u

/* An analytic reference: event port as a request, state port as the active reference. */
typedef struct xgc2_ugv_analytic_reference {
    int64_t stamp_ns;      /* header stamp of the request */
    int64_t start_time_ns; /* 0: as soon as possible */
    double duration;
    double origin_x, origin_y, origin_z;
    double origin_qx, origin_qy, origin_qz, origin_qw;
    double
        params[XGC2_UGV_MAX_ANALYTIC_PARAMS]; /* radius, line speed, entry duration, centre x, y */
    uint32_t request_id, trajectory_id, revision, flags;
    uint32_t param_count;
    uint16_t analytic_type; /* XGC2_UGV_ANALYTIC_* */
    uint16_t reserved;
} xgc2_ugv_analytic_reference;

/* One sample of a sampled reference (the planar reference point message). */
typedef struct xgc2_ugv_planar_point {
    double t_from_start;
    double x, y, yaw;
    double speed, linear_acceleration, yaw_rate, yaw_acceleration, curvature;
    double vx, vy, ax, ay, jx, jy;
} xgc2_ugv_planar_point;

#define XGC2_UGV_SAMPLED_FLAG_EXPLICIT_PLANAR_KINEMATICS 32768u

/* A sampled reference: event port as a request, state port as the active reference. */
typedef struct xgc2_ugv_sampled_reference {
    int64_t stamp_ns;
    int64_t start_time_ns;
    double sample_dt;
    uint32_t trajectory_id, revision, flags;
    uint32_t point_count; /* used entries of points */
    xgc2_ugv_planar_point points[XGC2_UGV_MAX_SAMPLED_POINTS];
} xgc2_ugv_sampled_reference;

/* The active polynomial reference the waypoint planner produced (state port). */
typedef struct xgc2_ugv_polynomial_reference {
    int64_t stamp_ns;
    int64_t start_time_ns;
    double duration;
    uint32_t trajectory_id, revision, flags;
    uint32_t order; /* at most 7 */
    uint32_t segment_count;
    uint32_t coeff_x_count, coeff_y_count,
        coeff_yaw_count; /* segment_count * (order + 1), or 0 for yaw */
    uint32_t reserved, reserved2;
    double segment_durations[XGC2_UGV_MAX_POLYNOMIAL_SEGMENTS];
    double coeff_x[XGC2_UGV_MAX_POLYNOMIAL_SEGMENTS * XGC2_UGV_POLYNOMIAL_COEFFS];
    double coeff_y[XGC2_UGV_MAX_POLYNOMIAL_SEGMENTS * XGC2_UGV_POLYNOMIAL_COEFFS];
    double coeff_yaw[XGC2_UGV_MAX_POLYNOMIAL_SEGMENTS * XGC2_UGV_POLYNOMIAL_COEFFS];
} xgc2_ugv_polynomial_reference;

/* One waypoint: position and orientation quaternion. */
typedef struct xgc2_ugv_waypoint {
    double x, y, z;
    double qx, qy, qz, qw;
} xgc2_ugv_waypoint;

/* Event port. Fixed-time seventh-order interpolation through waypoints; the header stamp is the
 * requested start time (0: as soon as possible). */
typedef struct xgc2_ugv_waypoint_request {
    int64_t stamp_ns;
    uint32_t request_id, trajectory_id, revision, flags;
    uint32_t waypoint_count;
    uint32_t segment_time_count; /* 0, or waypoint_count - 1 */
    double start_velocity[3], start_acceleration[3], end_velocity[3], end_acceleration[3];
    double desired_speed, max_velocity, max_acceleration, max_yaw_rate, max_linear_acceleration;
    uint32_t objective;
    uint32_t reserved;
    xgc2_ugv_waypoint waypoints[XGC2_UGV_MAX_WAYPOINTS];
    double segment_times[XGC2_UGV_MAX_WAYPOINTS];
} xgc2_ugv_waypoint_request;

#define XGC2_UGV_REFERENCE_STATE_SELF_CHECK 1u
#define XGC2_UGV_REFERENCE_STATE_READY 2u
#define XGC2_UGV_REFERENCE_STATE_PLANNING 3u
#define XGC2_UGV_REFERENCE_STATE_ACTIVE 4u
#define XGC2_UGV_REFERENCE_STATE_FAULT 9u
#define XGC2_UGV_REFERENCE_TYPE_NONE 0u
#define XGC2_UGV_REFERENCE_TYPE_ANALYTIC 1u
#define XGC2_UGV_REFERENCE_TYPE_POLYNOMIAL 2u
#define XGC2_UGV_REFERENCE_TYPE_SAMPLED 3u

/* State port. The reference generator's status. */
typedef struct xgc2_ugv_reference_status {
    int64_t stamp_ns;
    uint32_t state; /* XGC2_UGV_REFERENCE_STATE_* */
    uint32_t flags;
    uint32_t active_trajectory_id;
    uint32_t active_revision;
    uint32_t active_type; /* XGC2_UGV_REFERENCE_TYPE_* */
    uint32_t reserved;
} xgc2_ugv_reference_status;

/* Event port. Restart the reference generator. */
typedef struct xgc2_ugv_reference_reset {
    int64_t stamp_ns; /* receipt time */
} xgc2_ugv_reference_reset;

#ifdef __cplusplus
} /* extern "C" */
#endif

/* ------------------------------------------------------------------------------------------------
 */
/* Size and alignment: the host binds ports by these numbers. Every payload is a multiple of 8
 * bytes. */

#ifdef __cplusplus
#define XGC2_UGV_ASSERT_LAYOUT(type, bytes)                                                  \
    static_assert(sizeof(type) == (bytes), #type " size changed: add a new schema version"); \
    static_assert(alignof(type) == 8u, #type " alignment changed")
#else
#define XGC2_UGV_ASSERT_LAYOUT(type, bytes)                                                   \
    _Static_assert(sizeof(type) == (bytes), #type " size changed: add a new schema version"); \
    _Static_assert(_Alignof(type) == 8u, #type " alignment changed")
#endif

XGC2_UGV_ASSERT_LAYOUT(xgc2_ugv_planar_state, 80u);
XGC2_UGV_ASSERT_LAYOUT(xgc2_ugv_command, 16u);
XGC2_UGV_ASSERT_LAYOUT(xgc2_ugv_reset_target, 24u);
XGC2_UGV_ASSERT_LAYOUT(xgc2_ugv_reset_session, 192u);
XGC2_UGV_ASSERT_LAYOUT(xgc2_ugv_reset_grant, 584u);
XGC2_UGV_ASSERT_LAYOUT(xgc2_ugv_reset_result, 144u);
XGC2_UGV_ASSERT_LAYOUT(xgc2_ugv_cmd_vel, 40u);
XGC2_UGV_ASSERT_LAYOUT(xgc2_ugv_controller_status, 40u);
XGC2_UGV_ASSERT_LAYOUT(xgc2_ugv_planar_pva, 64u);
XGC2_UGV_ASSERT_LAYOUT(xgc2_ugv_analytic_reference, 168u);
XGC2_UGV_ASSERT_LAYOUT(xgc2_ugv_planar_point, 120u);
XGC2_UGV_ASSERT_LAYOUT(xgc2_ugv_sampled_reference, 61480u);
XGC2_UGV_ASSERT_LAYOUT(xgc2_ugv_polynomial_reference, 12864u);
XGC2_UGV_ASSERT_LAYOUT(xgc2_ugv_waypoint, 56u);
XGC2_UGV_ASSERT_LAYOUT(xgc2_ugv_waypoint_request, 4272u);
XGC2_UGV_ASSERT_LAYOUT(xgc2_ugv_reference_status, 32u);
XGC2_UGV_ASSERT_LAYOUT(xgc2_ugv_reference_reset, 8u);

#endif /* XGC2_UGV_PAYLOADS_H */
