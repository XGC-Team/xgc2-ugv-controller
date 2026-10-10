#pragma once

// The reference payloads (xgc2_ugv/payloads.h) and the plain types of the reference generator
// (unicycle_reference_trajectory/reference_types.h), field for field. The generator module
// reads its requests and writes its active references with these; the ROS edge sits between the
// messages (unicycle_reference_trajectory/ros_reference_conversion.h) and the payloads.
//
// A payload holds no frame id and no header sequence number: they are not part of the
// reference, and a plain reference converted from a payload has an empty frame id and sequence 0.
// Times are nanoseconds in the host clock; a time the plain type holds as zero stays zero.

#include <algorithm>
#include <cstdint>

#include "unicycle_reference_trajectory/reference_types.h"
#include "xgc2_ugv/payloads.h"

namespace ugv_modules {
namespace reference_payloads {

namespace ref = ::unicycle_reference_trajectory::reference;
using ::unicycle_reference_trajectory::Time;

// The payload constants are the plain types' constants (and so the messages').
static_assert(XGC2_UGV_ANALYTIC_HOLD == ref::AnalyticReference::ANALYTIC_HOLD, "");
static_assert(XGC2_UGV_ANALYTIC_CIRCLE == ref::AnalyticReference::ANALYTIC_CIRCLE, "");
static_assert(XGC2_UGV_ANALYTIC_CIRCLE_ENTRY == ref::AnalyticReference::ANALYTIC_CIRCLE_ENTRY, "");
static_assert(XGC2_UGV_ANALYTIC_FIGURE_EIGHT == ref::AnalyticReference::ANALYTIC_FIGURE_EIGHT, "");
static_assert(XGC2_UGV_SAMPLED_FLAG_EXPLICIT_PLANAR_KINEMATICS ==
                  ref::SampledReference::FLAG_EXPLICIT_PLANAR_KINEMATICS,
              "");
static_assert(XGC2_UGV_REFERENCE_STATE_SELF_CHECK == ref::ReferenceStatus::STATE_SELF_CHECK, "");
static_assert(XGC2_UGV_REFERENCE_STATE_READY == ref::ReferenceStatus::STATE_READY, "");
static_assert(XGC2_UGV_REFERENCE_STATE_PLANNING == ref::ReferenceStatus::STATE_PLANNING, "");
static_assert(XGC2_UGV_REFERENCE_STATE_ACTIVE == ref::ReferenceStatus::STATE_ACTIVE, "");
static_assert(XGC2_UGV_REFERENCE_STATE_FAULT == ref::ReferenceStatus::STATE_FAULT, "");
static_assert(XGC2_UGV_REFERENCE_TYPE_NONE == ref::ReferenceStatus::TYPE_NONE, "");
static_assert(XGC2_UGV_REFERENCE_TYPE_ANALYTIC == ref::ReferenceStatus::TYPE_ANALYTIC, "");
static_assert(XGC2_UGV_REFERENCE_TYPE_POLYNOMIAL == ref::ReferenceStatus::TYPE_POLYNOMIAL, "");
static_assert(XGC2_UGV_REFERENCE_TYPE_SAMPLED == ref::ReferenceStatus::TYPE_SAMPLED, "");

inline Time timeOfNs(int64_t ns) {
    Time time;
    time.fromNSec(ns > 0 ? static_cast<uint64_t>(ns) : 0U);
    return time;
}

inline int64_t nsOfTime(const Time& time) {
    return static_cast<int64_t>(time.toNSec());
}

constexpr size_t kPolynomialCapacity =
    XGC2_UGV_MAX_POLYNOMIAL_SEGMENTS * XGC2_UGV_POLYNOMIAL_COEFFS;

// ---- payload -> plain: false when a count exceeds the payload's capacity
// ---------------------------

inline bool toPlain(const xgc2_ugv_analytic_reference& p, ref::AnalyticReference& m) {
    if (p.param_count > XGC2_UGV_MAX_ANALYTIC_PARAMS) {
        return false;
    }
    m = ref::AnalyticReference{};
    m.header.stamp = timeOfNs(p.stamp_ns);
    m.request_id = p.request_id;
    m.trajectory_id = p.trajectory_id;
    m.revision = p.revision;
    m.analytic_type = p.analytic_type;
    m.flags = p.flags;
    m.start_time = timeOfNs(p.start_time_ns);
    m.duration = p.duration;
    m.origin.position = {p.origin_x, p.origin_y, p.origin_z};
    m.origin.orientation = {p.origin_qx, p.origin_qy, p.origin_qz, p.origin_qw};
    m.params.assign(p.params, p.params + p.param_count);
    return true;
}

inline bool toPlain(const xgc2_ugv_sampled_reference& p, ref::SampledReference& m) {
    if (p.point_count > XGC2_UGV_MAX_SAMPLED_POINTS) {
        return false;
    }
    m = ref::SampledReference{};
    m.header.stamp = timeOfNs(p.stamp_ns);
    m.trajectory_id = p.trajectory_id;
    m.revision = p.revision;
    m.flags = p.flags;
    m.start_time = timeOfNs(p.start_time_ns);
    m.sample_dt = p.sample_dt;
    m.points.reserve(p.point_count);
    for (uint32_t i = 0; i < p.point_count; ++i) {
        const xgc2_ugv_planar_point& q = p.points[i];
        m.points.push_back({q.t_from_start, q.x, q.y, q.yaw, q.speed, q.linear_acceleration,
                            q.yaw_rate, q.yaw_acceleration, q.curvature, q.vx, q.vy, q.ax, q.ay,
                            q.jx, q.jy});
    }
    return true;
}

inline bool toPlain(const xgc2_ugv_waypoint_request& p, ref::WaypointReferenceRequest& m) {
    if (p.waypoint_count > XGC2_UGV_MAX_WAYPOINTS ||
        p.segment_time_count > XGC2_UGV_MAX_WAYPOINTS) {
        return false;
    }
    m = ref::WaypointReferenceRequest{};
    m.header.stamp = timeOfNs(p.stamp_ns);
    m.request_id = p.request_id;
    m.trajectory_id = p.trajectory_id;
    m.revision = p.revision;
    m.flags = p.flags;
    m.waypoints.reserve(p.waypoint_count);
    for (uint32_t i = 0; i < p.waypoint_count; ++i) {
        const xgc2_ugv_waypoint& w = p.waypoints[i];
        m.waypoints.push_back({{w.x, w.y, w.z}, {w.qx, w.qy, w.qz, w.qw}});
    }
    m.segment_times.assign(p.segment_times, p.segment_times + p.segment_time_count);
    m.start_velocity = {p.start_velocity[0], p.start_velocity[1], p.start_velocity[2]};
    m.start_acceleration = {p.start_acceleration[0], p.start_acceleration[1],
                            p.start_acceleration[2]};
    m.end_velocity = {p.end_velocity[0], p.end_velocity[1], p.end_velocity[2]};
    m.end_acceleration = {p.end_acceleration[0], p.end_acceleration[1], p.end_acceleration[2]};
    m.desired_speed = p.desired_speed;
    m.max_velocity = p.max_velocity;
    m.max_acceleration = p.max_acceleration;
    m.max_yaw_rate = p.max_yaw_rate;
    m.max_linear_acceleration = p.max_linear_acceleration;
    m.objective = static_cast<uint8_t>(p.objective);
    return true;
}

inline bool toPlain(const xgc2_ugv_polynomial_reference& p, ref::ActivePolynomialReference& m) {
    if (p.segment_count > XGC2_UGV_MAX_POLYNOMIAL_SEGMENTS ||
        p.coeff_x_count > kPolynomialCapacity || p.coeff_y_count > kPolynomialCapacity ||
        p.coeff_yaw_count > kPolynomialCapacity) {
        return false;
    }
    m = ref::ActivePolynomialReference{};
    m.header.stamp = timeOfNs(p.stamp_ns);
    m.trajectory_id = p.trajectory_id;
    m.revision = p.revision;
    m.flags = p.flags;
    m.start_time = timeOfNs(p.start_time_ns);
    m.duration = p.duration;
    m.order = static_cast<uint8_t>(p.order);
    m.segment_durations.assign(p.segment_durations, p.segment_durations + p.segment_count);
    m.coeff_x.assign(p.coeff_x, p.coeff_x + p.coeff_x_count);
    m.coeff_y.assign(p.coeff_y, p.coeff_y + p.coeff_y_count);
    m.coeff_yaw.assign(p.coeff_yaw, p.coeff_yaw + p.coeff_yaw_count);
    return true;
}

inline ref::ReferenceStatus toPlain(const xgc2_ugv_reference_status& p) {
    ref::ReferenceStatus m;
    m.header.stamp = timeOfNs(p.stamp_ns);
    m.state = static_cast<uint8_t>(p.state);
    m.flags = p.flags;
    m.active_trajectory_id = p.active_trajectory_id;
    m.active_revision = p.active_revision;
    m.active_type = static_cast<uint8_t>(p.active_type);
    return m;
}

// ---- plain -> payload: fits() first; the payload is zeroed by the caller
// ----------------------------

inline bool fits(const ref::AnalyticReference& m) {
    return m.params.size() <= XGC2_UGV_MAX_ANALYTIC_PARAMS;
}
inline bool fits(const ref::SampledReference& m) {
    return m.points.size() <= XGC2_UGV_MAX_SAMPLED_POINTS;
}
inline bool fits(const ref::WaypointReferenceRequest& m) {
    return m.waypoints.size() <= XGC2_UGV_MAX_WAYPOINTS &&
           m.segment_times.size() <= XGC2_UGV_MAX_WAYPOINTS;
}
inline bool fits(const ref::ActivePolynomialReference& m) {
    return m.segment_durations.size() <= XGC2_UGV_MAX_POLYNOMIAL_SEGMENTS &&
           m.coeff_x.size() <= kPolynomialCapacity && m.coeff_y.size() <= kPolynomialCapacity &&
           m.coeff_yaw.size() <= kPolynomialCapacity;
}

inline void toPayload(xgc2_ugv_analytic_reference& p, const ref::AnalyticReference& m) {
    p.stamp_ns = nsOfTime(m.header.stamp);
    p.start_time_ns = nsOfTime(m.start_time);
    p.duration = m.duration;
    p.origin_x = m.origin.position.x;
    p.origin_y = m.origin.position.y;
    p.origin_z = m.origin.position.z;
    p.origin_qx = m.origin.orientation.x;
    p.origin_qy = m.origin.orientation.y;
    p.origin_qz = m.origin.orientation.z;
    p.origin_qw = m.origin.orientation.w;
    std::copy(m.params.begin(), m.params.end(), p.params);
    p.request_id = m.request_id;
    p.trajectory_id = m.trajectory_id;
    p.revision = m.revision;
    p.flags = m.flags;
    p.param_count = static_cast<uint32_t>(m.params.size());
    p.analytic_type = m.analytic_type;
}

inline void toPayload(xgc2_ugv_sampled_reference& p, const ref::SampledReference& m) {
    p.stamp_ns = nsOfTime(m.header.stamp);
    p.start_time_ns = nsOfTime(m.start_time);
    p.sample_dt = m.sample_dt;
    p.trajectory_id = m.trajectory_id;
    p.revision = m.revision;
    p.flags = m.flags;
    p.point_count = static_cast<uint32_t>(m.points.size());
    for (size_t i = 0; i < m.points.size(); ++i) {
        const ref::PlanarReferencePoint& q = m.points[i];
        xgc2_ugv_planar_point& out = p.points[i];
        out.t_from_start = q.t_from_start;
        out.x = q.x;
        out.y = q.y;
        out.yaw = q.yaw;
        out.speed = q.speed;
        out.linear_acceleration = q.linear_acceleration;
        out.yaw_rate = q.yaw_rate;
        out.yaw_acceleration = q.yaw_acceleration;
        out.curvature = q.curvature;
        out.vx = q.vx;
        out.vy = q.vy;
        out.ax = q.ax;
        out.ay = q.ay;
        out.jx = q.jx;
        out.jy = q.jy;
    }
}

inline void toPayload(xgc2_ugv_waypoint_request& p, const ref::WaypointReferenceRequest& m) {
    p.stamp_ns = nsOfTime(m.header.stamp);
    p.request_id = m.request_id;
    p.trajectory_id = m.trajectory_id;
    p.revision = m.revision;
    p.flags = m.flags;
    p.waypoint_count = static_cast<uint32_t>(m.waypoints.size());
    p.segment_time_count = static_cast<uint32_t>(m.segment_times.size());
    p.start_velocity[0] = m.start_velocity.x;
    p.start_velocity[1] = m.start_velocity.y;
    p.start_velocity[2] = m.start_velocity.z;
    p.start_acceleration[0] = m.start_acceleration.x;
    p.start_acceleration[1] = m.start_acceleration.y;
    p.start_acceleration[2] = m.start_acceleration.z;
    p.end_velocity[0] = m.end_velocity.x;
    p.end_velocity[1] = m.end_velocity.y;
    p.end_velocity[2] = m.end_velocity.z;
    p.end_acceleration[0] = m.end_acceleration.x;
    p.end_acceleration[1] = m.end_acceleration.y;
    p.end_acceleration[2] = m.end_acceleration.z;
    p.desired_speed = m.desired_speed;
    p.max_velocity = m.max_velocity;
    p.max_acceleration = m.max_acceleration;
    p.max_yaw_rate = m.max_yaw_rate;
    p.max_linear_acceleration = m.max_linear_acceleration;
    p.objective = m.objective;
    for (size_t i = 0; i < m.waypoints.size(); ++i) {
        const ref::Pose& w = m.waypoints[i];
        p.waypoints[i] = {w.position.x,    w.position.y,    w.position.z,   w.orientation.x,
                          w.orientation.y, w.orientation.z, w.orientation.w};
    }
    std::copy(m.segment_times.begin(), m.segment_times.end(), p.segment_times);
}

inline void toPayload(xgc2_ugv_polynomial_reference& p, const ref::ActivePolynomialReference& m) {
    p.stamp_ns = nsOfTime(m.header.stamp);
    p.start_time_ns = nsOfTime(m.start_time);
    p.duration = m.duration;
    p.trajectory_id = m.trajectory_id;
    p.revision = m.revision;
    p.flags = m.flags;
    p.order = m.order;
    p.segment_count = static_cast<uint32_t>(m.segment_durations.size());
    p.coeff_x_count = static_cast<uint32_t>(m.coeff_x.size());
    p.coeff_y_count = static_cast<uint32_t>(m.coeff_y.size());
    p.coeff_yaw_count = static_cast<uint32_t>(m.coeff_yaw.size());
    std::copy(m.segment_durations.begin(), m.segment_durations.end(), p.segment_durations);
    std::copy(m.coeff_x.begin(), m.coeff_x.end(), p.coeff_x);
    std::copy(m.coeff_y.begin(), m.coeff_y.end(), p.coeff_y);
    std::copy(m.coeff_yaw.begin(), m.coeff_yaw.end(), p.coeff_yaw);
}

inline void toPayload(xgc2_ugv_reference_status& p, const ref::ReferenceStatus& m) {
    p.stamp_ns = nsOfTime(m.header.stamp);
    p.state = m.state;
    p.flags = m.flags;
    p.active_trajectory_id = m.active_trajectory_id;
    p.active_revision = m.active_revision;
    p.active_type = m.active_type;
}

}  // namespace reference_payloads
}  // namespace ugv_modules
