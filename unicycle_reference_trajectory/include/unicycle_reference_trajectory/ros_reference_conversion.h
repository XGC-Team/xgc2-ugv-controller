#pragma once

// The ROS edge of the reference trajectory runtime: messages <-> the plain
// types in reference_types.h, field for field and bit-exact (times keep
// their sec/nsec). Only the node, its tests and the ROS replay checks include
// this; the runtime never does.

#include <unicycle_reference_trajectory_msgs/ActivePolynomialReference.h>
#include <unicycle_reference_trajectory_msgs/AnalyticReference.h>
#include <unicycle_reference_trajectory_msgs/ReferenceStatus.h>
#include <unicycle_reference_trajectory_msgs/SampledReference.h>
#include <unicycle_reference_trajectory_msgs/WaypointReferenceRequest.h>

#include "unicycle_reference_trajectory/reference_types.h"

namespace unicycle_reference_trajectory {

namespace msgs = ::unicycle_reference_trajectory_msgs;

// The plain constants must stay the message constants.
static_assert(reference::AnalyticReference::ANALYTIC_HOLD == msgs::AnalyticReference::ANALYTIC_HOLD,
              "");
static_assert(reference::AnalyticReference::ANALYTIC_CIRCLE ==
                  msgs::AnalyticReference::ANALYTIC_CIRCLE,
              "");
static_assert(reference::AnalyticReference::ANALYTIC_CIRCLE_ENTRY ==
                  msgs::AnalyticReference::ANALYTIC_CIRCLE_ENTRY,
              "");
static_assert(reference::AnalyticReference::ANALYTIC_FIGURE_EIGHT ==
                  msgs::AnalyticReference::ANALYTIC_FIGURE_EIGHT,
              "");
static_assert(reference::SampledReference::FLAG_EXPLICIT_PLANAR_KINEMATICS ==
                  msgs::SampledReference::FLAG_EXPLICIT_PLANAR_KINEMATICS,
              "");
static_assert(reference::WaypointReferenceRequest::OBJECTIVE_SEPTIC_INTERPOLATION ==
                  msgs::WaypointReferenceRequest::OBJECTIVE_SEPTIC_INTERPOLATION,
              "");
static_assert(reference::ReferenceStatus::STATE_SELF_CHECK ==
                  msgs::ReferenceStatus::STATE_SELF_CHECK,
              "");
static_assert(reference::ReferenceStatus::STATE_READY == msgs::ReferenceStatus::STATE_READY, "");
static_assert(reference::ReferenceStatus::STATE_PLANNING == msgs::ReferenceStatus::STATE_PLANNING,
              "");
static_assert(reference::ReferenceStatus::STATE_ACTIVE == msgs::ReferenceStatus::STATE_ACTIVE, "");
static_assert(reference::ReferenceStatus::STATE_FAULT == msgs::ReferenceStatus::STATE_FAULT, "");
static_assert(reference::ReferenceStatus::TYPE_NONE == msgs::ReferenceStatus::TYPE_NONE, "");
static_assert(reference::ReferenceStatus::TYPE_ANALYTIC == msgs::ReferenceStatus::TYPE_ANALYTIC,
              "");
static_assert(reference::ReferenceStatus::TYPE_POLYNOMIAL == msgs::ReferenceStatus::TYPE_POLYNOMIAL,
              "");
static_assert(reference::ReferenceStatus::TYPE_SAMPLED == msgs::ReferenceStatus::TYPE_SAMPLED, "");

inline Time toCore(const ros::Time& t) {
    return Time(t.sec, t.nsec);
}
inline ros::Time toRos(const Time& t) {
    return ros::Time(t.sec, t.nsec);
}

inline reference::Header toCore(const std_msgs::Header& h) {
    return {h.seq, toCore(h.stamp), h.frame_id};
}
inline std_msgs::Header toRos(const reference::Header& h) {
    std_msgs::Header out;
    out.seq = h.seq;
    out.stamp = toRos(h.stamp);
    out.frame_id = h.frame_id;
    return out;
}
inline reference::Point toCore(const geometry_msgs::Point& p) {
    return {p.x, p.y, p.z};
}
inline reference::Vector3 toCore(const geometry_msgs::Vector3& v) {
    return {v.x, v.y, v.z};
}
inline reference::Pose toCore(const geometry_msgs::Pose& p) {
    return {toCore(p.position),
            {p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w}};
}
inline geometry_msgs::Point toRos(const reference::Point& p) {
    geometry_msgs::Point out;
    out.x = p.x;
    out.y = p.y;
    out.z = p.z;
    return out;
}
inline geometry_msgs::Vector3 toRos(const reference::Vector3& v) {
    geometry_msgs::Vector3 out;
    out.x = v.x;
    out.y = v.y;
    out.z = v.z;
    return out;
}
inline geometry_msgs::Pose toRos(const reference::Pose& p) {
    geometry_msgs::Pose out;
    out.position = toRos(p.position);
    out.orientation.x = p.orientation.x;
    out.orientation.y = p.orientation.y;
    out.orientation.z = p.orientation.z;
    out.orientation.w = p.orientation.w;
    return out;
}

inline reference::AnalyticReference toCore(const msgs::AnalyticReference& m) {
    reference::AnalyticReference out;
    out.header = toCore(m.header);
    out.request_id = m.request_id;
    out.trajectory_id = m.trajectory_id;
    out.revision = m.revision;
    out.analytic_type = m.analytic_type;
    out.flags = m.flags;
    out.start_time = toCore(m.start_time);
    out.duration = m.duration;
    out.origin = toCore(m.origin);
    out.params = m.params;
    return out;
}
inline msgs::AnalyticReference toRos(const reference::AnalyticReference& m) {
    msgs::AnalyticReference out;
    out.header = toRos(m.header);
    out.request_id = m.request_id;
    out.trajectory_id = m.trajectory_id;
    out.revision = m.revision;
    out.analytic_type = m.analytic_type;
    out.flags = m.flags;
    out.start_time = toRos(m.start_time);
    out.duration = m.duration;
    out.origin = toRos(m.origin);
    out.params = m.params;
    return out;
}

inline reference::SampledReference toCore(const msgs::SampledReference& m) {
    reference::SampledReference out;
    out.header = toCore(m.header);
    out.trajectory_id = m.trajectory_id;
    out.revision = m.revision;
    out.flags = m.flags;
    out.start_time = toCore(m.start_time);
    out.sample_dt = m.sample_dt;
    out.points.reserve(m.points.size());
    for (const auto& p : m.points) {
        out.points.push_back({p.t_from_start, p.x, p.y, p.yaw, p.speed, p.linear_acceleration,
                              p.yaw_rate, p.yaw_acceleration, p.curvature, p.vx, p.vy, p.ax, p.ay,
                              p.jx, p.jy});
    }
    return out;
}
inline msgs::SampledReference toRos(const reference::SampledReference& m) {
    msgs::SampledReference out;
    out.header = toRos(m.header);
    out.trajectory_id = m.trajectory_id;
    out.revision = m.revision;
    out.flags = m.flags;
    out.start_time = toRos(m.start_time);
    out.sample_dt = m.sample_dt;
    out.points.reserve(m.points.size());
    for (const auto& p : m.points) {
        msgs::PlanarReferencePoint q;
        q.t_from_start = p.t_from_start;
        q.x = p.x;
        q.y = p.y;
        q.yaw = p.yaw;
        q.speed = p.speed;
        q.linear_acceleration = p.linear_acceleration;
        q.yaw_rate = p.yaw_rate;
        q.yaw_acceleration = p.yaw_acceleration;
        q.curvature = p.curvature;
        q.vx = p.vx;
        q.vy = p.vy;
        q.ax = p.ax;
        q.ay = p.ay;
        q.jx = p.jx;
        q.jy = p.jy;
        out.points.push_back(q);
    }
    return out;
}

inline reference::WaypointReferenceRequest toCore(const msgs::WaypointReferenceRequest& m) {
    reference::WaypointReferenceRequest out;
    out.header = toCore(m.header);
    out.request_id = m.request_id;
    out.trajectory_id = m.trajectory_id;
    out.revision = m.revision;
    out.flags = m.flags;
    out.waypoints.reserve(m.waypoints.size());
    for (const auto& pose : m.waypoints) {
        out.waypoints.push_back(toCore(pose));
    }
    out.segment_times = m.segment_times;
    out.start_velocity = toCore(m.start_velocity);
    out.start_acceleration = toCore(m.start_acceleration);
    out.end_velocity = toCore(m.end_velocity);
    out.end_acceleration = toCore(m.end_acceleration);
    out.desired_speed = m.desired_speed;
    out.max_velocity = m.max_velocity;
    out.max_acceleration = m.max_acceleration;
    out.max_yaw_rate = m.max_yaw_rate;
    out.max_linear_acceleration = m.max_linear_acceleration;
    out.objective = m.objective;
    return out;
}
inline msgs::WaypointReferenceRequest toRos(const reference::WaypointReferenceRequest& m) {
    msgs::WaypointReferenceRequest out;
    out.header = toRos(m.header);
    out.request_id = m.request_id;
    out.trajectory_id = m.trajectory_id;
    out.revision = m.revision;
    out.flags = m.flags;
    out.waypoints.reserve(m.waypoints.size());
    for (const auto& pose : m.waypoints) {
        out.waypoints.push_back(toRos(pose));
    }
    out.segment_times = m.segment_times;
    out.start_velocity = toRos(m.start_velocity);
    out.start_acceleration = toRos(m.start_acceleration);
    out.end_velocity = toRos(m.end_velocity);
    out.end_acceleration = toRos(m.end_acceleration);
    out.desired_speed = m.desired_speed;
    out.max_velocity = m.max_velocity;
    out.max_acceleration = m.max_acceleration;
    out.max_yaw_rate = m.max_yaw_rate;
    out.max_linear_acceleration = m.max_linear_acceleration;
    out.objective = m.objective;
    return out;
}

inline msgs::ActivePolynomialReference toRos(const reference::ActivePolynomialReference& m) {
    msgs::ActivePolynomialReference out;
    out.header = toRos(m.header);
    out.trajectory_id = m.trajectory_id;
    out.revision = m.revision;
    out.flags = m.flags;
    out.start_time = toRos(m.start_time);
    out.duration = m.duration;
    out.order = m.order;
    out.segment_durations = m.segment_durations;
    out.coeff_x = m.coeff_x;
    out.coeff_y = m.coeff_y;
    out.coeff_yaw = m.coeff_yaw;
    return out;
}

inline msgs::ReferenceStatus toRos(const reference::ReferenceStatus& m) {
    msgs::ReferenceStatus out;
    out.header = toRos(m.header);
    out.state = m.state;
    out.flags = m.flags;
    out.active_trajectory_id = m.active_trajectory_id;
    out.active_revision = m.active_revision;
    out.active_type = m.active_type;
    return out;
}

}  // namespace unicycle_reference_trajectory
