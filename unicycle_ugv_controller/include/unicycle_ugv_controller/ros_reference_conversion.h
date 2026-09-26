#pragma once

#include <unicycle_reference_trajectory_msgs/ActivePolynomialReference.h>
#include <unicycle_reference_trajectory_msgs/AnalyticReference.h>
#include <unicycle_reference_trajectory_msgs/SampledReference.h>

#include "unicycle_ugv_controller/common/reference_types.h"
#include "unicycle_ugv_controller/ros_time_conversion.h"

namespace unicycle_ugv_controller {

// ROS edge only: the reference messages as the core's plain references.
// Every field the core reads is copied unchanged.

inline reference::AnalyticReference toCoreReference(
    const unicycle_reference_trajectory_msgs::AnalyticReference& m) {
    reference::AnalyticReference r;
    r.request_id = m.request_id;
    r.trajectory_id = m.trajectory_id;
    r.revision = m.revision;
    r.analytic_type = m.analytic_type;
    r.flags = m.flags;
    r.start_time = toCoreTime(m.start_time);
    r.duration = m.duration;
    r.origin.position.x = m.origin.position.x;
    r.origin.position.y = m.origin.position.y;
    r.origin.position.z = m.origin.position.z;
    r.origin.orientation.x = m.origin.orientation.x;
    r.origin.orientation.y = m.origin.orientation.y;
    r.origin.orientation.z = m.origin.orientation.z;
    r.origin.orientation.w = m.origin.orientation.w;
    r.params.assign(m.params.begin(), m.params.end());
    return r;
}

inline reference::SampledReference toCoreReference(
    const unicycle_reference_trajectory_msgs::SampledReference& m) {
    reference::SampledReference r;
    r.trajectory_id = m.trajectory_id;
    r.revision = m.revision;
    r.flags = m.flags;
    r.start_time = toCoreTime(m.start_time);
    r.sample_dt = m.sample_dt;
    r.points.reserve(m.points.size());
    for (const auto& p : m.points) {
        reference::PlanarReferencePoint q;
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
        r.points.push_back(q);
    }
    return r;
}

inline reference::ActivePolynomialReference toCoreReference(
    const unicycle_reference_trajectory_msgs::ActivePolynomialReference& m) {
    reference::ActivePolynomialReference r;
    r.trajectory_id = m.trajectory_id;
    r.revision = m.revision;
    r.flags = m.flags;
    r.start_time = toCoreTime(m.start_time);
    r.duration = m.duration;
    r.order = m.order;
    r.segment_durations.assign(m.segment_durations.begin(), m.segment_durations.end());
    r.coeff_x.assign(m.coeff_x.begin(), m.coeff_x.end());
    r.coeff_y.assign(m.coeff_y.begin(), m.coeff_y.end());
    r.coeff_yaw.assign(m.coeff_yaw.begin(), m.coeff_yaw.end());
    return r;
}

}  // namespace unicycle_ugv_controller
