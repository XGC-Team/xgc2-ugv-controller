// The payloads of the vehicle modules: one layout for C and C++, and the plain types of the cores
// converted to payloads and back without loss, up to the capacities of the payloads.

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <cstring>
#include <memory>
#include <vector>

#include "reference_payloads.h"
#include "xgc2_ugv/payloads.h"

extern "C" size_t ugv_payload_layout_in_c(size_t* out, size_t capacity);

namespace {

namespace rp = ugv_modules::reference_payloads;
namespace ref = unicycle_reference_trajectory::reference;

TEST(Payloads, CAndCppSeeTheSameLayout) {
    size_t c[64];
    const size_t count = ugv_payload_layout_in_c(c, 64);
    ASSERT_EQ(count, 48u);
    const size_t cpp[] = {
        sizeof(xgc2_ugv_planar_state),
        alignof(xgc2_ugv_planar_state),
        offsetof(xgc2_ugv_planar_state, reserved),
        sizeof(xgc2_ugv_command),
        alignof(xgc2_ugv_command),
        offsetof(xgc2_ugv_command, source),
        sizeof(xgc2_ugv_reset_target),
        alignof(xgc2_ugv_reset_target),
        offsetof(xgc2_ugv_reset_target, yaw),
        sizeof(xgc2_ugv_reset_clearance),
        alignof(xgc2_ugv_reset_clearance),
        offsetof(xgc2_ugv_reset_clearance, status),
        sizeof(xgc2_ugv_reset_session),
        alignof(xgc2_ugv_reset_session),
        offsetof(xgc2_ugv_reset_session, flags),
        sizeof(xgc2_ugv_cmd_vel),
        alignof(xgc2_ugv_cmd_vel),
        offsetof(xgc2_ugv_cmd_vel, reserved),
        sizeof(xgc2_ugv_controller_status),
        alignof(xgc2_ugv_controller_status),
        offsetof(xgc2_ugv_controller_status, control_state_name),
        sizeof(xgc2_ugv_planar_pva),
        alignof(xgc2_ugv_planar_pva),
        offsetof(xgc2_ugv_planar_pva, ay),
        sizeof(xgc2_ugv_analytic_reference),
        alignof(xgc2_ugv_analytic_reference),
        offsetof(xgc2_ugv_analytic_reference, reserved),
        sizeof(xgc2_ugv_planar_point),
        alignof(xgc2_ugv_planar_point),
        offsetof(xgc2_ugv_planar_point, jy),
        sizeof(xgc2_ugv_sampled_reference),
        alignof(xgc2_ugv_sampled_reference),
        offsetof(xgc2_ugv_sampled_reference, points),
        sizeof(xgc2_ugv_polynomial_reference),
        alignof(xgc2_ugv_polynomial_reference),
        offsetof(xgc2_ugv_polynomial_reference, coeff_yaw),
        sizeof(xgc2_ugv_waypoint),
        alignof(xgc2_ugv_waypoint),
        offsetof(xgc2_ugv_waypoint, qw),
        sizeof(xgc2_ugv_waypoint_request),
        alignof(xgc2_ugv_waypoint_request),
        offsetof(xgc2_ugv_waypoint_request, segment_times),
        sizeof(xgc2_ugv_reference_status),
        alignof(xgc2_ugv_reference_status),
        offsetof(xgc2_ugv_reference_status, reserved),
        sizeof(xgc2_ugv_reference_reset),
        alignof(xgc2_ugv_reference_reset),
        offsetof(xgc2_ugv_reference_reset, stamp_ns),
    };
    ASSERT_EQ(sizeof cpp / sizeof cpp[0], count);
    for (size_t i = 0; i < count; ++i) {
        EXPECT_EQ(c[i], cpp[i]) << i;
    }
}

TEST(Payloads, EveryPayloadIsAMultipleOfItsAlignmentAndPlainData) {
    // the static asserts of the header fix the numbers; the schema ids are unique
    const char* schemas[] = {
        XGC2_UGV_SCHEMA_PLANAR_STATE,         XGC2_UGV_SCHEMA_COMMAND,
        XGC2_UGV_SCHEMA_RESET_TARGET,         XGC2_UGV_SCHEMA_RESET_CLEARANCE,
        XGC2_UGV_SCHEMA_RESET_SESSION,        XGC2_UGV_SCHEMA_CMD_VEL,
        XGC2_UGV_SCHEMA_CONTROLLER_STATUS,    XGC2_UGV_SCHEMA_PLANAR_PVA,
        XGC2_UGV_SCHEMA_ANALYTIC_REFERENCE,   XGC2_UGV_SCHEMA_SAMPLED_REFERENCE,
        XGC2_UGV_SCHEMA_POLYNOMIAL_REFERENCE, XGC2_UGV_SCHEMA_WAYPOINT_REQUEST,
        XGC2_UGV_SCHEMA_REFERENCE_STATUS,     XGC2_UGV_SCHEMA_REFERENCE_RESET};
    for (size_t i = 0; i < sizeof schemas / sizeof schemas[0]; ++i) {
        EXPECT_EQ(std::strncmp(schemas[i], "xgc2.ugv.", 9), 0) << schemas[i];
        EXPECT_NE(std::strstr(schemas[i], ".v1"), nullptr) << schemas[i];
        for (size_t j = i + 1; j < sizeof schemas / sizeof schemas[0]; ++j) {
            EXPECT_STRNE(schemas[i], schemas[j]);
        }
    }
    EXPECT_EQ(sizeof(xgc2_ugv_sampled_reference) % 8, 0u);
    EXPECT_EQ(sizeof(xgc2_ugv_polynomial_reference) % 8, 0u);
    EXPECT_EQ(sizeof(xgc2_ugv_waypoint_request) % 8, 0u);
}

TEST(Payloads, TimesKeepTheirNanoseconds) {
    for (int64_t ns : {int64_t{0}, int64_t{1}, int64_t{999999999}, int64_t{1000000000},
                       int64_t{150737250000001}, int64_t{1791678765693717999}}) {
        EXPECT_EQ(rp::nsOfTime(rp::timeOfNs(ns)), ns);
    }
    EXPECT_TRUE(rp::timeOfNs(-5).isZero());  // a time before the epoch is no time
}

ref::AnalyticReference analytic() {
    ref::AnalyticReference m;
    m.header.stamp = rp::timeOfNs(123456789012LL);
    m.request_id = 7;
    m.trajectory_id = 8;
    m.revision = 9;
    m.analytic_type = 3;
    m.flags = 0x8001;
    m.start_time = rp::timeOfNs(124000000001LL);
    m.duration = 42.5;
    m.origin.position = {1.5, -2.5, 0.25};
    m.origin.orientation = {0.1, 0.2, 0.3, 0.9};
    m.params = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0};
    return m;
}

TEST(Payloads, AnAnalyticReferenceRoundTrips) {
    const ref::AnalyticReference m = analytic();
    ASSERT_TRUE(rp::fits(m));
    xgc2_ugv_analytic_reference p;
    std::memset(&p, 0, sizeof p);
    rp::toPayload(p, m);
    ref::AnalyticReference back;
    ASSERT_TRUE(rp::toPlain(p, back));
    EXPECT_EQ(back.header.stamp, m.header.stamp);
    EXPECT_EQ(back.request_id, m.request_id);
    EXPECT_EQ(back.trajectory_id, m.trajectory_id);
    EXPECT_EQ(back.revision, m.revision);
    EXPECT_EQ(back.analytic_type, m.analytic_type);
    EXPECT_EQ(back.flags, m.flags);
    EXPECT_EQ(back.start_time, m.start_time);
    EXPECT_EQ(back.duration, m.duration);
    EXPECT_EQ(back.origin.position.x, m.origin.position.x);
    EXPECT_EQ(back.origin.position.z, m.origin.position.z);
    EXPECT_EQ(back.origin.orientation.w, m.origin.orientation.w);
    EXPECT_EQ(back.params, m.params);
    // a ninth parameter does not fit, and a payload that claims one is refused, not truncated
    ref::AnalyticReference many = m;
    many.params.push_back(9.0);
    EXPECT_FALSE(rp::fits(many));
    p.param_count = XGC2_UGV_MAX_ANALYTIC_PARAMS + 1;
    EXPECT_FALSE(rp::toPlain(p, back));
}

TEST(Payloads, ASampledReferenceRoundTripsUpToItsCapacity) {
    ref::SampledReference m;
    m.header.stamp = rp::timeOfNs(5000000000LL);
    m.trajectory_id = 1;
    m.revision = 2;
    m.flags = 32768;
    m.start_time = rp::timeOfNs(6000000000LL);
    m.sample_dt = 0.05;
    for (uint32_t k = 0; k < XGC2_UGV_MAX_SAMPLED_POINTS; ++k) {
        ref::PlanarReferencePoint p;
        p.t_from_start = 0.05 * k;
        p.x = k;
        p.y = -static_cast<double>(k);
        p.yaw = 0.001 * k;
        p.speed = 0.5;
        p.linear_acceleration = 0.01 * k;
        p.yaw_rate = 0.02;
        p.yaw_acceleration = 0.03;
        p.curvature = 0.04;
        p.vx = 1;
        p.vy = 2;
        p.ax = 3;
        p.ay = 4;
        p.jx = 5;
        p.jy = 6;
        m.points.push_back(p);
    }
    ASSERT_TRUE(rp::fits(m));
    auto payload = std::make_unique<xgc2_ugv_sampled_reference>();
    std::memset(payload.get(), 0, sizeof *payload);
    rp::toPayload(*payload, m);
    ref::SampledReference back;
    ASSERT_TRUE(rp::toPlain(*payload, back));
    ASSERT_EQ(back.points.size(), m.points.size());
    for (size_t k = 0; k < m.points.size(); ++k) {
        EXPECT_EQ(std::memcmp(&back.points[k], &m.points[k], sizeof(ref::PlanarReferencePoint)), 0)
            << k;
    }
    EXPECT_EQ(back.sample_dt, m.sample_dt);
    EXPECT_EQ(back.start_time, m.start_time);
    m.points.push_back(m.points.back());
    EXPECT_FALSE(rp::fits(m));
}

TEST(Payloads, APolynomialAndAWaypointRequestRoundTrip) {
    ref::ActivePolynomialReference poly;
    poly.header.stamp = rp::timeOfNs(1);
    poly.trajectory_id = 3;
    poly.revision = 4;
    poly.flags = 5;
    poly.start_time = rp::timeOfNs(7000000000LL);
    poly.duration = 9.0;
    poly.order = 7;
    for (uint32_t s = 0; s < XGC2_UGV_MAX_POLYNOMIAL_SEGMENTS; ++s) {
        poly.segment_durations.push_back(0.1 * (s + 1));
        for (uint32_t c = 0; c < XGC2_UGV_POLYNOMIAL_COEFFS; ++c) {
            poly.coeff_x.push_back(s + 0.01 * c);
            poly.coeff_y.push_back(-static_cast<double>(s) - 0.01 * c);
        }
    }
    ASSERT_TRUE(rp::fits(poly));
    auto p = std::make_unique<xgc2_ugv_polynomial_reference>();
    std::memset(p.get(), 0, sizeof *p);
    rp::toPayload(*p, poly);
    ref::ActivePolynomialReference back;
    ASSERT_TRUE(rp::toPlain(*p, back));
    EXPECT_EQ(back.segment_durations, poly.segment_durations);
    EXPECT_EQ(back.coeff_x, poly.coeff_x);
    EXPECT_EQ(back.coeff_y, poly.coeff_y);
    EXPECT_TRUE(back.coeff_yaw.empty());
    EXPECT_EQ(back.order, 7);
    poly.coeff_yaw.assign(XGC2_UGV_MAX_POLYNOMIAL_SEGMENTS * XGC2_UGV_POLYNOMIAL_COEFFS + 1, 0.0);
    EXPECT_FALSE(rp::fits(poly));

    ref::WaypointReferenceRequest w;
    w.header.stamp = rp::timeOfNs(8000000000LL);
    w.request_id = 1;
    w.trajectory_id = 2;
    w.revision = 3;
    w.flags = 4;
    for (uint32_t k = 0; k < XGC2_UGV_MAX_WAYPOINTS; ++k) {
        ref::Pose pose;
        pose.position = {1.0 * k, 2.0 * k, 3.0 * k};
        pose.orientation = {0.0, 0.0, 0.6, 0.8};
        w.waypoints.push_back(pose);
        w.segment_times.push_back(1.0 + k);
    }
    w.start_velocity = {1, 2, 3};
    w.end_acceleration = {4, 5, 6};
    w.desired_speed = 0.6;
    w.max_velocity = 0.7;
    w.max_acceleration = 0.8;
    w.max_yaw_rate = 0.9;
    w.max_linear_acceleration = 1.1;
    w.objective = 1;
    ASSERT_TRUE(rp::fits(w));
    auto wp = std::make_unique<xgc2_ugv_waypoint_request>();
    std::memset(wp.get(), 0, sizeof *wp);
    rp::toPayload(*wp, w);
    ref::WaypointReferenceRequest wback;
    ASSERT_TRUE(rp::toPlain(*wp, wback));
    ASSERT_EQ(wback.waypoints.size(), w.waypoints.size());
    EXPECT_EQ(wback.waypoints[63].position.z, 189.0);
    EXPECT_EQ(wback.waypoints[63].orientation.w, 0.8);
    EXPECT_EQ(wback.segment_times, w.segment_times);
    EXPECT_EQ(wback.start_velocity.z, 3.0);
    EXPECT_EQ(wback.end_acceleration.x, 4.0);
    EXPECT_EQ(wback.desired_speed, 0.6);
    EXPECT_EQ(wback.max_linear_acceleration, 1.1);
    EXPECT_EQ(wback.objective, 1);
    EXPECT_EQ(wback.header.stamp, w.header.stamp);
    w.waypoints.push_back(ref::Pose());
    EXPECT_FALSE(rp::fits(w));
}

TEST(Payloads, AStatusRoundTrips) {
    ref::ReferenceStatus m;
    m.header.stamp = rp::timeOfNs(42);
    m.state = ref::ReferenceStatus::STATE_ACTIVE;
    m.flags = 0x21;
    m.active_trajectory_id = 5;
    m.active_revision = 6;
    m.active_type = ref::ReferenceStatus::TYPE_SAMPLED;
    xgc2_ugv_reference_status p;
    std::memset(&p, 0, sizeof p);
    rp::toPayload(p, m);
    const ref::ReferenceStatus back = rp::toPlain(p);
    EXPECT_EQ(back.header.stamp, m.header.stamp);
    EXPECT_EQ(back.state, m.state);
    EXPECT_EQ(back.flags, m.flags);
    EXPECT_EQ(back.active_trajectory_id, 5u);
    EXPECT_EQ(back.active_revision, 6u);
    EXPECT_EQ(back.active_type, m.active_type);
}

}  // namespace
