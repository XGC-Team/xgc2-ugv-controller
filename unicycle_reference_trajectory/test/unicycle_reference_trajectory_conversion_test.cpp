// The ROS edge of the reference runtime: messages and plain types convert field for field.

#include <gtest/gtest.h>
#include <ros/time.h>

#include <cmath>
#include <limits>

#include "unicycle_reference_trajectory/ros_reference_conversion.h"

namespace {

namespace urt = unicycle_reference_trajectory;
namespace ref = unicycle_reference_trajectory::reference;
namespace msgs = unicycle_reference_trajectory_msgs;

std_msgs::Header header(uint32_t seq, double stamp, const char* frame) {
    std_msgs::Header h;
    h.seq = seq;
    h.stamp = ros::Time(stamp);
    h.frame_id = frame;
    return h;
}

// The core Time reproduces ros::Time exactly, whatever the conversion.
TEST(ReferenceTime, MatchesRosTimeForSecondsAndNanoseconds) {
    const double samples[] = {0.0,
                              1e-9,
                              0.5,
                              1.0,
                              1000.35,
                              1700000000.1,
                              1700000000.9999999,
                              4294967295.0,
                              123456.7890123456};
    for (double value : samples) {
        const ros::Time ros_time(value);
        const urt::Time core(value);
        EXPECT_EQ(core.sec, ros_time.sec) << value;
        EXPECT_EQ(core.nsec, ros_time.nsec) << value;
        EXPECT_DOUBLE_EQ(core.toSec(), ros_time.toSec()) << value;
        EXPECT_EQ(core.toNSec(), ros_time.toNSec()) << value;
        EXPECT_EQ(urt::toRos(urt::toCore(ros_time)), ros_time);
    }
    EXPECT_THROW(urt::Time(-1.0), std::runtime_error);
    EXPECT_THROW(ros::Time(-1.0), std::runtime_error);
    EXPECT_THROW(urt::Time(std::nan("")), std::runtime_error);
    EXPECT_THROW(urt::Time(5.0e9), std::runtime_error);
    EXPECT_THROW(ros::Time(5.0e9), std::runtime_error);
    urt::Time from_ns;
    from_ns.fromNSec(1700000000123456789ULL);
    ros::Time ros_from_ns;
    ros_from_ns.fromNSec(1700000000123456789ULL);
    EXPECT_EQ(from_ns.sec, ros_from_ns.sec);
    EXPECT_EQ(from_ns.nsec, ros_from_ns.nsec);
}

TEST(ReferenceConversion, AnalyticRoundTrip) {
    msgs::AnalyticReference m;
    m.header = header(4U, 1000.25, "world");
    m.request_id = 11U;
    m.trajectory_id = 12U;
    m.revision = 13U;
    m.analytic_type = msgs::AnalyticReference::ANALYTIC_FIGURE_EIGHT;
    m.flags = 0x8001U;
    m.start_time = ros::Time(1005.5);
    m.duration = 17.25;
    m.origin.position.x = 1.5;
    m.origin.position.y = -2.5;
    m.origin.position.z = 0.125;
    m.origin.orientation.x = 0.1;
    m.origin.orientation.y = 0.2;
    m.origin.orientation.z = 0.3;
    m.origin.orientation.w = 0.9;
    m.params = {2.0, std::numeric_limits<double>::infinity(), -0.0, 1e-300};
    const auto core = urt::toCore(m);
    EXPECT_EQ(core.header.seq, 4U);
    EXPECT_EQ(core.header.frame_id, "world");
    EXPECT_EQ(core.analytic_type, ref::AnalyticReference::ANALYTIC_FIGURE_EIGHT);
    EXPECT_EQ(core.start_time.sec, 1005U);
    EXPECT_DOUBLE_EQ(core.origin.orientation.w, 0.9);
    EXPECT_EQ(core.params, m.params);
    const auto back = urt::toRos(core);
    EXPECT_EQ(back, m);
}

TEST(ReferenceConversion, SampledRoundTrip) {
    msgs::SampledReference m;
    m.header = header(2U, 50.0, "map");
    m.trajectory_id = 3U;
    m.revision = 4U;
    m.flags = msgs::SampledReference::FLAG_EXPLICIT_PLANAR_KINEMATICS;
    m.start_time = ros::Time(60.125);
    m.sample_dt = 0.05;
    for (int k = 0; k < 4; ++k) {
        msgs::PlanarReferencePoint p;
        p.t_from_start = 0.05 * k;
        p.x = 1.0 + k;
        p.y = 2.0 + k;
        p.yaw = 0.1 * k;
        p.speed = 0.5;
        p.linear_acceleration = 0.01 * k;
        p.yaw_rate = 0.25;
        p.yaw_acceleration = 0.02;
        p.curvature = 0.5;
        p.vx = 0.4;
        p.vy = 0.3;
        p.ax = -0.1;
        p.ay = 0.1;
        p.jx = 0.01;
        p.jy = -0.01;
        m.points.push_back(p);
    }
    const auto core = urt::toCore(m);
    ASSERT_EQ(core.points.size(), 4U);
    EXPECT_DOUBLE_EQ(core.points[3].jy, -0.01);
    EXPECT_DOUBLE_EQ(core.points[2].t_from_start, 0.1);
    EXPECT_EQ(urt::toRos(core), m);
}

TEST(ReferenceConversion, WaypointRequestRoundTrip) {
    msgs::WaypointReferenceRequest m;
    m.header = header(9U, 70.0, "world");
    m.request_id = 21U;
    m.trajectory_id = 22U;
    m.revision = 0U;
    m.flags = 5U;
    for (int k = 0; k < 3; ++k) {
        geometry_msgs::Pose pose;
        pose.position.x = k;
        pose.position.y = 0.5 * k;
        pose.orientation.z = std::sin(0.2 * k);
        pose.orientation.w = std::cos(0.2 * k);
        m.waypoints.push_back(pose);
    }
    m.segment_times = {2.0, 3.0};
    m.start_velocity.x = 0.2;
    m.start_acceleration.y = 0.1;
    m.end_velocity.y = -0.3;
    m.end_acceleration.x = -0.1;
    m.desired_speed = 0.6;
    m.max_velocity = 1.0;
    m.max_acceleration = 2.0;
    m.max_yaw_rate = 0.5;
    m.max_linear_acceleration = 1.5;
    m.objective = msgs::WaypointReferenceRequest::OBJECTIVE_SEPTIC_INTERPOLATION;
    const auto core = urt::toCore(m);
    ASSERT_EQ(core.waypoints.size(), 3U);
    EXPECT_DOUBLE_EQ(core.waypoints[2].orientation.z, std::sin(0.4));
    EXPECT_EQ(core.segment_times, m.segment_times);
    EXPECT_DOUBLE_EQ(core.end_velocity.y, -0.3);
    EXPECT_EQ(core.objective, 1U);
    EXPECT_EQ(urt::toRos(core), m);
}

TEST(ReferenceConversion, PolynomialAndStatusToRos) {
    ref::ActivePolynomialReference p;
    p.header.seq = 1U;
    p.header.stamp = urt::Time(10.5);
    p.header.frame_id = "world";
    p.trajectory_id = 2U;
    p.revision = 3U;
    p.flags = 4U;
    p.start_time = urt::Time(11.25);
    p.duration = 8.0;
    p.order = 7U;
    p.segment_durations = {4.0, 4.0};
    p.coeff_x = {1.0, 2.0};
    p.coeff_y = {3.0, 4.0};
    p.coeff_yaw = {5.0, 6.0};
    const auto m = urt::toRos(p);
    EXPECT_EQ(m.header.frame_id, "world");
    EXPECT_EQ(m.start_time, ros::Time(11.25));
    EXPECT_EQ(m.order, 7U);
    EXPECT_EQ(m.coeff_yaw, p.coeff_yaw);

    ref::ReferenceStatus s;
    s.header.stamp = urt::Time(12.0);
    s.state = ref::ReferenceStatus::STATE_ACTIVE;
    s.flags = 96U;
    s.active_trajectory_id = 5U;
    s.active_revision = 6U;
    s.active_type = ref::ReferenceStatus::TYPE_SAMPLED;
    const auto status = urt::toRos(s);
    EXPECT_EQ(status.state, msgs::ReferenceStatus::STATE_ACTIVE);
    EXPECT_EQ(status.flags, 96U);
    EXPECT_EQ(status.active_type, msgs::ReferenceStatus::TYPE_SAMPLED);
    EXPECT_EQ(status.header.stamp, ros::Time(12.0));
}

}  // namespace

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
