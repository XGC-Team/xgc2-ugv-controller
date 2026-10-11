// The reference module as a shared library, loaded the way the host loads it, in an in-test host.

#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <string>

#include "test_host.h"
#include "xgc2_ugv/payloads.h"

namespace {

using ugv_modules_test::ModuleLibrary;
using ugv_modules_test::TestHost;

constexpr int64_t kSecond = 1000000000LL;
constexpr int64_t kStep = 10000000LL;  // the 100 Hz period of the node's main loop
constexpr int64_t kT0 = 1000 * kSecond;

xgc2_ugv_analytic_reference circle(int64_t stamp_ns, int64_t start_ns = 0) {
    xgc2_ugv_analytic_reference r;
    std::memset(&r, 0, sizeof r);
    r.stamp_ns = stamp_ns;
    r.start_time_ns = start_ns;
    r.request_id = 11;
    r.trajectory_id = 3;
    r.revision = 1;
    r.analytic_type = XGC2_UGV_ANALYTIC_CIRCLE;
    r.duration = 6.0;
    r.origin_qw = 1.0;
    r.param_count = 5;
    const double params[5] = {2.0, 0.8, 3.0, 0.0, 1.0};
    std::memcpy(r.params, params, sizeof params);
    return r;
}

xgc2_ugv_waypoint_request waypoints(int64_t stamp_ns) {
    xgc2_ugv_waypoint_request r;
    std::memset(&r, 0, sizeof r);
    r.stamp_ns = stamp_ns;
    r.request_id = 21;
    r.trajectory_id = 4;
    r.waypoint_count = 3;
    for (uint32_t k = 0; k < 3; ++k) {
        r.waypoints[k].x = 1.0 * k;
        r.waypoints[k].y = (k % 2 == 0) ? 0.0 : 0.8;
        r.waypoints[k].qz = std::sin(0.2 * k);
        r.waypoints[k].qw = std::cos(0.2 * k);
    }
    r.segment_time_count = 2;
    r.segment_times[0] = 2.0;
    r.segment_times[1] = 2.0;
    r.desired_speed = 0.6;
    r.objective = 1;
    return r;
}

xgc2_ugv_sampled_reference arc(int64_t stamp_ns, uint32_t count) {
    xgc2_ugv_sampled_reference r;
    std::memset(&r, 0, sizeof r);
    r.stamp_ns = stamp_ns;
    r.trajectory_id = 5;
    r.revision = 2;
    r.flags = XGC2_UGV_SAMPLED_FLAG_EXPLICIT_PLANAR_KINEMATICS;
    r.sample_dt = 0.05;
    r.point_count = count;
    const double v = 0.5, w = 0.25;
    for (uint32_t k = 0; k < count; ++k) {
        xgc2_ugv_planar_point& p = r.points[k];
        const double s = k * 0.05;
        p.t_from_start = s;
        p.yaw = w * s;
        p.x = v / w * std::sin(p.yaw);
        p.y = v / w * (1.0 - std::cos(p.yaw));
        p.speed = v;
        p.yaw_rate = w;
        p.curvature = w / v;
        p.vx = v * std::cos(p.yaw);
        p.vy = v * std::sin(p.yaw);
        p.ax = -v * w * std::sin(p.yaw);
        p.ay = v * w * std::cos(p.yaw);
        p.jx = -v * w * w * std::cos(p.yaw);
        p.jy = -v * w * w * std::sin(p.yaw);
    }
    return r;
}

class ReferenceModuleTest : public ::testing::Test {
   protected:
    ReferenceModuleTest() : library_(REFERENCE_MODULE_PATH), host_(library_.desc()) {}

    void SetUp() override {
        host_.setNow(kT0);
        ASSERT_EQ(host_.create("{}"), XGC2_OK);
        ASSERT_EQ(host_.start(), XGC2_OK);
    }

    void TearDown() override {
        // The host keeps an instance on one thread; so does this test.
        EXPECT_EQ(host_.callingThreads(), 1u);
    }

    // Steps at the 100 Hz grid up to `seconds` after kT0.
    void runUntil(double seconds) {
        const int64_t end = kT0 + static_cast<int64_t>(seconds * 1e9);
        while (now_ + kStep <= end) {
            now_ += kStep;
            host_.setNow(now_);
            ASSERT_EQ(host_.step(), XGC2_OK);
        }
    }

    ModuleLibrary library_;
    TestHost host_;
    int64_t now_{kT0};
};

TEST_F(ReferenceModuleTest, DescribesItsPortsForTheHost) {
    const xgc2_module_desc* d = library_.desc();
    EXPECT_EQ(d->abi_major, XGC2_MODULE_ABI_MAJOR);
    EXPECT_STREQ(d->name, "ugv_unicycle_reference");
    ASSERT_EQ(d->port_count, 8u);
    struct Expect {
        const char* name;
        xgc2_port_direction direction;
        xgc2_port_kind kind;
        const char* schema;
        uint32_t size;
        uint32_t depth;
    };
    const Expect expected[] = {
        {"analytic_request", XGC2_PORT_IN, XGC2_PORT_EVENT,
         "xgc2.ugv.unicycle_reference.analytic.v1", sizeof(xgc2_ugv_analytic_reference), 8},
        {"sampled_request", XGC2_PORT_IN, XGC2_PORT_EVENT, "xgc2.ugv.unicycle_reference.sampled.v1",
         sizeof(xgc2_ugv_sampled_reference), 4},
        {"waypoint_request", XGC2_PORT_IN, XGC2_PORT_EVENT,
         "xgc2.ugv.unicycle_reference.waypoint_request.v1", sizeof(xgc2_ugv_waypoint_request), 4},
        {"reset", XGC2_PORT_IN, XGC2_PORT_EVENT, "xgc2.ugv.unicycle_reference.reset.v1",
         sizeof(xgc2_ugv_reference_reset), 2},
        {"status", XGC2_PORT_OUT, XGC2_PORT_STATE, "xgc2.ugv.unicycle_reference.status.v1",
         sizeof(xgc2_ugv_reference_status), 0},
        {"active_analytic", XGC2_PORT_OUT, XGC2_PORT_STATE,
         "xgc2.ugv.unicycle_reference.analytic.v1", sizeof(xgc2_ugv_analytic_reference), 0},
        {"active_polynomial", XGC2_PORT_OUT, XGC2_PORT_STATE,
         "xgc2.ugv.unicycle_reference.polynomial.v1", sizeof(xgc2_ugv_polynomial_reference), 0},
        {"active_sampled", XGC2_PORT_OUT, XGC2_PORT_STATE, "xgc2.ugv.unicycle_reference.sampled.v1",
         sizeof(xgc2_ugv_sampled_reference), 0},
    };
    for (uint32_t i = 0; i < d->port_count; ++i) {
        const xgc2_port_desc& p = d->ports[i];
        EXPECT_STREQ(p.name, expected[i].name);
        EXPECT_EQ(p.direction, static_cast<uint32_t>(expected[i].direction)) << p.name;
        EXPECT_EQ(p.kind, static_cast<uint32_t>(expected[i].kind)) << p.name;
        EXPECT_STREQ(p.schema_id, expected[i].schema) << p.name;
        EXPECT_EQ(p.size, expected[i].size) << p.name;
        EXPECT_EQ(p.align, 8u) << p.name;
        EXPECT_EQ(p.queue_depth, expected[i].depth) << p.name;
        EXPECT_EQ(p.flags, 0u) << p.name;
    }
    // The inputs come first, so the input index is the port index.
    EXPECT_EQ(d->ports[3].direction, static_cast<uint32_t>(XGC2_PORT_IN));
    EXPECT_EQ(d->ports[4].direction, static_cast<uint32_t>(XGC2_PORT_OUT));
}

TEST_F(ReferenceModuleTest, ReachesReadyAndPublishesItsStatusAtTheConfiguredRate) {
    runUntil(2.0);
    const auto status = host_.outputsAs<xgc2_ugv_reference_status>("status");
    ASSERT_GE(status.size(), 19u);
    ASSERT_LE(status.size(), 21u);
    EXPECT_EQ(status.back().state, XGC2_UGV_REFERENCE_STATE_READY);
    EXPECT_EQ(status.back().active_type, XGC2_UGV_REFERENCE_TYPE_NONE);
    // The status is stamped with the host clock of the step that wrote it.
    const auto samples = host_.outputs("status");
    for (size_t i = 1; i < samples.size(); ++i) {
        EXPECT_GT(samples[i].stamp_ns, samples[i - 1].stamp_ns);
        EXPECT_EQ(samples[i].stamp_ns % kStep, 0);
    }
    EXPECT_TRUE(host_.outputs("active_analytic").empty());
    EXPECT_EQ(host_.openSlots(), 0u);
}

TEST_F(ReferenceModuleTest, AnAnalyticRequestBecomesTheActiveReference) {
    runUntil(0.5);
    host_.clearOutputs();
    ASSERT_TRUE(host_.push("analytic_request", circle(now_), now_));
    runUntil(1.5);
    const auto active = host_.outputsAs<xgc2_ugv_analytic_reference>("active_analytic");
    ASSERT_FALSE(active.empty());
    EXPECT_EQ(active.back().trajectory_id, 3u);
    EXPECT_EQ(active.back().revision, 1u);
    EXPECT_EQ(active.back().analytic_type, XGC2_UGV_ANALYTIC_CIRCLE);
    EXPECT_EQ(active.back().param_count, 5u);
    EXPECT_DOUBLE_EQ(active.back().params[0], 2.0);
    EXPECT_GT(active.back().start_time_ns, now_ - 2 * kSecond);  // set by the runtime
    const auto status = host_.outputsAs<xgc2_ugv_reference_status>("status");
    ASSERT_FALSE(status.empty());
    EXPECT_EQ(status.back().state, XGC2_UGV_REFERENCE_STATE_ACTIVE);
    EXPECT_EQ(status.back().active_type, XGC2_UGV_REFERENCE_TYPE_ANALYTIC);
    EXPECT_EQ(status.back().active_trajectory_id, 3u);
    // The active reference is repeated at active_publish_rate (10 Hz).
    EXPECT_GE(active.size(), 5u);
}

TEST_F(ReferenceModuleTest, ASampledRequestBecomesTheActiveReference) {
    runUntil(0.5);
    host_.clearOutputs();
    ASSERT_TRUE(host_.push("sampled_request", arc(now_, 120), now_));
    runUntil(1.5);
    const auto active = host_.outputsAs<xgc2_ugv_sampled_reference>("active_sampled");
    ASSERT_FALSE(active.empty());
    EXPECT_EQ(active.back().trajectory_id, 5u);
    EXPECT_EQ(active.back().point_count, 120u);
    EXPECT_DOUBLE_EQ(active.back().points[119].t_from_start, 119 * 0.05);
    EXPECT_EQ(active.back().flags & XGC2_UGV_SAMPLED_FLAG_EXPLICIT_PLANAR_KINEMATICS,
              XGC2_UGV_SAMPLED_FLAG_EXPLICIT_PLANAR_KINEMATICS);
    const auto status = host_.outputsAs<xgc2_ugv_reference_status>("status");
    EXPECT_EQ(status.back().active_type, XGC2_UGV_REFERENCE_TYPE_SAMPLED);
}

TEST_F(ReferenceModuleTest, AWaypointRequestIsPlannedIntoAPolynomialReference) {
    runUntil(0.5);
    host_.clearOutputs();
    ASSERT_TRUE(host_.push("waypoint_request", waypoints(0), now_));
    runUntil(2.0);
    const auto active = host_.outputsAs<xgc2_ugv_polynomial_reference>("active_polynomial");
    ASSERT_FALSE(active.empty());
    EXPECT_EQ(active.back().trajectory_id, 4u);
    EXPECT_EQ(active.back().segment_count, 2u);
    EXPECT_EQ(active.back().order, 7u);
    EXPECT_EQ(active.back().coeff_x_count, 2u * 8u);
    EXPECT_EQ(active.back().coeff_y_count, 2u * 8u);
    EXPECT_NEAR(active.back().segment_durations[0], 2.0, 1e-12);
    const auto status = host_.outputsAs<xgc2_ugv_reference_status>("status");
    EXPECT_EQ(status.back().active_type, XGC2_UGV_REFERENCE_TYPE_POLYNOMIAL);
}

TEST_F(ReferenceModuleTest, AResetRequestRestartsTheGenerator) {
    runUntil(0.5);
    ASSERT_TRUE(host_.push("analytic_request", circle(now_), now_));
    runUntil(1.5);
    ASSERT_EQ(host_.outputsAs<xgc2_ugv_reference_status>("status").back().state,
              XGC2_UGV_REFERENCE_STATE_ACTIVE);
    host_.clearOutputs();
    xgc2_ugv_reference_reset reset{};
    reset.stamp_ns = now_;
    ASSERT_TRUE(host_.push("reset", reset, now_));
    runUntil(3.0);
    const auto status = host_.outputsAs<xgc2_ugv_reference_status>("status");
    ASSERT_FALSE(status.empty());
    EXPECT_EQ(status.back().state, XGC2_UGV_REFERENCE_STATE_READY);
    EXPECT_EQ(status.back().active_type, XGC2_UGV_REFERENCE_TYPE_NONE);
}

TEST_F(ReferenceModuleTest, RequestsOfSeveralPortsAreServedInTheOrderTheyWereCommitted) {
    runUntil(0.5);
    // Both requests wait in the same step, and the later one replaces the earlier: whichever port
    // holds it, the commit stamps decide.
    ASSERT_TRUE(host_.push("sampled_request", arc(now_, 120), now_ + 2000));
    ASSERT_TRUE(host_.push("analytic_request", circle(now_), now_ + 1000));
    runUntil(1.5);
    EXPECT_EQ(host_.outputsAs<xgc2_ugv_reference_status>("status").back().active_type,
              XGC2_UGV_REFERENCE_TYPE_SAMPLED);

    ASSERT_EQ(host_.configure("{}"), XGC2_OK);  // start over
    runUntil(2.5);
    ASSERT_EQ(host_.outputsAs<xgc2_ugv_reference_status>("status").back().state,
              XGC2_UGV_REFERENCE_STATE_READY);
    ASSERT_TRUE(host_.push("analytic_request", circle(now_), now_ + 2000));
    ASSERT_TRUE(host_.push("sampled_request", arc(now_, 120), now_ + 1000));
    runUntil(3.5);
    EXPECT_EQ(host_.outputsAs<xgc2_ugv_reference_status>("status").back().active_type,
              XGC2_UGV_REFERENCE_TYPE_ANALYTIC);
}

TEST_F(ReferenceModuleTest, TheConfigurationIsStrict) {
    TestHost fresh(library_.desc());
    EXPECT_EQ(fresh.create("{\"status_rat\": 20.0}"), XGC2_ERR_INVALID);
    EXPECT_TRUE(fresh.logged(3, "unknown configuration key 'status_rat'"));
    EXPECT_FALSE(fresh.created());

    TestHost typed(library_.desc());
    EXPECT_EQ(typed.create("{\"status_rate\": \"fast\"}"), XGC2_ERR_INVALID);
    EXPECT_TRUE(typed.logged(3, "'status_rate' must be a number"));

    TestHost malformed(library_.desc());
    EXPECT_EQ(malformed.create("{\"status_rate\": 20.0,}"), XGC2_ERR_INVALID);
    EXPECT_EQ(malformed.create("[1, 2]"), XGC2_ERR_INVALID);

    TestHost nested(library_.desc());
    EXPECT_EQ(nested.create("{\"default_analytic\": {\"enabled\": 1}}"), XGC2_ERR_INVALID);
    EXPECT_TRUE(nested.logged(3, "'default_analytic/enabled' must be a boolean"));

    TestHost good(library_.desc());
    EXPECT_EQ(good.create("{\"status_rate\": 20, \"max_velocity\": 0.8, "
                          "\"default_analytic\": {\"radius\": 2.5, \"analytic_type\": 1}}"),
              XGC2_OK);
}

TEST_F(ReferenceModuleTest, ALiveConfigureRestartsTheGeneratorAndARefusedOneChangesNothing) {
    runUntil(0.5);
    ASSERT_TRUE(host_.push("analytic_request", circle(now_), now_));
    runUntil(1.5);
    ASSERT_EQ(host_.outputsAs<xgc2_ugv_reference_status>("status").back().state,
              XGC2_UGV_REFERENCE_STATE_ACTIVE);

    EXPECT_EQ(host_.configure("{\"status_rate\": \"fast\"}"), XGC2_ERR_INVALID);
    host_.clearOutputs();
    runUntil(2.0);
    EXPECT_EQ(host_.outputsAs<xgc2_ugv_reference_status>("status").back().state,
              XGC2_UGV_REFERENCE_STATE_ACTIVE);

    EXPECT_EQ(host_.configure("{\"status_rate\": 50}"), XGC2_OK);
    host_.clearOutputs();
    runUntil(3.0);
    const auto status = host_.outputsAs<xgc2_ugv_reference_status>("status");
    ASSERT_GT(status.size(), 40u);  // 50 Hz, one second
    EXPECT_EQ(status.back().state, XGC2_UGV_REFERENCE_STATE_READY);
}

TEST_F(ReferenceModuleTest, TheDefaultAnalyticReferenceIsIssuedOnceTheGeneratorIsReady) {
    TestHost host(library_.desc());
    host.setNow(kT0);
    ASSERT_EQ(host.create("{\"default_analytic\": {\"enabled\": true, \"analytic_type\": 1, "
                          "\"radius\": 2.0, \"line_speed\": 0.7, \"duration\": 30.0, "
                          "\"start_delay\": 0.5, \"trajectory_id\": 9}}"),
              XGC2_OK);
    ASSERT_EQ(host.start(), XGC2_OK);
    int64_t now = kT0;
    for (int k = 0; k < 300; ++k) {
        now += kStep;
        host.setNow(now);
        ASSERT_EQ(host.step(), XGC2_OK);
    }
    const auto active = host.outputsAs<xgc2_ugv_analytic_reference>("active_analytic");
    ASSERT_FALSE(active.empty());
    EXPECT_EQ(active.back().trajectory_id, 9u);
    EXPECT_EQ(active.back().analytic_type, XGC2_UGV_ANALYTIC_CIRCLE);
    EXPECT_DOUBLE_EQ(active.back().params[0], 2.0);
}

TEST_F(ReferenceModuleTest, ARequestThatDoesNotFitItsPayloadIsRefusedNotTruncated) {
    runUntil(0.5);
    xgc2_ugv_analytic_reference bad = circle(now_);
    bad.param_count = XGC2_UGV_MAX_ANALYTIC_PARAMS + 1;
    ASSERT_TRUE(host_.push("analytic_request", bad, now_));
    xgc2_ugv_sampled_reference many = arc(now_, 10);
    many.point_count = XGC2_UGV_MAX_SAMPLED_POINTS + 1;
    ASSERT_TRUE(host_.push("sampled_request", many, now_));
    xgc2_ugv_waypoint_request wp = waypoints(0);
    wp.waypoint_count = XGC2_UGV_MAX_WAYPOINTS + 1;
    ASSERT_TRUE(host_.push("waypoint_request", wp, now_));
    runUntil(1.5);
    EXPECT_TRUE(host_.outputs("active_analytic").empty());
    EXPECT_TRUE(host_.outputs("active_sampled").empty());
    EXPECT_TRUE(host_.outputs("active_polynomial").empty());
    EXPECT_TRUE(host_.logged(2, "Refused an analytic reference"));
    EXPECT_TRUE(host_.logged(2, "Refused a sampled reference"));
    EXPECT_TRUE(host_.logged(2, "Refused a waypoint request"));
    EXPECT_EQ(host_.outputsAs<xgc2_ugv_reference_status>("status").back().state,
              XGC2_UGV_REFERENCE_STATE_READY);
}

TEST_F(ReferenceModuleTest, OutputsAreWrittenWholeAndSlotsAreNeverLeaked) {
    runUntil(0.5);
    ASSERT_TRUE(host_.push("analytic_request", circle(now_), now_));
    runUntil(1.5);
    // The test host fills a slot with 0xA5 before the module gets it: whatever the module did not
    // write must have been zeroed, so the whole payload equals the same fields over zeros.
    const auto samples = host_.outputs("status");
    ASSERT_FALSE(samples.empty());
    const auto status = samples.back().as<xgc2_ugv_reference_status>();
    xgc2_ugv_reference_status expected;
    std::memset(&expected, 0, sizeof expected);
    expected.stamp_ns = status.stamp_ns;
    expected.state = status.state;
    expected.flags = status.flags;
    expected.active_trajectory_id = status.active_trajectory_id;
    expected.active_revision = status.active_revision;
    expected.active_type = status.active_type;
    EXPECT_EQ(std::memcmp(&expected, &status, sizeof status), 0);

    const auto analytic = host_.outputs("active_analytic").back().as<xgc2_ugv_analytic_reference>();
    for (uint32_t k = analytic.param_count; k < XGC2_UGV_MAX_ANALYTIC_PARAMS; ++k) {
        EXPECT_EQ(analytic.params[k], 0.0) << k;
    }
    EXPECT_EQ(analytic.reserved, 0u);
    EXPECT_EQ(host_.openSlots(), 0u);

    // An output that refuses writes (a full queue) costs the sample, not the instance.
    host_.limitOutput("status", 0);
    host_.limitOutput("active_analytic", 0);
    runUntil(2.5);
    EXPECT_GT(host_.refusedWrites("status"), 0u);
    EXPECT_GT(host_.refusedWrites("active_analytic"), 0u);
    EXPECT_EQ(host_.openSlots(), 0u);
    EXPECT_TRUE(host_.reports().empty());
}

// The host clock starts wherever it likes: CLOCK_MONOTONIC counts from boot, a simulation from
// zero, a recorded flight from the wall clock of the day.
class ReferenceClockTest : public ::testing::TestWithParam<int64_t> {};

TEST_P(ReferenceClockTest, ServesARequestWhateverTheClockReads) {
    ModuleLibrary library(REFERENCE_MODULE_PATH);
    TestHost host(library.desc());
    const int64_t t0 = GetParam();
    int64_t now = t0;
    host.setNow(now);
    ASSERT_EQ(host.create("{}"), XGC2_OK);
    ASSERT_EQ(host.start(), XGC2_OK);
    auto run = [&](double seconds) {
        const int64_t end = now + static_cast<int64_t>(seconds * 1e9);
        while (now + kStep <= end) {
            now += kStep;
            host.setNow(now);
            ASSERT_EQ(host.step(), XGC2_OK);
        }
    };
    run(0.5);
    ASSERT_TRUE(host.push("analytic_request", circle(now, 0), now));
    run(1.5);
    const auto status = host.outputsAs<xgc2_ugv_reference_status>("status");
    ASSERT_FALSE(status.empty());
    EXPECT_EQ(status.back().state, XGC2_UGV_REFERENCE_STATE_ACTIVE) << "clock " << t0;
    EXPECT_EQ(status.back().flags, 0u) << "clock " << t0;
    EXPECT_EQ(status.back().active_type, XGC2_UGV_REFERENCE_TYPE_ANALYTIC);
    const auto active = host.outputsAs<xgc2_ugv_analytic_reference>("active_analytic");
    ASSERT_FALSE(active.empty());
    EXPECT_GT(active.back().start_time_ns, now - 2 * kSecond);
    EXPECT_LT(active.back().start_time_ns, now + 2 * kSecond);
}

INSTANTIATE_TEST_SUITE_P(Clocks, ReferenceClockTest,
                         ::testing::Values(int64_t{1} * kSecond, int64_t{1000} * kSecond,
                                           int64_t{150737250000000},  // an uptime of 42 hours
                                           int64_t{1700000000} * kSecond));  // the wall clock

// The state machine of the generator belongs to the thread that built it. The host keeps an
// instance on one thread (affinity "sticky"); a host that did not would leave the generator half
// alive without a sound, so the module checks and fails the instance with the reason.
TEST(ReferenceModuleThreads, AStepOnAnotherThreadThanTheBuilderFailsTheInstanceLoudly) {
    ModuleLibrary library(REFERENCE_MODULE_PATH);
    TestHost host(library.desc());
    ugv_modules_test::RotatingThreads threads(2);  // the builder's and another one
    int64_t now = kT0;
    host.setNow(now);
    xgc2_status status = XGC2_OK;
    threads.run([&] { status = host.create("{}"); });  // thread 0 builds the generator
    ASSERT_EQ(status, XGC2_OK);
    threads.run([&] { status = host.start(); });  // thread 1; start does not touch it
    ASSERT_EQ(status, XGC2_OK);
    auto step = [&] {
        now += kStep;
        host.setNow(now);
        threads.run([&] { status = host.step(); });
    };
    step();  // thread 0
    EXPECT_EQ(status, XGC2_OK);
    EXPECT_TRUE(host.reports().empty());
    step();  // thread 1
    EXPECT_EQ(status, XGC2_ERR_INTERNAL);
    ASSERT_FALSE(host.reports().empty());
    EXPECT_EQ(host.reports().back().first, 2);  // failed
    EXPECT_NE(host.reports().back().second.find("another thread"), std::string::npos)
        << host.reports().back().second;
    EXPECT_NE(host.reports().back().second.find("affinity"), std::string::npos);
    EXPECT_EQ(host.callingThreads(), 2u);
    threads.run([&] { host.destroy(); });
}

}  // namespace
