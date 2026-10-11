// The reference trajectory runtime and its driver, through the ROS-free core only.

#include "unicycle_reference_trajectory/unicycle_reference_trajectory_runtime.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "unicycle_reference_trajectory/config_loader.h"
#include "unicycle_reference_trajectory/default_analytic.h"
#include "unicycle_reference_trajectory/reference_driver.h"
#include "unicycle_reference_trajectory/reference_types.h"

namespace {

namespace urt = unicycle_reference_trajectory;
namespace ref = unicycle_reference_trajectory::reference;
namespace trajectory = xgc2_math::trajectory;
using Request = urt::ReferenceTrajectoryDriver::Request;

constexpr double kT0 = 100.0;
constexpr double kDt = 0.01;

ref::AnalyticReference circle(uint32_t id, double start = 0.0, double duration = 6.0) {
    ref::AnalyticReference msg;
    msg.request_id = 10U + id;
    msg.trajectory_id = id;
    msg.revision = 3U;
    msg.analytic_type = ref::AnalyticReference::ANALYTIC_CIRCLE;
    msg.start_time = start > 0.0 ? urt::Time(start) : urt::Time();
    msg.duration = duration;
    msg.origin.orientation.w = 1.0;
    msg.params = {2.0, 0.8, 3.0, 0.0, 1.0};
    return msg;
}

ref::SampledReference arc(uint32_t id, int count, bool monotonic = true) {
    ref::SampledReference msg;
    msg.trajectory_id = id;
    msg.revision = 2U;
    msg.flags = ref::SampledReference::FLAG_EXPLICIT_PLANAR_KINEMATICS;
    msg.sample_dt = 0.05;
    const double v = 0.5, w = 0.25;
    for (int k = 0; k < count; ++k) {
        ref::PlanarReferencePoint p;
        p.t_from_start = (monotonic || k != count / 2) ? 0.05 * k : 0.0;
        p.yaw = w * p.t_from_start;
        p.x = v / w * std::sin(p.yaw);
        p.y = v / w * (1.0 - std::cos(p.yaw));
        p.speed = v;
        p.yaw_rate = w;
        p.curvature = w / v;
        p.vx = v * std::cos(p.yaw);
        p.vy = v * std::sin(p.yaw);
        p.ax = -v * w * std::sin(p.yaw);
        p.ay = v * w * std::cos(p.yaw);
        msg.points.push_back(p);
    }
    return msg;
}

ref::WaypointReferenceRequest line(int count, std::vector<double> segment_times = {}) {
    ref::WaypointReferenceRequest msg;
    msg.trajectory_id = 5U;
    for (int k = 0; k < count; ++k) {
        ref::Pose pose;
        pose.position.x = 1.0 * k;
        pose.position.y = (k % 2 == 0) ? 0.0 : 0.5;
        pose.orientation.w = 1.0;
        msg.waypoints.push_back(pose);
    }
    msg.segment_times = std::move(segment_times);
    msg.desired_speed = 0.6;
    return msg;
}

// Runs the driver at 100 Hz from `from` for `seconds`, collecting output event ids.
class Clock {
   public:
    explicit Clock(urt::ReferenceTrajectoryDriver& driver) : driver_(driver) {}

    std::vector<uint32_t> run(double seconds) {
        std::vector<uint32_t> ids;
        const int steps = static_cast<int>(std::lround(seconds / kDt));
        for (int k = 0; k < steps; ++k) {
            now_ += kDt;
            for (const auto& event : driver_.update(now_).events) {
                ids.push_back(event.id);
            }
        }
        return ids;
    }
    double now() const {
        return now_;
    }

   private:
    urt::ReferenceTrajectoryDriver& driver_;
    double now_{kT0};
};

int count(const std::vector<uint32_t>& ids, uint32_t id) {
    return static_cast<int>(std::count(ids.begin(), ids.end(), id));
}

TEST(ReferenceRuntime, StartsInSelfCheckAndBecomesReady) {
    urt::ReferenceTrajectoryDriver driver;
    EXPECT_EQ(driver.runtime().currentState(), ref::ReferenceStatus::STATE_SELF_CHECK);
    Clock clock(driver);
    const auto ids = clock.run(0.5);
    EXPECT_EQ(driver.runtime().currentState(), ref::ReferenceStatus::STATE_READY);
    EXPECT_GE(count(ids, urt::output_event_type::PUBLISH_STATUS), 4);
    EXPECT_EQ(count(ids, urt::output_event_type::PUBLISH_ACTIVE_ANALYTIC), 0);
    const auto status = driver.runtime().makeStatus(clock.now());
    EXPECT_EQ(status.state, ref::ReferenceStatus::STATE_READY);
    EXPECT_EQ(status.active_type, ref::ReferenceStatus::TYPE_NONE);
    EXPECT_EQ(status.active_trajectory_id, 0U);
}

TEST(ReferenceRuntime, AnalyticRequestActivatesWithAdjustedStartTime) {
    urt::ReferenceTrajectoryDriver driver;
    Clock clock(driver);
    clock.run(0.1);
    const double now = clock.now();
    ASSERT_EQ(driver.acceptAnalytic(circle(7U), now), Request::kPosted);
    const auto ids = clock.run(1.0);
    const auto& rt = driver.runtime();
    EXPECT_EQ(rt.currentState(), ref::ReferenceStatus::STATE_ACTIVE);
    EXPECT_EQ(rt.activeType(), trajectory::TrajectoryModelType::kAnalytic);
    EXPECT_EQ(rt.activeTrajectoryId(), 7U);
    EXPECT_EQ(rt.activeRevision(), 3U);
    // No start time requested: the earliest start is now plus the lead time.
    EXPECT_NEAR(rt.activeAnalyticMessage().start_time.toSec(), now + rt.config().min_lead_time,
                1e-6);
    EXPECT_GE(count(ids, urt::output_event_type::PUBLISH_ACTIVE_ANALYTIC), 9);
    ASSERT_NE(rt.evaluator(), nullptr);
    trajectory::PlanarReference2 sample;
    ASSERT_TRUE(rt.evaluator()->evaluate(1.0, sample));
    EXPECT_TRUE(trajectory::TrajectoryValidator2::finite(sample));
    const auto status = rt.makeStatus(clock.now());
    EXPECT_EQ(status.state, ref::ReferenceStatus::STATE_ACTIVE);
    EXPECT_EQ(status.active_type, ref::ReferenceStatus::TYPE_ANALYTIC);
}

TEST(ReferenceRuntime, FutureStartTimeIsKeptAndNearStartTimeIsPushedByTheLeadTime) {
    urt::ReferenceTrajectoryDriver driver;
    Clock clock(driver);
    clock.run(0.1);
    const double now = clock.now();
    ASSERT_EQ(driver.acceptAnalytic(circle(1U, now + 5.0), now), Request::kPosted);
    clock.run(0.1);
    EXPECT_NEAR(driver.runtime().activeAnalyticMessage().start_time.toSec(), now + 5.0, 1e-6);

    const double later = clock.now();
    ASSERT_EQ(driver.acceptAnalytic(circle(2U, later + 0.01), later), Request::kPosted);
    clock.run(0.1);
    EXPECT_NEAR(driver.runtime().activeAnalyticMessage().start_time.toSec(),
                later + driver.runtime().config().min_lead_time, 1e-6);
}

TEST(ReferenceRuntime, UnknownAnalyticTypeAndNonFiniteParametersUseTheDefaults) {
    urt::ReferenceTrajectoryDriver driver;
    Clock clock(driver);
    clock.run(0.1);
    auto odd = circle(3U);
    odd.analytic_type = 77U;
    odd.params = {1.0, 1.0, 1.0};
    EXPECT_EQ(driver.acceptAnalytic(odd, clock.now()), Request::kPosted);
    clock.run(0.1);
    ASSERT_NE(driver.runtime().evaluator(), nullptr);
    EXPECT_EQ(driver.runtime().activeType(), trajectory::TrajectoryModelType::kAnalytic);

    auto nonfinite = circle(4U);
    nonfinite.params = {std::nan(""), 0.8, std::numeric_limits<double>::infinity()};
    EXPECT_EQ(driver.acceptAnalytic(nonfinite, clock.now()), Request::kPosted);
}

TEST(ReferenceRuntime, SampledRequestsAreValidated) {
    urt::ReferenceTrajectoryDriver driver;
    Clock clock(driver);
    clock.run(0.1);
    EXPECT_EQ(driver.acceptSampled(arc(1U, 0), clock.now()), Request::kRejected);
    EXPECT_EQ(driver.acceptSampled(arc(2U, 60, false), clock.now()), Request::kRejected);
    EXPECT_NE(driver.runtime().flags() & trajectory::kFlagInvalidInput, 0U);
    EXPECT_EQ(driver.runtime().currentState(), ref::ReferenceStatus::STATE_READY);
    ASSERT_EQ(driver.acceptSampled(arc(3U, 100), clock.now()), Request::kPosted);
    const auto ids = clock.run(0.5);
    EXPECT_EQ(driver.runtime().currentState(), ref::ReferenceStatus::STATE_ACTIVE);
    EXPECT_EQ(driver.runtime().activeType(), trajectory::TrajectoryModelType::kSampled);
    EXPECT_GE(count(ids, urt::output_event_type::PUBLISH_ACTIVE_SAMPLED), 4);
    EXPECT_EQ(driver.runtime().activeSampledMessage().points.size(), 100U);
    EXPECT_EQ(driver.runtime().flags(), ref::SampledReference::FLAG_EXPLICIT_PLANAR_KINEMATICS);
}

TEST(ReferenceRuntime, WaypointRequestIsPlannedIntoAPolynomial) {
    urt::ReferenceTrajectoryDriver driver;
    Clock clock(driver);
    clock.run(0.1);
    EXPECT_EQ(driver.acceptWaypoint(line(1), clock.now()), Request::kRejected);
    EXPECT_EQ(driver.acceptWaypoint(line(3, {2.0}), clock.now()), Request::kRejected);
    ASSERT_EQ(driver.acceptWaypoint(line(3, {2.0, 2.0}), clock.now()), Request::kPosted);
    const auto ids = clock.run(0.5);
    const auto& rt = driver.runtime();
    EXPECT_EQ(rt.currentState(), ref::ReferenceStatus::STATE_ACTIVE);
    EXPECT_EQ(rt.activeType(), trajectory::TrajectoryModelType::kPolynomial);
    EXPECT_GE(count(ids, urt::output_event_type::PUBLISH_ACTIVE_POLYNOMIAL), 4);
    const auto& polynomial = rt.activePolynomialMessage();
    EXPECT_EQ(polynomial.order, 7U);
    EXPECT_EQ(polynomial.segment_durations.size(), 2U);
    EXPECT_EQ(polynomial.coeff_x.size(), 16U);
    EXPECT_NEAR(polynomial.duration, 4.0, 1e-9);
    // No revision requested: the planner numbers it after the previous active one.
    EXPECT_EQ(polynomial.revision, 1U);
    EXPECT_EQ(rt.makeStatus(clock.now()).active_type, ref::ReferenceStatus::TYPE_POLYNOMIAL);
    trajectory::PlanarReference2 end;
    ASSERT_TRUE(rt.evaluator()->evaluate(4.0, end));
    EXPECT_NEAR(end.position.x(), 2.0, 1e-9);
}

TEST(ReferenceRuntime, ActiveReferenceExpiresBackToReady) {
    urt::ReferenceTrajectoryDriver driver;
    Clock clock(driver);
    clock.run(0.1);
    ASSERT_EQ(driver.acceptAnalytic(circle(1U, 0.0, 1.0), clock.now()), Request::kPosted);
    clock.run(1.0);
    EXPECT_EQ(driver.runtime().currentState(), ref::ReferenceStatus::STATE_ACTIVE);
    // start + duration + trajectory_timeout = 0.2 + 1.0 + 0.5 s after the request
    clock.run(1.0);
    EXPECT_EQ(driver.runtime().currentState(), ref::ReferenceStatus::STATE_READY);
}

TEST(ReferenceRuntime, ResetRestartsTheRuntime) {
    urt::ReferenceTrajectoryDriver driver;
    Clock clock(driver);
    clock.run(0.1);
    ASSERT_EQ(driver.acceptAnalytic(circle(1U), clock.now()), Request::kPosted);
    clock.run(0.5);
    ASSERT_EQ(driver.runtime().currentState(), ref::ReferenceStatus::STATE_ACTIVE);
    EXPECT_TRUE(driver.reset(clock.now()).ok());
    EXPECT_EQ(driver.runtime().currentState(), ref::ReferenceStatus::STATE_SELF_CHECK);
    EXPECT_EQ(driver.runtime().evaluator(), nullptr);
    clock.run(0.1);
    EXPECT_EQ(driver.runtime().currentState(), ref::ReferenceStatus::STATE_READY);
    EXPECT_EQ(driver.runtime().activeTrajectoryId(), 0U);
}

TEST(ReferenceRuntime, ConfigurationIsClamped) {
    urt::ReferenceTrajectoryRuntime runtime;
    urt::ReferenceTrajectoryConfig config;
    config.status_rate_hz = -1.0;
    config.active_publish_rate_hz = std::nan("");
    config.validation_sample_dt = 0.0;
    config.trajectory_timeout = -3.0;
    config.min_lead_time = std::numeric_limits<double>::infinity();
    runtime.setConfig(config);
    EXPECT_DOUBLE_EQ(runtime.config().status_rate_hz, 10.0);
    EXPECT_DOUBLE_EQ(runtime.config().active_publish_rate_hz, 10.0);
    EXPECT_DOUBLE_EQ(runtime.config().validation_sample_dt, 0.02);
    EXPECT_DOUBLE_EQ(runtime.config().trajectory_timeout, 0.5);
    EXPECT_DOUBLE_EQ(runtime.config().min_lead_time, 0.2);
}

TEST(ReferenceDriver, DefaultAnalyticIsRequestedOnceWhenReadyAndAgainAfterReset) {
    urt::ReferenceTrajectoryDriver driver;
    urt::DefaultAnalyticReferenceConfig defaults;
    defaults.enabled = true;
    defaults.trajectory_id = 9U;
    defaults.revision = 4U;
    driver.configure(urt::ReferenceTrajectoryConfig{}, defaults);
    Clock clock(driver);
    clock.run(0.5);
    EXPECT_EQ(driver.runtime().currentState(), ref::ReferenceStatus::STATE_ACTIVE);
    EXPECT_EQ(driver.runtime().activeTrajectoryId(), 9U);
    EXPECT_EQ(driver.runtime().activeRevision(), 4U);

    // Requested once: a replacement stays in force.
    ASSERT_EQ(driver.acceptAnalytic(circle(2U), clock.now()), Request::kPosted);
    clock.run(0.5);
    EXPECT_EQ(driver.runtime().activeTrajectoryId(), 2U);

    driver.reset(clock.now());
    clock.run(0.5);
    EXPECT_EQ(driver.runtime().activeTrajectoryId(), 9U);
}

TEST(ReferenceDriver, DefaultAnalyticIsNotRequestedWhenDisabled) {
    urt::ReferenceTrajectoryDriver driver;
    Clock clock(driver);
    clock.run(1.0);
    EXPECT_EQ(driver.runtime().currentState(), ref::ReferenceStatus::STATE_READY);
}

TEST(ReferenceDriver, UpdateReportsTheDefaultRequestOnlyWhenItIsIssued) {
    urt::ReferenceTrajectoryDriver driver;
    urt::DefaultAnalyticReferenceConfig defaults;
    defaults.enabled = true;
    driver.configure(urt::ReferenceTrajectoryConfig{}, defaults);
    int issued = 0;
    for (int k = 0; k < 100; ++k) {
        const auto result = driver.update(kT0 + kDt * (k + 1));
        if (result.default_analytic) {
            EXPECT_EQ(*result.default_analytic, Request::kPosted);
            ++issued;
        }
    }
    EXPECT_EQ(issued, 1);
}

TEST(DefaultAnalytic, BuildsTheRequestFromTheConfiguration) {
    urt::DefaultAnalyticReferenceConfig config;
    config.enabled = true;
    config.request_id = 5U;
    config.trajectory_id = 6U;
    config.revision = 7U;
    config.analytic_type = 99U;  // not one of the four: becomes a circle
    config.start_delay = 1.5;
    config.duration = -1.0;  // not positive: 120 s
    config.origin_x = 1.0;
    config.origin_y = 2.0;
    config.origin_yaw = 1.0;
    config.radius = 4.0;
    config.line_speed = 0.7;
    config.entry_duration = 2.0;
    config.center_x = 0.25;
    config.center_y = -0.5;
    const auto msg = urt::makeDefaultAnalytic(config, 50.0);
    EXPECT_EQ(msg.request_id, 5U);
    EXPECT_EQ(msg.trajectory_id, 6U);
    EXPECT_EQ(msg.revision, 7U);
    EXPECT_EQ(msg.analytic_type, ref::AnalyticReference::ANALYTIC_CIRCLE);
    EXPECT_DOUBLE_EQ(msg.header.stamp.toSec(), 50.0);
    EXPECT_DOUBLE_EQ(msg.start_time.toSec(), 51.5);
    EXPECT_DOUBLE_EQ(msg.duration, 120.0);
    EXPECT_DOUBLE_EQ(msg.origin.position.x, 1.0);
    EXPECT_DOUBLE_EQ(msg.origin.orientation.z, std::sin(0.5));
    EXPECT_DOUBLE_EQ(msg.origin.orientation.w, std::cos(0.5));
    ASSERT_EQ(msg.params.size(), 5U);
    EXPECT_DOUBLE_EQ(msg.params[0], 4.0);
    EXPECT_DOUBLE_EQ(msg.params[4], -0.5);
}

// A configuration source over a map, the shape the ROS and JSON sources have.
struct MapSource {
    std::map<std::string, double> values;

    template <typename T>
    bool get(const std::string& key, T& value) const {
        const auto it = values.find(key);
        if (it == values.end()) {
            return false;
        }
        value = static_cast<T>(it->second);
        return true;
    }
};

TEST(ConfigLoader, MissingKeysKeepTheDefaultsAndPresentKeysOverrideThem) {
    urt::ReferenceTrajectoryConfig config;
    urt::DefaultAnalyticReferenceConfig defaults;
    MapSource source;
    source.values = {{"status_rate", 20.0},
                     {"max_velocity", 1.5},
                     {"default_analytic/enabled", 1.0},
                     {"default_analytic/analytic_type", 4.0},
                     {"default_analytic/request_id", -3.0},
                     {"default_analytic/radius", 5.5}};
    urt::loadReferenceConfig(source, config, defaults);
    EXPECT_DOUBLE_EQ(config.status_rate_hz, 20.0);
    EXPECT_DOUBLE_EQ(config.active_publish_rate_hz, 10.0);
    EXPECT_DOUBLE_EQ(config.limits.max_velocity, 1.5);
    EXPECT_DOUBLE_EQ(config.limits.max_yaw_rate, 0.0);
    EXPECT_TRUE(defaults.enabled);
    EXPECT_EQ(defaults.analytic_type, 4U);
    EXPECT_EQ(defaults.request_id, 0U);  // a negative id is clamped to zero
    EXPECT_EQ(defaults.trajectory_id, 1U);
    EXPECT_DOUBLE_EQ(defaults.radius, 5.5);
    EXPECT_DOUBLE_EQ(defaults.start_delay, 0.5);
}

}  // namespace

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
