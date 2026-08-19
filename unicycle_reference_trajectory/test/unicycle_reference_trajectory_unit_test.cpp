#include <gtest/gtest.h>
#include <unicycle_reference_trajectory/shuttle_leg.h>

#include <cmath>
#include <vector>
#include <xgc2_math/trajectory.hpp>

namespace {
namespace trajectory = xgc2_math::trajectory;

TEST(UnicycleReferenceTrajectoryCore, CircleProvidesUnicycleReferences) {
    trajectory::CircleCurveParameters2 params;
    params.radius = 3.0;
    params.line_speed = 2.0;
    params.center = Eigen::Vector2d::Zero();

    trajectory::CircleCurveEvaluator2 evaluator(params);
    trajectory::PlanarReference2 output;
    ASSERT_TRUE(evaluator.evaluate(0.0, output));
    EXPECT_NEAR(output.position.x(), 3.0, 1e-12);
    EXPECT_NEAR(output.speed, 2.0, 1e-12);
    EXPECT_NEAR(output.yaw_rate, 2.0 / 3.0, 1e-12);
    EXPECT_TRUE(std::isfinite(output.curvature));
}

TEST(UnicycleReferenceTrajectoryCore, WaypointSolverProducesPlanarPolynomial) {
    trajectory::WaypointProblem2 problem;
    for (const auto& point :
         {Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(1.0, 1.0), Eigen::Vector2d(2.0, 0.0)}) {
        trajectory::WaypointConstraint2 constraint;
        constraint.position = point;
        problem.constraints.push_back(constraint);
    }
    problem.segment_times = {2.0, 2.0};
    problem.start_velocity = Eigen::Vector2d(0.2, 0.0);
    problem.end_velocity = Eigen::Vector2d(0.0, -0.2);

    trajectory::PiecewisePolynomialEvaluator2 evaluator;
    uint32_t flags = 0U;
    ASSERT_TRUE(trajectory::MincoWaypointSolver2().solve(problem, evaluator, &flags));
    EXPECT_EQ(evaluator.order(), 7U);
    EXPECT_EQ(evaluator.segments().size(), 2U);

    trajectory::PlanarReference2 start;
    trajectory::PlanarReference2 middle;
    trajectory::PlanarReference2 end;
    ASSERT_TRUE(evaluator.evaluate(0.0, start));
    ASSERT_TRUE(evaluator.evaluate(2.0, middle));
    ASSERT_TRUE(evaluator.evaluate(4.0, end));
    EXPECT_TRUE(start.position.isApprox(problem.constraints.front().position, 1e-9));
    EXPECT_TRUE(middle.position.isApprox(problem.constraints[1].position, 1e-9));
    EXPECT_TRUE(end.position.isApprox(problem.constraints.back().position, 1e-9));
    EXPECT_TRUE(start.velocity.isApprox(problem.start_velocity, 1e-9));
    EXPECT_TRUE(end.velocity.isApprox(problem.end_velocity, 1e-9));
}

TEST(UnicycleReferenceTrajectoryCore, ExplicitYawPreservesReverseSpeed) {
    trajectory::PolynomialSegment2 segment;
    segment.duration = 1.0;
    segment.x = {0.0, -1.0};
    segment.y = {0.0, 0.0};
    segment.yaw = {0.0, 0.0};

    trajectory::PiecewisePolynomialEvaluator2 evaluator;
    ASSERT_TRUE(evaluator.setSegments({segment}, 1U));
    trajectory::PlanarReference2 output;
    ASSERT_TRUE(evaluator.evaluate(0.5, output));
    EXPECT_NEAR(output.speed, -1.0, 1e-12);
    EXPECT_NEAR(output.yaw, 0.0, 1e-12);
}

TEST(ShuttleLeg, PlusYKeepsYawAndUsesForwardSpeed) {
    unicycle_reference_trajectory::ShuttleLegRequest request;
    request.start_y = -2.0;
    request.x_fixed = 1.25;
    request.y_goal = 2.0;
    request.desired_speed = 0.8;
    request.max_acceleration = 1.0;
    request.sample_dt = 0.05;
    request.hold_duration = 0.2;
    std::vector<unicycle_reference_trajectory::ShuttleSample> samples;
    ASSERT_TRUE(unicycle_reference_trajectory::buildShuttleLeg(request, samples));
    ASSERT_GT(samples.size(), 8U);
    EXPECT_NEAR(samples.front().y, -2.0, 1e-6);
    EXPECT_NEAR(samples.back().y, 2.0, 1e-6);
    EXPECT_NEAR(samples.back().speed, 0.0, 1e-9);
    double max_speed = 0.0;
    for (std::size_t i = 0; i < samples.size(); ++i) {
        EXPECT_NEAR(samples[i].x, 1.25, 1e-12);
        EXPECT_NEAR(samples[i].yaw, unicycle_reference_trajectory::shuttleYawAlongPlusY(), 1e-12);
        EXPECT_NEAR(samples[i].vx, 0.0, 1e-12);
        EXPECT_GE(samples[i].speed, -1e-9);
        max_speed = std::max(max_speed, samples[i].speed);
        if (i > 0) {
            EXPECT_GE(samples[i].y + 1e-9, samples[i - 1].y);
        }
    }
    EXPECT_GT(max_speed, 0.3);
}

TEST(ShuttleLeg, MinusYUsesReverseWithoutChangingYaw) {
    unicycle_reference_trajectory::ShuttleLegRequest request;
    request.start_y = 2.0;
    request.x_fixed = 0.0;
    request.y_goal = -2.0;
    request.desired_speed = 0.6;
    request.max_acceleration = 1.2;
    request.sample_dt = 0.05;
    request.hold_duration = 0.0;
    std::vector<unicycle_reference_trajectory::ShuttleSample> samples;
    ASSERT_TRUE(unicycle_reference_trajectory::buildShuttleLeg(request, samples));
    ASSERT_GT(samples.size(), 8U);
    double min_speed = 0.0;
    for (std::size_t i = 0; i < samples.size(); ++i) {
        EXPECT_NEAR(samples[i].yaw, unicycle_reference_trajectory::shuttleYawAlongPlusY(), 1e-12);
        EXPECT_LE(samples[i].speed, 1e-9);
        min_speed = std::min(min_speed, samples[i].speed);
    }
    EXPECT_LT(min_speed, -0.3);
    EXPECT_NEAR(samples.back().y, -2.0, 1e-6);
}

TEST(ShuttleLeg, FacingMinusYGoesForwardTowardMinusY) {
    unicycle_reference_trajectory::ShuttleLegRequest request;
    request.start_y = 2.0;
    request.x_fixed = 0.0;
    request.y_goal = -2.0;
    request.desired_speed = 0.6;
    request.max_acceleration = 1.2;
    request.sample_dt = 0.05;
    request.hold_duration = 0.0;
    request.rail_yaw = unicycle_reference_trajectory::shuttleYawAlongMinusY();
    std::vector<unicycle_reference_trajectory::ShuttleSample> samples;
    ASSERT_TRUE(unicycle_reference_trajectory::buildShuttleLeg(request, samples));
    ASSERT_GT(samples.size(), 8U);
    double max_speed = 0.0;
    for (std::size_t i = 0; i < samples.size(); ++i) {
        EXPECT_NEAR(samples[i].yaw, request.rail_yaw, 1e-12);
        EXPECT_GE(samples[i].speed, -1e-9);
        max_speed = std::max(max_speed, samples[i].speed);
    }
    EXPECT_GT(max_speed, 0.3);
    EXPECT_NEAR(samples.back().y, -2.0, 1e-6);
}

TEST(ShuttleLeg, NearestRailYawPicksShorterTurn) {
    const double plus = unicycle_reference_trajectory::shuttleYawAlongPlusY();
    const double minus = unicycle_reference_trajectory::shuttleYawAlongMinusY();
    EXPECT_NEAR(unicycle_reference_trajectory::shuttleNearestRailYaw(-1.62), minus, 1e-12);
    EXPECT_NEAR(unicycle_reference_trajectory::shuttleNearestRailYaw(1.20), plus, 1e-12);
    EXPECT_NEAR(unicycle_reference_trajectory::shuttleNearestRailYaw(0.0), plus, 1e-12);
}

TEST(ShuttleLeg, OnRailRequiresXAndRailAxisYaw) {
    const double rail = 1.0;
    const double plus = unicycle_reference_trajectory::shuttleYawAlongPlusY();
    const double minus = unicycle_reference_trajectory::shuttleYawAlongMinusY();
    EXPECT_TRUE(unicycle_reference_trajectory::shuttleOnRail(1.0, plus, rail, 0.25, 0.4));
    EXPECT_TRUE(unicycle_reference_trajectory::shuttleOnRail(1.0, minus, rail, 0.25, 0.4));
    EXPECT_FALSE(unicycle_reference_trajectory::shuttleOnRail(2.0, plus, rail, 0.25, 0.4));
    EXPECT_FALSE(unicycle_reference_trajectory::shuttleOnRail(1.0, 0.0, rail, 0.25, 0.4));
    EXPECT_TRUE(unicycle_reference_trajectory::shuttleOnRailX(1.1, rail, 0.25));
    EXPECT_FALSE(unicycle_reference_trajectory::shuttleOnRailX(2.0, rail, 0.25));
    EXPECT_NEAR(unicycle_reference_trajectory::shuttleHeadingTowardRail(2.0, 1.0),
                3.141592653589793, 1e-9);
    EXPECT_NEAR(unicycle_reference_trajectory::shuttleHeadingTowardRail(0.0, 1.0), 0.0, 1e-9);
}

TEST(ShuttleLeg, NextGoalTogglesRailEnds) {
    EXPECT_NEAR(unicycle_reference_trajectory::nextShuttleGoalY(0.0, -3.0, 3.0, false, 0.0), 3.0,
                1e-12);
    EXPECT_NEAR(unicycle_reference_trajectory::nextShuttleGoalY(2.0, -3.0, 3.0, true, 3.0), -3.0,
                1e-12);
}

TEST(ShuttleLeg, ReplanOnArrivalOrTimeoutOnly) {
    using unicycle_reference_trajectory::ShuttleReplanReason;
    EXPECT_EQ(unicycle_reference_trajectory::shuttleReplanReason(false, false, false),
              ShuttleReplanReason::FirstPlan);
    EXPECT_EQ(unicycle_reference_trajectory::shuttleReplanReason(true, true, true),
              ShuttleReplanReason::Arrived);
    EXPECT_EQ(unicycle_reference_trajectory::shuttleReplanReason(true, false, false),
              ShuttleReplanReason::TimedOut);
    EXPECT_EQ(unicycle_reference_trajectory::shuttleReplanReason(true, false, true),
              ShuttleReplanReason::Keep);
}

TEST(ShuttleLeg, ReplanKeepsSameEndUntilArrival) {
    EXPECT_NEAR(
        unicycle_reference_trajectory::shuttleGoalYForReplan(true, false, 0.0, -2.0, 2.0, 2.0), 2.0,
        1e-12);
    EXPECT_NEAR(
        unicycle_reference_trajectory::shuttleGoalYForReplan(true, true, 2.0, -2.0, 2.0, 2.0), -2.0,
        1e-12);
}

TEST(ShuttleLeg, ArrivalUsesPlanarHypotNotJustY) {
    const double tol = 0.35;
    // Field robots routinely sit ~20 cm off the setpoint; that must still count.
    EXPECT_TRUE(unicycle_reference_trajectory::shuttleArrived(1.20, 2.00, 1.0, 2.0, tol));
    // Same Y as the goal but still far off the rail in X must not flip ends.
    EXPECT_FALSE(unicycle_reference_trajectory::shuttleArrived(1.80, 2.0, 1.0, 2.0, tol));
    EXPECT_TRUE(unicycle_reference_trajectory::shuttleArrived(1.10, 2.10, 1.0, 2.0, tol));
}

TEST(ShuttleLeg, OffRailUsesFeasibleApproachOnRailUsesReverse) {
    const double rail = 1.0;
    const double plus_y = unicycle_reference_trajectory::shuttleYawAlongPlusY();
    EXPECT_EQ(unicycle_reference_trajectory::shuttleTrackMode(2.0, plus_y, rail, 0.25, 0.4),
              unicycle_reference_trajectory::ShuttleTrackMode::FeasibleApproach);
    EXPECT_EQ(unicycle_reference_trajectory::shuttleTrackMode(1.0, 0.0, rail, 0.25, 0.4),
              unicycle_reference_trajectory::ShuttleTrackMode::FeasibleApproach);
    EXPECT_EQ(unicycle_reference_trajectory::shuttleTrackMode(1.0, plus_y, rail, 0.25, 0.4),
              unicycle_reference_trajectory::ShuttleTrackMode::ReverseRail);
    EXPECT_EQ(unicycle_reference_trajectory::shuttleTrackMode(
                  1.0, unicycle_reference_trajectory::shuttleYawAlongMinusY(), rail, 0.25, 0.4),
              unicycle_reference_trajectory::ShuttleTrackMode::ReverseRail);
}

TEST(ShuttleLeg, EntryPoseClampsOntoRailSegment) {
    double ex = 0.0;
    double ey = 0.0;
    unicycle_reference_trajectory::shuttleEntryPose(-12.47, 7.60, -13.0, -5.0, 5.0, ex, ey);
    EXPECT_NEAR(ex, -13.0, 1e-12);
    EXPECT_NEAR(ey, 5.0, 1e-12);
    unicycle_reference_trajectory::shuttleEntryPose(-12.47, 1.2, -13.0, -5.0, 5.0, ex, ey);
    EXPECT_NEAR(ey, 1.2, 1e-12);
}

TEST(ShuttleLeg, ReverseMotionOnlyWhenOnRail) {
    using unicycle_reference_trajectory::ShuttleReplanReason;
    EXPECT_FALSE(unicycle_reference_trajectory::shuttleBeginReverseMotion(
        false, true, true, ShuttleReplanReason::TimedOut));
    EXPECT_FALSE(unicycle_reference_trajectory::shuttleBeginReverseMotion(
        false, true, false, ShuttleReplanReason::FirstPlan));
    EXPECT_TRUE(unicycle_reference_trajectory::shuttleBeginReverseMotion(
        true, true, false, ShuttleReplanReason::FirstPlan));
    EXPECT_TRUE(unicycle_reference_trajectory::shuttleBeginReverseMotion(
        true, true, false, ShuttleReplanReason::Arrived));
}

TEST(ShuttleLeg, CaptureRadiusIsThirtyMetresToTheRailSegment) {
    const double rail = -13.0;
    const double lo = -5.0;
    const double hi = 5.0;
    const double radius = unicycle_reference_trajectory::shuttleDefaultCaptureRadius();
    EXPECT_NEAR(radius, 30.0, 1e-12);
    EXPECT_NEAR(unicycle_reference_trajectory::shuttleDistanceToRail(-13.0, 0.0, rail, lo, hi), 0.0,
                1e-12);
    EXPECT_NEAR(unicycle_reference_trajectory::shuttleDistanceToRail(-43.0, 0.0, rail, lo, hi),
                30.0, 1e-12);
    EXPECT_TRUE(
        unicycle_reference_trajectory::shuttleWithinCapture(-43.0, 0.0, rail, lo, hi, radius));
    EXPECT_FALSE(
        unicycle_reference_trajectory::shuttleWithinCapture(-43.01, 0.0, rail, lo, hi, radius));
    // Past the +Y end of the corridor: distance is to the endpoint (-13, 5).
    EXPECT_NEAR(unicycle_reference_trajectory::shuttleDistanceToRail(-12.47, 7.60, rail, lo, hi),
                std::hypot(0.53, 2.60), 1e-9);
    EXPECT_TRUE(
        unicycle_reference_trajectory::shuttleWithinCapture(-12.47, 7.60, rail, lo, hi, radius));
}

TEST(ShuttleLeg, AnyPoseWithinThirtyMetresPlansOntoTheRail) {
    const double rail = -13.0;
    const double lo = -5.0;
    const double hi = 5.0;
    const double radius = 30.0;
    const double yaws[] = {
        0.0, 1.5707963267948966, 3.141592653589793, -1.5707963267948966, 0.05, 1.19, -1.56, 2.8};
    const double xs[] = {-43.0, -33.0, -23.0, -15.75, -13.5, -13.0, -12.47, -3.0, 7.0, 17.0};
    const double ys[] = {-35.0, -15.0, -5.0, 0.0, 1.2, 5.0, 7.49, 7.60, 15.0, 35.0};
    int planned = 0;
    int refused = 0;
    for (double x : xs) {
        for (double y : ys) {
            const bool inside =
                unicycle_reference_trajectory::shuttleWithinCapture(x, y, rail, lo, hi, radius);
            for (double yaw : yaws) {
                std::vector<unicycle_reference_trajectory::ShuttleSample> samples;
                double entry_x = 0.0;
                double entry_y = 0.0;
                const bool ok = unicycle_reference_trajectory::planShuttleEntry(
                    x, y, yaw, rail, lo, hi, 0.5, 1.0, 0.05, 0.1, radius, samples, entry_x,
                    entry_y);
                if (!inside) {
                    EXPECT_FALSE(ok) << "x=" << x << " y=" << y;
                    ++refused;
                    continue;
                }
                ASSERT_TRUE(ok) << "x=" << x << " y=" << y << " yaw=" << yaw;
                ASSERT_FALSE(samples.empty());
                EXPECT_NEAR(samples.front().x, x, 1e-6);
                EXPECT_NEAR(samples.front().y, y, 1e-6);
                EXPECT_NEAR(samples.back().x, rail, 1e-6);
                EXPECT_NEAR(samples.back().y, entry_y, 1e-6);
                EXPECT_NEAR(samples.back().yaw,
                            unicycle_reference_trajectory::shuttleNearestRailYaw(yaw), 1e-9);
                EXPECT_NEAR(samples.back().speed, 0.0, 1e-9);
                EXPECT_GT(samples.back().t, 0.0);
                EXPECT_LE(samples.back().t, 90.0);
                EXPECT_LE(std::hypot(entry_x - x, entry_y - y), radius + 1e-9);
                ++planned;
            }
        }
    }
    EXPECT_GE(planned, 200);
    EXPECT_GE(refused, 8);
}

TEST(ShuttleLeg, FieldOffRailPosesMeetEntryGates) {
    struct Case {
        double x;
        double y;
        double yaw;
        double entry_y;
    };
    const Case cases[] = {
        {-12.47, 7.60, 0.05, 5.0},
        {-15.75, 7.49, 1.19, 5.0},
        {-13.00, 1.20, 0.05, 1.2},
        {-13.56, 6.92, -1.62, 5.0},
    };
    for (const Case& c : cases) {
        std::vector<unicycle_reference_trajectory::ShuttleSample> samples;
        double entry_x = 0.0;
        double entry_y = 0.0;
        ASSERT_TRUE(unicycle_reference_trajectory::planShuttleEntry(c.x, c.y, c.yaw, -13.0, -5.0,
                                                                    5.0, 0.5, 1.0, 0.02, 0.1, 30.0,
                                                                    samples, entry_x, entry_y));
        EXPECT_NEAR(entry_x, -13.0, 1e-12);
        EXPECT_NEAR(entry_y, c.entry_y, 1e-12);
        EXPECT_NEAR(samples.back().x, -13.0, 1e-6);
        EXPECT_NEAR(samples.back().y, c.entry_y, 1e-6);
        EXPECT_NEAR(samples.back().yaw, unicycle_reference_trajectory::shuttleNearestRailYaw(c.yaw),
                    1e-9);
    }
}

TEST(UnicycleReferenceTrajectoryCore, HoldReportsLowSpeedSingularity) {
    trajectory::HoldCurveEvaluator2 evaluator;
    trajectory::PlanarReference2 output;
    ASSERT_TRUE(evaluator.evaluate(0.0, output));
    EXPECT_NE(output.flags & trajectory::kFlagLowSpeedSingularity, 0U);
}

}  // namespace

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
