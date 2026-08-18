#include <gtest/gtest.h>

#include <cmath>
#include <unicycle_reference_trajectory/shuttle_leg.h>
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

TEST(ShuttleLeg, OnRailRequiresXAndPlusYYaw) {
    const double rail = 1.0;
    const double yaw = unicycle_reference_trajectory::shuttleYawAlongPlusY();
    EXPECT_TRUE(unicycle_reference_trajectory::shuttleOnRail(1.0, yaw, rail, 0.25, 0.4));
    EXPECT_FALSE(unicycle_reference_trajectory::shuttleOnRail(2.0, yaw, rail, 0.25, 0.4));
    EXPECT_FALSE(unicycle_reference_trajectory::shuttleOnRail(1.0, 0.0, rail, 0.25, 0.4));
    EXPECT_TRUE(unicycle_reference_trajectory::shuttleOnRailX(1.1, rail, 0.25));
    EXPECT_FALSE(unicycle_reference_trajectory::shuttleOnRailX(2.0, rail, 0.25));
    EXPECT_NEAR(unicycle_reference_trajectory::shuttleHeadingTowardRail(2.0, 1.0), 3.141592653589793,
                1e-9);
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
    EXPECT_NEAR(unicycle_reference_trajectory::shuttleGoalYForReplan(true, false, 0.0, -2.0, 2.0,
                                                                     2.0),
                2.0, 1e-12);
    EXPECT_NEAR(unicycle_reference_trajectory::shuttleGoalYForReplan(true, true, 2.0, -2.0, 2.0, 2.0),
                -2.0, 1e-12);
}

TEST(ShuttleLeg, ArrivalUsesPlanarHypotNotJustY) {
    const double tol = 0.35;
    // Field robots routinely sit ~20 cm off the setpoint; that must still count.
    EXPECT_TRUE(unicycle_reference_trajectory::shuttleArrived(1.20, 2.00, 1.0, 2.0, tol));
    // Same Y as the goal but still far off the rail in X must not flip ends.
    EXPECT_FALSE(unicycle_reference_trajectory::shuttleArrived(1.80, 2.0, 1.0, 2.0, tol));
    EXPECT_TRUE(unicycle_reference_trajectory::shuttleArrived(1.10, 2.10, 1.0, 2.0, tol));
}

TEST(ShuttleLeg, OffRailOrBadYawUsesMincoOnRailUsesReverse) {
    const double rail = 1.0;
    const double plus_y = unicycle_reference_trajectory::shuttleYawAlongPlusY();
    EXPECT_EQ(unicycle_reference_trajectory::shuttleTrackMode(2.0, plus_y, rail, 0.25, 0.4),
              unicycle_reference_trajectory::ShuttleTrackMode::MincoApproach);
    EXPECT_EQ(unicycle_reference_trajectory::shuttleTrackMode(1.0, 0.0, rail, 0.25, 0.4),
              unicycle_reference_trajectory::ShuttleTrackMode::MincoApproach);
    EXPECT_EQ(unicycle_reference_trajectory::shuttleTrackMode(1.0, plus_y, rail, 0.25, 0.4),
              unicycle_reference_trajectory::ShuttleTrackMode::ReverseRail);
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
