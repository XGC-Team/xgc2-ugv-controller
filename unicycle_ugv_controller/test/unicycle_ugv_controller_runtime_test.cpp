#include <gtest/gtest.h>
#include <rigid_state_estimator_msgs/RigidStateEstimate.h>
#include <unicycle_reference_trajectory_msgs/AnalyticReference.h>
#include <unicycle_reference_trajectory_msgs/SampledReference.h>

#include <algorithm>
#include <limits>
#include <utility>

#include "unicycle_ugv_controller/common/reference_cache.h"
#include "unicycle_ugv_controller/common/rigid_to_unicycle.h"
#include "unicycle_ugv_controller/common/types.h"
#include "unicycle_ugv_controller/nmpc/unicycle_nmpc_solver.h"
#include "unicycle_ugv_controller/unicycle_ugv_controller.h"

namespace unicycle_ugv_controller {
namespace {

bool hasOutputEvent(UnicycleUgvController& controller, ::state_machine::EventId id) {
    const auto& events = controller.stateMachine().currentOutputEvents();
    return std::any_of(events.begin(), events.end(),
                       [id](const ::state_machine::Event& event) { return event.id == id; });
}

void makeTrackingReady(UnicycleUgvController& controller, UgvState& state) {
    state.received = true;
    state.stamp = ros::Time(1.0);
    state.estimator_state = rigid_state_estimator_msgs::RigidStateEstimate::STATE_RUNNING;
    state.estimator_flags = 0U;

    unicycle_reference_trajectory_msgs::AnalyticReference reference;
    reference.trajectory_id = 1U;
    reference.revision = 1U;
    reference.analytic_type = unicycle_reference_trajectory_msgs::AnalyticReference::ANALYTIC_HOLD;
    reference.start_time = ros::Time(1.0);
    reference.duration = 10.0;
    reference.origin.orientation.w = 1.0;
    ASSERT_TRUE(controller.referenceCache().updateAnalytic(reference, ros::Time(1.0)));

    controller.update(1.0);
    controller.update(1.01);
    ASSERT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::Ready);

    ::state_machine::Event tracking(event_type::TRACKING_REQUESTED,
                                    ::state_machine::EventTimestamp{1.01});
    tracking.source = "test";
    ASSERT_TRUE(controller.postEvent(std::move(tracking)).ok());
    controller.update(1.02);
    ASSERT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::Tracking);
}

TEST(UnicycleUgvControllerRuntime, StartsInSelfCheckWithoutStateEstimate) {
    ros::Time::init();
    UgvState state;
    UnicycleUgvController controller(state);
    controller.update(1.0);
    EXPECT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::SelfCheck);
}

TEST(UnicycleUgvControllerRuntime, RunningEstimatorMovesSelfCheckToReady) {
    ros::Time::init();
    UgvState state;
    state.received = true;
    state.stamp = ros::Time(1.0);
    state.estimator_state = rigid_state_estimator_msgs::RigidStateEstimate::STATE_RUNNING;
    state.estimator_flags = 0U;

    UnicycleUgvController controller(state);
    controller.update(1.0);
    controller.update(1.01);
    EXPECT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::Ready);
}

TEST(UnicycleUgvControllerRuntime, VrpnDirectDoesNotRequireEstimatorFlags) {
    ros::Time::init();
    UgvState state;
    state.received = true;
    state.stamp = ros::Time(1.0);
    state.x = 1.0;
    state.y = -2.0;
    state.yaw = 0.5;
    state.speed = 0.2;
    state.yaw_rate = -0.1;
    state.estimator_state = 0U;
    state.estimator_flags = rigid_state_estimator_msgs::RigidStateEstimate::FLAG_FAULT;

    UnicycleUgvController controller(state);
    auto config = controller.config();
    config.state_source = StateSource::VRPN_DIRECT;
    controller.setConfig(config);
    controller.update(1.0);
    controller.update(1.01);
    EXPECT_TRUE(controller.healthReady());
    EXPECT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::Ready);
}

TEST(UnicycleUgvControllerRuntime, StateFreshRejectsExcessivelyFutureStamp) {
    UgvState state;
    state.received = true;
    state.stamp = ros::Time(1.04);
    EXPECT_TRUE(stateFresh(state, ros::Time(1.0), 0.2));

    state.stamp = ros::Time(1.051);
    EXPECT_FALSE(stateFresh(state, ros::Time(1.0), 0.2));
}

TEST(UnicycleUgvControllerRuntime, VrpnQuaternionValidationRejectsZeroAndNonFiniteValues) {
    double yaw = 0.0;
    EXPECT_TRUE(tryYawFromQuaternion(0.0, 0.0, 0.0, 1.0, yaw));
    EXPECT_NEAR(yaw, 0.0, 1.0e-12);
    EXPECT_FALSE(tryYawFromQuaternion(0.0, 0.0, 0.0, 0.0, yaw));
    EXPECT_FALSE(
        tryYawFromQuaternion(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0, 1.0, yaw));
}

TEST(UnicycleUgvControllerRuntime, PlanarRunningValueIsNotEstimatorReady) {
    ros::Time::init();
    UgvState state;
    state.received = true;
    state.stamp = ros::Time(1.0);
    state.estimator_state = 2U;
    state.estimator_flags = 0U;

    UnicycleUgvController controller(state);
    controller.update(1.0);
    EXPECT_FALSE(controller.healthReady());
    EXPECT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::SelfCheck);

    state.estimator_state = rigid_state_estimator_msgs::RigidStateEstimate::STATE_RUNNING;
    controller.update(1.01);
    EXPECT_TRUE(controller.healthReady());
    EXPECT_EQ(rigid_state_estimator_msgs::RigidStateEstimate::STATE_RUNNING, 3U);
}

TEST(UnicycleUgvControllerRuntime, ProjectsRigidEstimateToForwardSpeed) {
    rigid_state_estimator_msgs::RigidStateEstimate msg;
    msg.position.x = 1.0;
    msg.position.y = -2.0;
    msg.orientation.w = 1.0;
    msg.velocity.x = 0.3;
    msg.velocity.y = 0.4;
    msg.angular_velocity.z = -0.1;
    const UnicycleProjection planar = projectRigidToUnicycle(msg);
    EXPECT_NEAR(planar.x, 1.0, 1.0e-12);
    EXPECT_NEAR(planar.y, -2.0, 1.0e-12);
    EXPECT_NEAR(planar.yaw, 0.0, 1.0e-12);
    EXPECT_NEAR(planar.speed, 0.3, 1.0e-12);
    EXPECT_NEAR(planar.yaw_rate, -0.1, 1.0e-12);
    EXPECT_TRUE(
        rigidEstimateHealthy(rigid_state_estimator_msgs::RigidStateEstimate::STATE_RUNNING, 0U));
    EXPECT_FALSE(rigidEstimateHealthy(2U, 0U));
}

TEST(UnicycleUgvControllerRuntime, FaultFlagReturnsToSelfCheck) {
    ros::Time::init();
    UgvState state;
    state.received = true;
    state.stamp = ros::Time(1.0);
    state.estimator_state = rigid_state_estimator_msgs::RigidStateEstimate::STATE_RUNNING;
    state.estimator_flags = 0U;

    UnicycleUgvController controller(state);
    controller.update(1.0);
    controller.update(1.01);
    ASSERT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::Ready);

    state.estimator_flags = rigid_state_estimator_msgs::RigidStateEstimate::FLAG_FAULT;
    controller.update(1.02);
    controller.update(1.03);
    EXPECT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::SelfCheck);
}

TEST(UnicycleUgvControllerRuntime, AutoStartTrackingWhenStateAndReferenceAreReady) {
    ros::Time::init();
    UgvState state;
    state.received = true;
    state.stamp = ros::Time(1.0);
    state.estimator_state = rigid_state_estimator_msgs::RigidStateEstimate::STATE_RUNNING;
    state.estimator_flags = 0U;

    UnicycleUgvController controller(state);
    auto config = controller.config();
    config.auto_start_tracking = true;
    controller.setConfig(config);

    unicycle_reference_trajectory_msgs::AnalyticReference reference;
    reference.trajectory_id = 1U;
    reference.revision = 1U;
    reference.analytic_type = unicycle_reference_trajectory_msgs::AnalyticReference::ANALYTIC_HOLD;
    reference.start_time = ros::Time(1.0);
    reference.duration = 10.0;
    reference.origin.orientation.w = 1.0;
    ASSERT_TRUE(controller.referenceCache().updateAnalytic(reference, ros::Time(1.01)));

    controller.update(1.0);
    controller.update(1.01);
    controller.update(1.02);
    EXPECT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::Tracking);
}

TEST(UnicycleUgvControllerRuntime, TransientNmpcFailureKeepsFreshCommand) {
    ros::Time::init();
    UgvState state;
    UnicycleUgvController controller(state);
    auto config = controller.config();
    config.result_timeout = 0.1;
    controller.setConfig(config);
    makeTrackingReady(controller, state);

    ControlCommand command;
    command.stamp = ros::Time(1.02);
    command.linear_speed = 0.8;
    command.angular_speed = -0.2;
    command.valid = true;
    controller.setCommand(command);

    ::state_machine::Event failure(event_type::INPUT_NMPC_SOLVE_FAILED,
                                   ::state_machine::EventTimestamp{1.05});
    failure.source = "test";
    failure.correlation_id = 1U;
    ASSERT_TRUE(controller.postEvent(std::move(failure)).ok());
    controller.update(1.05);

    EXPECT_TRUE(controller.commandReady());
    EXPECT_TRUE(controller.command().valid);
    EXPECT_FALSE(hasOutputEvent(controller, output_event_type::PUBLISH_ZERO_CMD_VEL));
}

TEST(UnicycleUgvControllerRuntime, SustainedNmpcFailureInvalidatesStaleCommandAndPublishesZero) {
    ros::Time::init();
    UgvState state;
    UnicycleUgvController controller(state);
    auto config = controller.config();
    config.result_timeout = 0.1;
    controller.setConfig(config);
    makeTrackingReady(controller, state);

    ControlCommand command;
    command.stamp = ros::Time(1.02);
    command.linear_speed = 0.8;
    command.angular_speed = -0.2;
    command.valid = true;
    controller.setCommand(command);

    ::state_machine::Event failure(event_type::INPUT_NMPC_SOLVE_FAILED,
                                   ::state_machine::EventTimestamp{1.13});
    failure.source = "test";
    failure.correlation_id = 1U;
    ASSERT_TRUE(controller.postEvent(std::move(failure)).ok());
    controller.update(1.13);

    EXPECT_FALSE(controller.commandReady());
    EXPECT_FALSE(controller.command().valid);
    EXPECT_TRUE(hasOutputEvent(controller, output_event_type::PUBLISH_ZERO_CMD_VEL));
}

TEST(UnicycleUgvControllerRuntime, SampledNmpcHorizonKeepsYawContinuousAcrossPi) {
    ros::Time::init();
    ReferenceCache cache;

    unicycle_reference_trajectory_msgs::AnalyticReference reference;
    reference.trajectory_id = 1U;
    reference.revision = 1U;
    reference.analytic_type =
        unicycle_reference_trajectory_msgs::AnalyticReference::ANALYTIC_CIRCLE;
    reference.start_time = ros::Time(0.0);
    reference.duration = 30.0;
    reference.origin.orientation.w = 1.0;
    reference.params = {3.0, 1.0, 0.0, 0.0, 0.0};
    ASSERT_TRUE(cache.updateAnalytic(reference, ros::Time(4.5)));

    std::vector<xgc2_math::control::Se2Reference> refs;
    ASSERT_TRUE(cache.sampleHorizon(ros::Time(4.5), 0.1, 10, 1.0, refs));
    ASSERT_EQ(refs.size(), 11U);
    for (size_t i = 1; i < refs.size(); ++i) {
        EXPECT_LT(std::abs(refs[i].state.yaw - refs[i - 1].state.yaw), 0.1);
        EXPECT_NEAR(refs[i].state.yaw - refs[i - 1].state.yaw, 1.0 / 30.0, 1.0e-6);
    }
}

TEST(UnicycleUgvControllerRuntime, ExplicitSampledPlanarKinematicsPreservesReverseSpeedBranch) {
    ros::Time::init();
    ReferenceCache cache;

    unicycle_reference_trajectory_msgs::SampledReference reference;
    reference.trajectory_id = 9U;
    reference.revision = 1U;
    reference.flags =
        unicycle_reference_trajectory_msgs::SampledReference::FLAG_EXPLICIT_PLANAR_KINEMATICS;
    reference.start_time = ros::Time(1.0);
    reference.sample_dt = 0.5;
    for (int index = 0; index < 3; ++index) {
        unicycle_reference_trajectory_msgs::PlanarReferencePoint point;
        point.t_from_start = 0.5 * static_cast<double>(index);
        point.x = -0.1 * static_cast<double>(index);
        point.y = 0.0;
        point.yaw = 0.0;
        point.speed = -0.2;
        point.vx = -0.2;
        point.vy = 0.0;
        reference.points.push_back(point);
    }
    ASSERT_TRUE(cache.updateSampled(reference, ros::Time(1.0)));

    std::vector<xgc2_math::control::Se2Reference> refs;
    ASSERT_TRUE(cache.sampleHorizon(ros::Time(1.0), 0.25, 4, 1.0, refs));
    ASSERT_EQ(refs.size(), 5U);
    for (const auto& ref : refs) {
        EXPECT_NEAR(ref.state.yaw, 0.0, 1.0e-12);
        EXPECT_NEAR(ref.state.linear_speed, -0.2, 1.0e-12);
    }
}

TEST(UnicycleUgvControllerRuntime, NmpcSolverUsesConfiguredPhysicalBounds) {
    UnicycleNmpcSolver solver;
    ASSERT_TRUE(solver.configureBounds(-1.5, 1.5, 2.0, 0.5235, 3.0));
    ASSERT_TRUE(solver.initialize());

    NmpcStateVector x0 = NmpcStateVector::Zero();
    std::vector<Se2Reference> refs(static_cast<size_t>(UnicycleNmpcSolver::horizonSteps() + 1));
    for (auto& ref : refs) {
        ref.state.position << 3.0, 3.0;
        ref.state.yaw = 1.5;
        ref.state.linear_speed = 3.0;
        ref.control.linear_acceleration = 2.0;
        ref.control.yaw_rate = 2.5;
    }

    ASSERT_TRUE(solver.solve(x0, refs));
    EXPECT_LE(std::abs(solver.predictedAngularSpeed()), 0.5235 + 1.0e-6);
    EXPECT_LE(std::abs(solver.optimalControl()(1)), 3.0 + 1.0e-6);
    for (size_t i = 1; i < solver.predictedStateCount(); ++i) {
        EXPECT_GE(solver.predictedStates()[i](3), -1.5 - 1.0e-6);
        EXPECT_LE(solver.predictedStates()[i](3), 1.5 + 1.0e-6);
    }
}

TEST(UnicycleUgvControllerRuntime, NmpcSolverRejectsInvalidBounds) {
    UnicycleNmpcSolver solver;
    EXPECT_FALSE(solver.configureBounds(1.5, 1.5, 2.0, 0.5235, 3.0));
    EXPECT_FALSE(solver.configureBounds(-1.5, 1.5, 0.0, 0.5235, 3.0));
    EXPECT_FALSE(solver.configureBounds(-1.5, 1.5, 2.0, 0.0, 3.0));
    EXPECT_FALSE(solver.configureBounds(-1.5, 1.5, 2.0, 0.5235, 0.0));
}

TEST(UnicycleUgvControllerRuntime, ResetCommandDrivesForwardAlongRail) {
    UgvState state;
    state.x = 0.0;
    state.y = 0.0;
    state.yaw = 0.0;
    state.speed = 0.0;
    state.yaw_rate = 0.0;
    ResetTarget goal;
    goal.x = 2.0;
    goal.y = 0.0;
    goal.yaw = 0.0;
    goal.valid = true;
    ControllerConfig config;
    const UnicycleResetOutput output = computeUnicycleResetCommand(state, goal, config);
    EXPECT_FALSE(output.position_ok);
    EXPECT_GT(output.linear_speed, 0.0);
    EXPECT_NEAR(output.angular_speed, 0.0, 1.0e-6);
}

TEST(UnicycleUgvControllerRuntime, ResetCommandPrefersReverseWhenFacingAway) {
    UgvState state;
    state.x = 0.0;
    state.y = 0.0;
    state.yaw = 0.0;
    state.speed = 0.0;
    state.yaw_rate = 0.0;
    ResetTarget goal;
    goal.x = -2.0;
    goal.y = 0.0;
    goal.yaw = 0.0;
    goal.valid = true;
    ControllerConfig config;
    const UnicycleResetOutput output = computeUnicycleResetCommand(state, goal, config);
    EXPECT_LT(output.linear_speed, 0.0);
}

TEST(UnicycleUgvControllerRuntime, ResetCommandSettlesWithLoosePhysicalTolerance) {
    UgvState state;
    state.x = 1.25;
    state.y = 0.10;
    state.yaw = 0.40;
    state.speed = 0.02;
    state.yaw_rate = 0.01;
    ResetTarget goal;
    goal.x = 1.0;
    goal.y = 0.0;
    goal.yaw = 0.0;
    goal.valid = true;
    ControllerConfig config;
    const UnicycleResetOutput output = computeUnicycleResetCommand(state, goal, config);
    EXPECT_TRUE(output.position_ok);
    EXPECT_TRUE(output.yaw_ok);
    EXPECT_TRUE(output.settled);
}

TEST(UnicycleUgvControllerRuntime, PlatformPoseReadyAcceptsCanonicalTopics) {
    ros::Time::init();
    UgvState state;
    state.received = true;
    state.stamp = ros::Time(1.0);
    state.x = 1.8;
    state.y = 0.0;
    state.yaw = 0.785;
    state.speed = 0.0;
    state.yaw_rate = 0.0;
    UnicycleUgvController controller(state);
    auto config = controller.config();
    config.state_source = StateSource::PLATFORM_POSE;
    controller.setConfig(config);
    controller.update(1.0);
    controller.update(1.01);
    EXPECT_TRUE(controller.healthReady());
    EXPECT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::Ready);
}

TEST(UnicycleUgvControllerRuntime, ResetCommandMovesReadyToResetThenTimeoutReturnsReady) {
    ros::Time::init();
    UgvState state;
    state.received = true;
    state.stamp = ros::Time(1.0);
    state.estimator_state = rigid_state_estimator_msgs::RigidStateEstimate::STATE_RUNNING;
    state.estimator_flags = 0U;
    state.x = 0.0;
    state.y = 0.0;
    state.yaw = 0.0;
    UnicycleUgvController controller(state);
    auto config = controller.config();
    config.reset_timeout = 0.05;
    config.placement_idle_silent = true;
    controller.setConfig(config);
    controller.update(1.0);
    controller.update(1.01);
    ASSERT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::Ready);

    ResetTarget goal;
    goal.x = 3.0;
    goal.y = 0.0;
    goal.yaw = 0.0;
    goal.valid = true;
    controller.setResetTarget(goal);

    ::state_machine::Event reset(event_type::RESET_REQUESTED,
                                 ::state_machine::EventTimestamp{1.02});
    reset.source = "test";
    ASSERT_TRUE(controller.postEvent(std::move(reset)).ok());
    controller.update(1.02);
    EXPECT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::Reset);

    state.stamp = ros::Time(1.10);
    controller.update(1.10);
    EXPECT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::Ready);
}

TEST(UnicycleUgvControllerRuntime, ResetArrivesWithLooseResidualThenReturnsReady) {
    ros::Time::init();
    UgvState state;
    state.received = true;
    state.stamp = ros::Time(1.0);
    state.estimator_state = rigid_state_estimator_msgs::RigidStateEstimate::STATE_RUNNING;
    state.estimator_flags = 0U;
    state.x = 0.05;
    state.y = -0.04;
    state.yaw = 0.10;
    state.speed = 0.0;
    state.yaw_rate = 0.0;
    UnicycleUgvController controller(state);
    auto config = controller.config();
    config.reset_settle_frames = 2;
    config.command_publish_rate_hz = 100.0;
    controller.setConfig(config);
    controller.update(1.0);
    controller.update(1.01);
    ASSERT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::Ready);

    ResetTarget goal;
    goal.x = 0.0;
    goal.y = 0.0;
    goal.yaw = 0.0;
    goal.valid = true;
    controller.setResetTarget(goal);
    ::state_machine::Event reset(event_type::RESET_REQUESTED,
                                 ::state_machine::EventTimestamp{1.02});
    reset.source = "test";
    ASSERT_TRUE(controller.postEvent(std::move(reset)).ok());
    controller.update(1.02);
    ASSERT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::Reset);
    state.stamp = ros::Time(1.03);
    controller.update(1.03);
    state.stamp = ros::Time(1.04);
    controller.update(1.04);
    EXPECT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::Ready);
}

TEST(UnicycleUgvControllerRuntime, NmpcSolverLimitsOneStageYawRateChange) {
    constexpr double kMaxAngularAcceleration = 0.6;
    constexpr double kStageDt = 0.1;
    UnicycleNmpcSolver solver;
    ASSERT_TRUE(solver.configureBounds(-1.5, 1.5, 2.0, 1.5, kMaxAngularAcceleration));
    ASSERT_TRUE(solver.initialize());

    NmpcStateVector x0 = NmpcStateVector::Zero();
    std::vector<Se2Reference> refs(static_cast<size_t>(UnicycleNmpcSolver::horizonSteps() + 1));
    for (auto& ref : refs) {
        ref.state.position << 0.0, 3.0;
        ref.state.yaw = 1.5707963267948966;
        ref.state.linear_speed = 1.0;
        ref.control.yaw_rate = 1.0;
    }

    ASSERT_TRUE(solver.solve(x0, refs));
    EXPECT_LE(std::abs(solver.optimalControl()(1)), kMaxAngularAcceleration + 1.0e-6);
    EXPECT_LE(std::abs(solver.predictedAngularSpeed() - x0(4)),
              kMaxAngularAcceleration * kStageDt + 1.0e-5);
}

TEST(UnicycleUgvControllerRuntime, NmpcSolverHeavierOmegaWeightDampsExistingRailYawRate) {
    const double rail_yaw = 1.5707963267948966;
    auto make_refs = [rail_yaw]() {
        std::vector<Se2Reference> refs(static_cast<size_t>(UnicycleNmpcSolver::horizonSteps() + 1));
        for (int i = 0; i <= UnicycleNmpcSolver::horizonSteps(); ++i) {
            refs[static_cast<size_t>(i)].state.position << -13.0, 0.05 * static_cast<double>(i);
            refs[static_cast<size_t>(i)].state.yaw = rail_yaw;
            refs[static_cast<size_t>(i)].state.linear_speed = 0.5;
        }
        return refs;
    };
    // On-rail, almost aligned, but still rotating: a heavier omega state cost
    // must damp the field-observed limit cycle more aggressively.
    NmpcStateVector x0;
    x0 << -12.97, 0.0, 1.610, 0.5, 0.5;

    NmpcCostWeights cheap;
    cheap.accel = 0.05;
    cheap.omega = 0.08;

    UnicycleNmpcSolver cheap_solver;
    ASSERT_TRUE(cheap_solver.configureBounds(-0.5, 0.5, 2.0, 0.8, 3.0));
    ASSERT_TRUE(cheap_solver.configureWeights(cheap));
    ASSERT_TRUE(cheap_solver.initialize());
    ASSERT_TRUE(cheap_solver.solve(x0, make_refs()));
    const double w_cheap = cheap_solver.predictedAngularSpeed();

    UnicycleNmpcSolver heavy_solver;
    ASSERT_TRUE(heavy_solver.configureBounds(-0.5, 0.5, 2.0, 0.8, 3.0));
    ASSERT_TRUE(heavy_solver.configureWeights(NmpcCostWeights{}));
    ASSERT_TRUE(heavy_solver.initialize());
    ASSERT_TRUE(heavy_solver.solve(x0, make_refs()));
    const double w_heavy = heavy_solver.predictedAngularSpeed();

    EXPECT_LT(std::abs(w_heavy), 0.35);
    EXPECT_LT(std::abs(w_heavy), std::abs(w_cheap));
}

TEST(UnicycleUgvControllerRuntime, NmpcSolverRejectsInvalidWeights) {
    UnicycleNmpcSolver solver;
    NmpcCostWeights weights;
    weights.omega = 0.0;
    EXPECT_FALSE(solver.configureWeights(weights));
    weights = NmpcCostWeights{};
    weights.angular_accel = 0.0;
    EXPECT_FALSE(solver.configureWeights(weights));
}

}  // namespace
}  // namespace unicycle_ugv_controller
