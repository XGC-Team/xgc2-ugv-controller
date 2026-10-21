#include <gtest/gtest.h>
#include <rigid_state_estimator_msgs/RigidStateEstimate.h>
#include <unicycle_reference_trajectory_msgs/AnalyticReference.h>
#include <unicycle_reference_trajectory_msgs/SampledReference.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include "unicycle_ugv_controller/common/core_log.h"
#include "unicycle_ugv_controller/common/operator_command.h"
#include "unicycle_ugv_controller/common/reference_cache.h"
#include "unicycle_ugv_controller/common/rigid_to_unicycle.h"
#include "unicycle_ugv_controller/common/types.h"
#include "unicycle_ugv_controller/common/wall_clock.h"
#include "unicycle_ugv_controller/nmpc/unicycle_nmpc_solver.h"
#include "unicycle_ugv_controller/ros_reference_conversion.h"
#include "unicycle_ugv_controller/unicycle_ugv_controller.h"

namespace unicycle_ugv_controller {
namespace {

bool hasOutputEvent(UnicycleUgvController& controller, ::state_machine::EventId id) {
    const auto& events = controller.stateMachine().currentOutputEvents();
    return std::any_of(events.begin(), events.end(),
                       [id](const ::state_machine::Event& event) { return event.id == id; });
}

void setPose(UgvState& state, double t, double x, double y, double yaw) {
    state.received = true;
    state.pose_stamp_trusted = true;
    state.stamp = Time(t);
    state.x = x;
    state.y = y;
    state.yaw = yaw;
}

void postCommand(UnicycleUgvController& controller, ::state_machine::EventId id, double t) {
    ::state_machine::Event event(id, ::state_machine::EventTimestamp{t});
    event.source = "test";
    event.category = ::state_machine::EventCategory::kInput;
    const auto posted=controller.postEvent(std::move(event));
    if (id==event_type::CUSTOM1_REQUESTED && (controller.resetExecutor().permissionPending() || controller.resetExecutor().active()))
        EXPECT_EQ(posted.code,::state_machine::ErrorCode::kTransitionRejected);
    else ASSERT_TRUE(posted.ok());
}

void goReadyEstimator(UnicycleUgvController& controller, UgvState& state, double t) {
    state.received = true;
    state.pose_stamp_trusted = true;
    state.stamp = Time(t);
    state.estimator_state = rigid_state_estimator_msgs::RigidStateEstimate::STATE_RUNNING;
    state.estimator_flags = 0U;
    controller.update(t);
    controller.update(t + 0.002);
    ASSERT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::Ready);
}

void goReadyPose(UnicycleUgvController& controller, UgvState& state, double t) {
    auto config = controller.config();
    config.state_source = StateSource::PLATFORM_POSE;
    controller.setConfig(config);
    setPose(state, t, 0.0, 0.0, 0.0);
    controller.update(t);
    controller.update(t + 0.002);
    ASSERT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::Ready);
}

void makeCustom1Ready(UnicycleUgvController& controller, UgvState& state) {
    goReadyEstimator(controller, state, 1.0);
    unicycle_reference_trajectory_msgs::AnalyticReference reference;
    reference.trajectory_id = 1U;
    reference.revision = 1U;
    reference.analytic_type = unicycle_reference_trajectory_msgs::AnalyticReference::ANALYTIC_HOLD;
    reference.start_time = ros::Time(1.0);
    reference.duration = 10.0;
    reference.origin.orientation.w = 1.0;
    ASSERT_TRUE(controller.referenceCache().updateAnalytic(toCoreReference(reference)));
    postCommand(controller, event_type::CUSTOM1_REQUESTED, 1.01);
    state.stamp = Time(1.01);
    controller.update(1.01);
    ASSERT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::Custom1);
}

void stepUnicyclePlant(UgvState& state, const ControlCommand& command, double dt) {
    const double v = command.valid ? command.linear_speed : 0.0;
    const double omega = command.valid ? command.angular_speed : 0.0;
    state.x += v * std::cos(state.yaw) * dt;
    state.y += v * std::sin(state.yaw) * dt;
    state.yaw = wrapAngle(state.yaw + omega * dt);
    state.stamp = Time(state.stamp.toSec() + dt);
}

TEST(UnicycleSm, StartsInSelfCheckAndPublishesZero) {
    ros::Time::init();
    UgvState state;
    UnicycleUgvController controller(state);
    controller.update(1.0);
    EXPECT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::SelfCheck);
    EXPECT_TRUE(hasOutputEvent(controller, output_event_type::PUBLISH_ZERO_CMD_VEL));
}

TEST(UnicycleSm, RunningEstimatorMovesSelfCheckToReady) {
    ros::Time::init();
    UgvState state;
    UnicycleUgvController controller(state);
    goReadyEstimator(controller, state, 1.0);
}

TEST(UnicycleSm, CanonicalPoseDoesNotNeedEstimatorOrTwist) {
    ros::Time::init();
    UgvState state;
    UnicycleUgvController controller(state);
    goReadyPose(controller, state, 1.0);
    EXPECT_TRUE(controller.healthReady());
}

TEST(UnicycleSm, BriefPoseDropoutKeepsReady) {
    ros::Time::init();
    UgvState state;
    UnicycleUgvController controller(state);
    goReadyPose(controller, state, 1.0);
    controller.update(1.3);
    EXPECT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::Ready);
}

TEST(UnicycleSm, PoseTimeoutHalfSecondGoesSelfCheck) {
    ros::Time::init();
    UgvState state;
    UnicycleUgvController controller(state);
    goReadyPose(controller, state, 1.0);
    controller.update(1.51);
    EXPECT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::SelfCheck);
}

TEST(UnicycleSm, FaultFlagReturnsToSelfCheck) {
    ros::Time::init();
    UgvState state;
    UnicycleUgvController controller(state);
    goReadyEstimator(controller, state, 1.0);
    state.estimator_flags = rigid_state_estimator_msgs::RigidStateEstimate::FLAG_FAULT;
    controller.update(1.02);
    EXPECT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::SelfCheck);
}

TEST(UnicycleSm, OutsideFenceGoesSelfCheck) {
    ros::Time::init();
    UgvState state;
    UnicycleUgvController controller(state);
    goReadyPose(controller, state, 1.0);
    setPose(state, 1.05, 50.0, 0.0, 0.0);
    controller.update(1.05);
    EXPECT_FALSE(controller.healthReady());
    EXPECT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::SelfCheck);
}

TEST(UnicycleSm, ResetStopReturnsReady) {
    ros::Time::init();
    UgvState state;
    UnicycleUgvController controller(state);
    goReadyPose(controller, state, 1.0);
    ResetTarget goal;
    goal.x = 3.0;
    goal.valid = true;
    controller.setResetTarget(goal);
    postCommand(controller, event_type::RESET_REQUESTED, 1.01);
    setPose(state, 1.01, 0.0, 0.0, 0.0);
    controller.update(1.01);
    ASSERT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::Reset);
    postCommand(controller, event_type::STOP_REQUESTED, 1.02);
    setPose(state, 1.02, 0.1, 0.0, 0.0);
    controller.update(1.02);
    EXPECT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::Ready);
}

TEST(UnicycleSm, ResetDoesNotJumpToCustom1) {
    ros::Time::init();
    UgvState state;
    UnicycleUgvController controller(state);
    goReadyPose(controller, state, 1.0);
    ResetTarget goal;
    goal.x = 3.0;
    goal.valid = true;
    controller.setResetTarget(goal);
    postCommand(controller, event_type::RESET_REQUESTED, 1.01);
    setPose(state, 1.01, 0.0, 0.0, 0.0);
    controller.update(1.01);
    ASSERT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::Reset);
    postCommand(controller, event_type::CUSTOM1_REQUESTED, 1.02);
    setPose(state, 1.02, 0.0, 0.0, 0.0);
    controller.update(1.02);
    EXPECT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::Reset);
}

TEST(UnicycleSm, ResetTimeoutReturnsReady) {
    ros::Time::init();
    UgvState state;
    UnicycleUgvController controller(state);
    auto config = controller.config();
    config.state_source = StateSource::PLATFORM_POSE;
    config.reset_timeout = 0.05;
    controller.setConfig(config);
    goReadyPose(controller, state, 1.0);
    ResetTarget goal;
    goal.x = 4.0;
    goal.valid = true;
    controller.setResetTarget(goal);
    postCommand(controller, event_type::RESET_REQUESTED, 1.01);
    setPose(state, 1.01, 0.0, 0.0, 0.0);
    controller.update(1.01);
    ASSERT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::Reset);
    setPose(state, 1.08, 0.0, 0.0, 0.0);
    controller.update(1.08);
    EXPECT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::Ready);
}

TEST(UnicycleSm, Custom1StopReturnsReady) {
    ros::Time::init();
    UgvState state;
    UnicycleUgvController controller(state);
    makeCustom1Ready(controller, state);
    postCommand(controller, event_type::STOP_REQUESTED, 1.02);
    state.stamp = Time(1.02);
    controller.update(1.02);
    EXPECT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::Ready);
}

TEST(UnicycleSm, Custom1CanReset) {
    ros::Time::init();
    UgvState state;
    UnicycleUgvController controller(state);
    makeCustom1Ready(controller, state);
    ResetTarget goal;
    goal.x = 1.0;
    goal.valid = true;
    controller.setResetTarget(goal);
    postCommand(controller, event_type::RESET_REQUESTED, 1.02);
    state.stamp = Time(1.02);
    controller.update(1.02);
    EXPECT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::Reset);
}

TEST(UnicycleSm, SelfCheckDoesNotJumpToCustom1OrReset) {
    ros::Time::init();
    UgvState state;
    UnicycleUgvController controller(state);
    controller.update(1.0);
    ASSERT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::SelfCheck);
    postCommand(controller, event_type::CUSTOM1_REQUESTED, 1.01);
    controller.update(1.01);
    EXPECT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::SelfCheck);
    postCommand(controller, event_type::RESET_REQUESTED, 1.02);
    controller.update(1.02);
    EXPECT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::SelfCheck);
    EXPECT_NE(controller.lastResetAdmissionMiss().find("RESET_REQUESTED unmatched"),
              std::string::npos);
    EXPECT_NE(controller.lastResetAdmissionMiss().find("CONTROL=SelfCheck"), std::string::npos);
    EXPECT_NE(controller.lastResetAdmissionMiss().find("unhealthy-SelfCheck-has-no-Reset-edge"),
              std::string::npos);
    std::cout << controller.lastResetAdmissionMiss() << std::endl;
}

TEST(UnicycleSm, ResetWithoutTargetStaysResetAndLogs) {
    ros::Time::init();
    UgvState state;
    UnicycleUgvController controller(state);
    goReadyPose(controller, state, 1.0);
    EXPECT_FALSE(controller.resetTargetReady());
    postCommand(controller, event_type::RESET_REQUESTED, 1.01);
    setPose(state, 1.01, 0.0, 0.0, 0.0);
    controller.update(1.01);
    EXPECT_TRUE(hasOutputEvent(controller, output_event_type::PUBLISH_ZERO_CMD_VEL));
    controller.update(1.012);
    EXPECT_FALSE(hasOutputEvent(controller, output_event_type::PUBLISH_ZERO_CMD_VEL));
    controller.update(1.02);
    EXPECT_FALSE(hasOutputEvent(controller, output_event_type::PUBLISH_ZERO_CMD_VEL));
    EXPECT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::Reset);
    EXPECT_TRUE(controller.resetExecutor().active());
    EXPECT_NE(controller.lastResetHoldReason().find("no target"), std::string::npos);
    std::cout << controller.lastResetHoldReason() << std::endl;
}

TEST(UnicycleSm, HoldStateIdIsGone) {
    EXPECT_NE(state_type::Custom1, 4U);
    EXPECT_EQ(state_type::Custom1, 3U);
    EXPECT_EQ(state_type::Reset, 5U);
}

TEST(UnicycleLaw, FilterRejectsNonPositiveDt) {
    PoseVelocityFilter filter;
    ControllerConfig config;
    EXPECT_FALSE(updatePoseVelocityFilter(filter, 1.0, 0.0, config.filter_wn, config.filter_zeta,
                                          config.velocity_dt_min, config.velocity_dt_max));
    EXPECT_FALSE(updatePoseVelocityFilter(filter, 1.0, 1.0e-6, config.filter_wn, config.filter_zeta,
                                          config.velocity_dt_min, config.velocity_dt_max));
}

TEST(UnicycleLaw, FilterUsesThisSampleDtNotFixedPeriod) {
    ControllerConfig config;
    PoseVelocityFilter fast;
    ASSERT_FALSE(updatePoseVelocityFilter(fast, 0.0, 0.01, config.filter_wn, config.filter_zeta,
                                          config.velocity_dt_min, config.velocity_dt_max));
    ASSERT_TRUE(updatePoseVelocityFilter(fast, 0.1, 0.01, config.filter_wn, config.filter_zeta,
                                         config.velocity_dt_min, config.velocity_dt_max));
    PoseVelocityFilter slow;
    updatePoseVelocityFilter(slow, 0.0, 0.05, config.filter_wn, config.filter_zeta,
                             config.velocity_dt_min, config.velocity_dt_max);
    updatePoseVelocityFilter(slow, 0.1, 0.05, config.filter_wn, config.filter_zeta,
                             config.velocity_dt_min, config.velocity_dt_max);
    EXPECT_GT(std::fabs(fast.x2 - slow.x2), 1.0e-6);
}

TEST(UnicycleLaw, ChassisBoxIsSeventyPercent) {
    double v = 2.0;
    double w = -2.0;
    boxSaturateUnicycle(v, w, 1.05, 1.05);
    EXPECT_NEAR(v, 1.05, 1.0e-12);
    EXPECT_NEAR(w, -1.05, 1.0e-12);
}

TEST(UnicycleLaw, FlatnessUsesWorldVelocityPd) {
    UgvState state;
    state.x = 0.0;
    state.y = 0.0;
    state.yaw = 0.0;
    state.vx = 0.0;
    state.vy = 0.0;
    state.speed = 0.2;
    state.velocity_valid = true;
    WorldPvaReference reference;
    reference.valid = true;
    reference.x = 1.0;
    reference.vx = 0.5;
    ControllerConfig config;
    const FlatnessCommandOutput output =
        computeFlatnessCommand(state, reference, 0.2, 0.02, config);
    ASSERT_TRUE(output.valid);
    EXPECT_GT(output.accel, 0.0);
    EXPECT_GT(output.linear_speed, 0.2);
}

TEST(UnicycleLaw, FlatnessYawIsContinuousAcrossZeroSpeed) {
    UgvState state;
    state.velocity_valid = true;
    WorldPvaReference reference;
    reference.valid = true;
    reference.y = 0.01;
    reference.ay = 0.2;
    ControllerConfig config;
    for (double speed : {-1e-6, 0.0, 1e-6}) {
        const auto output = computeFlatnessCommand(state, reference, speed, 0.004, config);
        ASSERT_TRUE(output.valid);
        EXPECT_LT(std::fabs(output.angular_speed), 1e-4);
    }
}

TEST(UnicycleSm, FlatnessRetainsCommandWhenClockDoesNotAdvance) {
    ros::Time::init();
    UgvState state;
    UnicycleUgvController controller(state);
    auto cfg = controller.config();
    cfg.tracking_strategy = TrackingStrategy::FLATNESS;
    controller.setConfig(cfg);
    goReadyPose(controller, state, 1.0);
    setPose(state, 1.02, 0.0, 0.0, 0.0);
    controller.update(1.02);
    postCommand(controller, event_type::CUSTOM1_REQUESTED, 1.02);
    controller.update(1.02);
    WorldPvaReference reference;
    reference.stamp = Time(1.02);
    reference.x = 0.1;
    reference.vx = 0.3;
    reference.valid = true;
    controller.setWorldPva(reference);
    setPose(state, 1.04, 0.0, 0.0, 0.0);
    controller.update(1.04);
    ASSERT_TRUE(controller.command().valid);
    const double speed = controller.command().linear_speed;
    ASSERT_GT(speed, 0.0);
    controller.update(1.04);
    EXPECT_FALSE(hasOutputEvent(controller, output_event_type::PUBLISH_ZERO_CMD_VEL));
    EXPECT_TRUE(controller.command().valid);
    EXPECT_DOUBLE_EQ(controller.command().linear_speed, speed);
}

TEST(UnicycleLaw, FlatnessRejectsInvalidDt) {
    UgvState state;
    state.velocity_valid = true;
    WorldPvaReference reference;
    reference.valid = true;
    reference.x = 1.0;
    EXPECT_FALSE(computeFlatnessCommand(state, reference, 0.2, 0.0, ControllerConfig{}).valid);
}

TEST(UnicycleSm, MissingCustom1ReferencePublishesIdleZeroAtFiveHz) {
    ros::Time::init();
    for (const auto strategy : {TrackingStrategy::FLATNESS, TrackingStrategy::NMPC}) {
        UgvState state;
        UnicycleUgvController controller(state);
        auto cfg = controller.config();
        cfg.tracking_strategy = strategy;
        controller.setConfig(cfg);
        goReadyPose(controller, state, 1.0);
        postCommand(controller, event_type::CUSTOM1_REQUESTED, 1.01);
        controller.update(1.01);
        ASSERT_EQ(controller.stateMachine().currentState(region_type::CONTROL),
                  state_type::Custom1);

        int zero_count = 0;
        for (int tick = 0; tick < 1000; ++tick) {
            const double now = 1.012 + tick * 0.002;
            setPose(state, now, 0.0, 0.0, 0.0);
            controller.update(now);
            zero_count += hasOutputEvent(controller, output_event_type::PUBLISH_ZERO_CMD_VEL);
            ASSERT_FALSE(hasOutputEvent(controller, output_event_type::PUBLISH_CMD_VEL));
        }
        EXPECT_GE(zero_count, 9);
        EXPECT_LE(zero_count, 10);
    }
}

TEST(UnicycleSm, FlatnessPublishesThirtyHzAndStopsImmediatelyWhenReferenceIsLost) {
    ros::Time::init();
    UgvState state;
    UnicycleUgvController controller(state);
    auto cfg = controller.config();
    cfg.tracking_strategy = TrackingStrategy::FLATNESS;
    controller.setConfig(cfg);
    goReadyPose(controller, state, 1.0);
    postCommand(controller, event_type::CUSTOM1_REQUESTED, 1.01);
    controller.update(1.01);
    controller.update(1.012);
    ASSERT_FALSE(controller.command().valid);

    WorldPvaReference reference;
    reference.valid = true;
    reference.x = 0.1;
    reference.vx = 0.3;
    controller.setWorldPva(reference);
    setPose(state, 1.014, 0.0, 0.0, 0.0);
    controller.update(1.014);
    ASSERT_TRUE(hasOutputEvent(controller, output_event_type::PUBLISH_CMD_VEL));
    ASSERT_GT(controller.command().linear_speed, 0.0);

    // A stop must bypass the recent idle and driving publication deadlines.
    controller.setWorldPva(WorldPvaReference{});
    controller.update(1.016);
    EXPECT_TRUE(hasOutputEvent(controller, output_event_type::PUBLISH_ZERO_CMD_VEL));
    EXPECT_FALSE(controller.command().valid);
    int zero_count = 0;
    for (int tick = 0; tick < 100; ++tick) {
        const double now = 1.018 + tick * 0.002;
        setPose(state, now, 0.0, 0.0, 0.0);
        controller.update(now);
        zero_count += hasOutputEvent(controller, output_event_type::PUBLISH_ZERO_CMD_VEL);
    }
    EXPECT_LE(zero_count, 1);

    // Recovery uses the same output cadence as an uninterrupted healthy run.
    controller.setWorldPva(reference);
    // The first computation after a long gap re-establishes a valid dt.
    setPose(state, 1.217, 0.0, 0.0, 0.0);
    controller.update(1.217);
    int command_count = 0;
    for (int tick = 0; tick < 1000; ++tick) {
        const double now = 1.218 + tick * 0.002;
        setPose(state, now, 0.0, 0.0, 0.0);
        controller.update(now);
        command_count += hasOutputEvent(controller, output_event_type::PUBLISH_CMD_VEL);
        ASSERT_FALSE(hasOutputEvent(controller, output_event_type::PUBLISH_ZERO_CMD_VEL));
    }
    EXPECT_GE(command_count, 59);
    EXPECT_LE(command_count, 61);
    postCommand(controller, event_type::STOP_REQUESTED, 3.217);
    controller.update(3.217);
    EXPECT_TRUE(hasOutputEvent(controller, output_event_type::PUBLISH_ZERO_CMD_VEL));
    EXPECT_FALSE(controller.command().valid);
}

TEST(UnicycleSm, RepeatedNmpcFailuresKeepIdlePublicationCadence) {
    ros::Time::init();
    UgvState state;
    UnicycleUgvController controller(state);
    makeCustom1Ready(controller, state);
    uint64_t pending_sequence = 0U;
    int zero_count = 0;
    int failure_count = 0;
    for (int tick = 0; tick < 1000; ++tick) {
        const double now = 1.012 + tick * 0.002;
        state.stamp = Time(now);
        if (pending_sequence != 0U) {
            ::state_machine::Event failure(event_type::INPUT_NMPC_SOLVE_FAILED,
                                           ::state_machine::EventTimestamp{now});
            failure.correlation_id = pending_sequence;
            ASSERT_TRUE(controller.postEvent(std::move(failure)).ok());
            pending_sequence = 0U;
            ++failure_count;
        }
        controller.update(now);
        for (const auto& event : controller.stateMachine().currentOutputEvents()) {
            if (event.id == output_event_type::REQUEST_NMPC_SOLVE) {
                pending_sequence = event.correlation_id;
            }
            zero_count += event.id == output_event_type::PUBLISH_ZERO_CMD_VEL;
            ASSERT_NE(event.id, output_event_type::PUBLISH_CMD_VEL);
        }
    }
    EXPECT_GT(failure_count, 100);
    EXPECT_GE(zero_count, 9);
    EXPECT_LE(zero_count, 10);
}

TEST(UnicycleLaw, PvaDoesNotValidateAlgorithmTimestamp) {
    WorldPvaReference reference;
    reference.valid = true;
    reference.stamp = Time(1.1);
    reference.x = 2.0;
    reference.vx = 0.5;
    ASSERT_TRUE(worldPvaReady(reference));
    const WorldPvaReference held = liftWorldPva(reference, 1.0);
    ASSERT_TRUE(held.valid);
    EXPECT_DOUBLE_EQ(held.x, reference.x);
    EXPECT_DOUBLE_EQ(held.vx, reference.vx);

    reference.stamp = Time(1001.0);
    EXPECT_TRUE(worldPvaReady(reference));
    EXPECT_TRUE(liftWorldPva(reference, 1.0).valid);
}

TEST(UnicycleSm, Custom1EntryDropsTheReferenceReceivedBeforeTheHold) {
    ros::Time::init();
    UgvState state;
    UnicycleUgvController controller(state);
    auto cfg = controller.config();
    cfg.tracking_strategy = TrackingStrategy::FLATNESS;
    controller.setConfig(cfg);
    goReadyPose(controller, state, 1.0);
    postCommand(controller, event_type::CUSTOM1_REQUESTED, 1.01);
    controller.update(1.01);
    ASSERT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::Custom1);

    WorldPvaReference reference;
    reference.valid = true;
    reference.stamp = Time(1.02);
    reference.x = 0.1;
    reference.vx = 1.0;
    controller.setWorldPva(reference);
    setPose(state, 1.03, 0.0, 0.0, 0.0);
    controller.update(1.03);
    ASSERT_TRUE(controller.worldPvaReady());

    // Pause, then resume: the reference from before the hold must not be tracked.
    postCommand(controller, event_type::STOP_REQUESTED, 1.04);
    controller.update(1.04);
    ASSERT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::Ready);
    postCommand(controller, event_type::CUSTOM1_REQUESTED, 1.05);
    setPose(state, 1.05, 0.0, 0.0, 0.0);
    controller.update(1.05);
    ASSERT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::Custom1);
    EXPECT_FALSE(controller.worldPvaReady());
    EXPECT_FALSE(controller.worldPva().valid);

    bool commanded = false;
    for (int tick = 0; tick < 50; ++tick) {
        const double now = 1.052 + tick * 0.002;
        setPose(state, now, 0.0, 0.0, 0.0);
        controller.update(now);
        commanded |= controller.command().valid;
    }
    EXPECT_FALSE(commanded);

    // A reference published after the entry is tracked.
    reference.stamp = Time(1.16);
    controller.setWorldPva(reference);
    for (int tick = 0; tick < 50; ++tick) {
        const double now = 1.16 + tick * 0.002;
        setPose(state, now, 0.0, 0.0, 0.0);
        controller.update(now);
        commanded |= controller.command().valid;
    }
    EXPECT_TRUE(commanded);
}

TEST(UnicycleUgvControllerRuntime, ProductHealthTimeoutIsHalfSecond) {
    EXPECT_DOUBLE_EQ(ControllerConfig{}.state_timeout, 0.5);
}

TEST(UnicycleUgvControllerRuntime, StateFreshRejectsExcessivelyFutureStamp) {
    UgvState state;
    state.received = true;
    state.pose_stamp_trusted = true;
    state.stamp = Time(1.04);
    EXPECT_TRUE(stateFresh(state, Time(1.0), 0.2));
    state.stamp = Time(1.051);
    EXPECT_FALSE(stateFresh(state, Time(1.0), 0.2));
}

TEST(UnicycleUgvControllerRuntime, VrpnQuaternionValidationRejectsZeroAndNonFiniteValues) {
    double yaw = 0.0;
    EXPECT_TRUE(tryYawFromQuaternion(0.0, 0.0, 0.0, 1.0, yaw));
    EXPECT_NEAR(yaw, 0.0, 1.0e-12);
    EXPECT_FALSE(tryYawFromQuaternion(0.0, 0.0, 0.0, 0.0, yaw));
    EXPECT_FALSE(
        tryYawFromQuaternion(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0, 1.0, yaw));
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
}

TEST(UnicycleUgvControllerRuntime, AutoStartCustom1WhenStateAndReferenceAreReady) {
    ros::Time::init();
    UgvState state;
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
    ASSERT_TRUE(controller.referenceCache().updateAnalytic(toCoreReference(reference)));
    state.received = true;
    state.pose_stamp_trusted = true;
    state.stamp = Time(1.0);
    state.estimator_state = rigid_state_estimator_msgs::RigidStateEstimate::STATE_RUNNING;
    controller.update(1.0);
    controller.update(1.01);
    controller.update(1.02);
    EXPECT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::Custom1);
}

TEST(UnicycleUgvControllerRuntime, TransientNmpcFailureKeepsFreshCommand) {
    ros::Time::init();
    UgvState state;
    UnicycleUgvController controller(state);
    auto config = controller.config();
    config.result_timeout = 0.1;
    controller.setConfig(config);
    makeCustom1Ready(controller, state);
    ControlCommand command;
    command.stamp = Time(1.02);
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
    makeCustom1Ready(controller, state);
    ControlCommand command;
    command.stamp = Time(1.02);
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
    ASSERT_TRUE(cache.updateAnalytic(toCoreReference(reference)));
    std::vector<xgc2_math::control::Se2Reference> refs;
    ASSERT_TRUE(cache.sampleHorizon(Time(4.5), 0.1, 10, refs));
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
    ASSERT_TRUE(cache.updateSampled(toCoreReference(reference)));
    std::vector<xgc2_math::control::Se2Reference> refs;
    ASSERT_TRUE(cache.sampleHorizon(Time(1.0), 0.25, 4, refs));
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
}

TEST(UnicycleUgvControllerRuntime, NmpcSolverRejectsInvalidBounds) {
    UnicycleNmpcSolver solver;
    EXPECT_FALSE(solver.configureBounds(1.5, 1.5, 2.0, 0.5235, 3.0));
    EXPECT_FALSE(solver.configureBounds(-1.5, 1.5, 0.0, 0.5235, 3.0));
    EXPECT_FALSE(solver.configureBounds(-1.5, 1.5, 2.0, 0.0, 3.0));
    EXPECT_FALSE(solver.configureBounds(-1.5, 1.5, 2.0, 0.5235, 0.0));
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

namespace {

// A coordinator grant for the controller's current session: the route from `start` through `via`
// to `goal`, the prefix `s_limit` of it, valid for one second after the controller receives it.
uint32_t nextResetPublication() {
    static uint32_t sequence=0;
    return sequence++;
}

ugv_reset_client::Grant goFor(const UnicycleUgvController& controller, const ugv_reset_client::Pose& start,
                              const std::vector<ugv_reset_client::Point>& via,
                              const ugv_reset_client::Pose& goal, double s_limit, uint32_t id = 1U) {
    ugv_reset_client::Grant g;
    g.session = controller.resetExecutor().sessionId();
    g.request_generation = controller.resetExecutor().requestGeneration();
    g.publication_seq = nextResetPublication();
    g.state = ugv_reset_client::GrantState::Go;
    g.grant_id = 100U + id;
    g.start = start;
    g.via = via;
    g.goal = goal;
    g.s_limit = s_limit;
    g.ttl = 1.0;
    return g;
}

ugv_reset_client::Grant rejectFor(const UnicycleUgvController& controller, ugv_reset_client::Reason reason,
                                  const std::string& detail) {
    ugv_reset_client::Grant g;
    g.session = controller.resetExecutor().sessionId();
    g.request_generation = controller.resetExecutor().requestGeneration();
    g.publication_seq = nextResetPublication();
    g.state = ugv_reset_client::GrantState::Reject;
    g.grant_id = 200U;
    g.reason = reason;
    g.detail = detail;
    return g;
}

void enterReset(UnicycleUgvController& controller, UgvState& state, double goal_x) {
    goReadyPose(controller, state, 0.5);
    ResetTarget goal;
    goal.x = goal_x;
    goal.valid = true;
    controller.setResetTarget(goal);
    postCommand(controller, event_type::RESET_REQUESTED, 0.51);
    setPose(state, 0.51, 0.0, 0.0, 0.0);
    controller.update(0.51);
    controller.update(0.512);
    // This fixture is the station/output edge: publish an exact all-axis zero and
    // collect fresh acquisition-stamped poses before offering any Go.
    ugv_reset_client::Grant brake;
    brake.session=controller.resetExecutor().sessionId();
    brake.request_generation=controller.resetExecutor().requestGeneration();
    brake.state=ugv_reset_client::GrantState::Brake;brake.grant_id=1;
    brake.publication_seq=nextResetPublication();controller.onResetGrant(brake);
    controller.notifyResetZeroPublished(.514,brake.request_generation,brake.grant_id);
    setPose(state,.52,0,0,0);controller.update(.52);
    setPose(state,.74,0,0,0);controller.update(.74);
    setPose(state,1.01,0,0,0);controller.update(1.012);
    ASSERT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::Reset);
}

// Every Reset outcome is on the log with its reason (R0): Ready alone does not say how it ended.
std::vector<std::string>& capturedLog() {
    static std::vector<std::string> lines;
    return lines;
}
void captureSink(LogLevel, const char* message) {
    capturedLog().emplace_back(message);
}
bool logged(const std::string& needle) {
    return std::any_of(capturedLog().begin(), capturedLog().end(),
                       [&](const std::string& line) { return line.find(needle) != std::string::npos; });
}

}  // namespace

TEST(UnicycleSm, ResetBeginsASessionThatDeclaresTheVehicleToTheCoordinator) {
    ros::Time::init();
    UgvState state;
    UnicycleUgvController controller(state);
    enterReset(controller, state, 1.0);
    EXPECT_DOUBLE_EQ(controller.command().linear_speed, 0.0);
    ASSERT_TRUE(controller.resetExecutor().active());
    ugv_reset_client::Session session;
    ASSERT_TRUE(controller.takeResetSession(&session));
    EXPECT_EQ(session.phase, ugv_reset_client::Phase::Requesting);
    EXPECT_DOUBLE_EQ(session.target.x, 1.0);
    EXPECT_EQ(session.spec.model, ugv_reset_client::Model::Unicycle);
    EXPECT_DOUBLE_EQ(session.spec.footprint.half_length, 0.45);  // 0.90 x 0.80 m
    EXPECT_DOUBLE_EQ(session.spec.footprint.half_width, 0.40);
    EXPECT_DOUBLE_EQ(session.spec.max_v, 0.35);
    EXPECT_DOUBLE_EQ(session.spec.ell_bound, 0.25);
    EXPECT_TRUE(session.healthy);
    controller.resetSessionPublished();
    EXPECT_FALSE(controller.takeResetSession(&session)) << "the next one is due in 0.2 s";
}

TEST(UnicycleSm, AGrantedRouteDrivesTheTwistAndAStopEndsTheSessionWithOneResult) {
    ros::Time::init();
    UgvState state;
    UnicycleUgvController controller(state);
    enterReset(controller, state, 1.0);
    const ugv_reset_client::Pose here{0.0, 0.0, 0.0};
    controller.onResetGrant(goFor(controller, here, {}, {1.0, 0.0, 0.0}, 1e9));
    // The tracker steps at 50 Hz of the controller's clock.
    for (int i = 0; i < 20; ++i) {
        setPose(state, 1.02 + 0.02 * i, 0.0, 0.0, 0.0);
        controller.update(1.02 + 0.02 * i);
    }
    EXPECT_GT(controller.command().linear_speed, 0.05);
    EXPECT_GT(controller.cmdVel().linear_x, 0.05);
    EXPECT_LE(controller.cmdVel().linear_x, 0.35 + 1e-9);
    EXPECT_NEAR(controller.cmdVel().angular_z, 0.0, 1e-9);
    ugv_reset_client::Session session;
    ASSERT_TRUE(controller.takeResetSession(&session));
    EXPECT_NE(session.phase, ugv_reset_client::Phase::Requesting);
    postCommand(controller, event_type::STOP_REQUESTED, 1.5);
    setPose(state, 1.5, 0.1, 0.0, 0.0);
    controller.update(1.5);
    EXPECT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::Ready);
    EXPECT_FALSE(controller.resetExecutor().active());
    ugv_reset_client::Result result;
    ASSERT_TRUE(controller.takeResetResult(&result));
    EXPECT_EQ(result.outcome, ugv_reset_client::Outcome::Stopped);
    EXPECT_FALSE(controller.takeResetResult(&result)) << "one result per session";
    // A grant that arrives after the session ended never moves the vehicle.
    controller.onResetGrant(goFor(controller, here, {}, {1.0, 0.0, 0.0}, 1e9, 2U));
    controller.update(1.51);
    EXPECT_DOUBLE_EQ(controller.command().linear_speed, 0.0);
    EXPECT_DOUBLE_EQ(controller.cmdVel().linear_x, 0.0);
}

TEST(UnicycleSm, AGrantOfAnotherSessionNeverMovesTheVehicle) {
    ros::Time::init();
    UgvState state;
    UnicycleUgvController controller(state);
    enterReset(controller, state, 1.0);
    ugv_reset_client::Grant stale = goFor(controller, {0, 0, 0}, {}, {1.0, 0.0, 0.0}, 1e9);
    stale.session += 1U;
    controller.onResetGrant(stale);
    for (int i = 0; i < 20; ++i) {
        setPose(state, 1.02 + 0.02 * i, 0.0, 0.0, 0.0);
        controller.update(1.02 + 0.02 * i);
    }
    EXPECT_DOUBLE_EQ(controller.command().linear_speed, 0.0);
}

TEST(UnicycleSm, ArrivalAtTheTargetEndsResetThroughTheCoordinatorsRoute) {
    ros::Time::init();
    UgvState state;
    UnicycleUgvController controller(state);
    enterReset(controller, state, 0.0);
    controller.onResetGrant(goFor(controller, {0, 0, 0}, {}, {0.0, 0.0, 0.0}, 1e9));
    // Standing on the target: the gate (5 cm, 5 deg, measured stopped, held 0.4 s) closes.
    double t = 1.02;
    for (int i = 0; i < 100 && controller.stateMachine().currentState(region_type::CONTROL) ==
                                   state_type::Reset;
         ++i, t += 0.02) {
        setPose(state, t, 0.0, 0.0, 0.0);
        controller.update(t);
        if (i % 10 == 0) {
            controller.onResetGrant(goFor(controller, {0, 0, 0}, {}, {0.0, 0.0, 0.0}, 1e9));
        }
    }
    EXPECT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::Ready);
    ugv_reset_client::Result result;
    ASSERT_TRUE(controller.takeResetResult(&result));
    EXPECT_EQ(result.outcome, ugv_reset_client::Outcome::Arrived);
    EXPECT_LE(result.final_planar_error, 0.05);
}

TEST(UnicycleSm, ARejectionByTheCoordinatorReturnsReadyAndAFreshRetryGetsANewSession) {
    ros::Time::init();
    UgvState state;
    UnicycleUgvController controller(state);
    enterReset(controller, state, 1.0);
    const uint32_t first = controller.resetExecutor().sessionId();
    controller.onResetGrant(rejectFor(controller, ugv_reset_client::Reason::GoalOccupied,
                                      "the target is occupied by ugv3"));
    setPose(state, 1.014, 0.0, 0.0, 0.0);
    controller.update(1.014);
    controller.update(1.016);
    ASSERT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::Ready);
    EXPECT_TRUE(controller.healthReady());
    ugv_reset_client::Result result;
    ASSERT_TRUE(controller.takeResetResult(&result));
    EXPECT_EQ(result.outcome, ugv_reset_client::Outcome::Infeasible);
    EXPECT_EQ(result.reason, ugv_reset_client::Reason::GoalOccupied);
    EXPECT_DOUBLE_EQ(controller.command().linear_speed, 0.0);
    // Health remains ready throughout: a retry needs no new HEALTH_READY edge.
    setPose(state, 1.018, 0.0, 0.0, 0.0);
    controller.update(1.018);
    postCommand(controller, event_type::RESET_REQUESTED, 1.02);
    setPose(state, 1.02, 0.0, 0.0, 0.0);
    controller.update(1.02);
    controller.update(1.022);
    ASSERT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::Reset);
    EXPECT_GT(controller.resetExecutor().sessionId(), first);
    // The earlier session's rejection does not end the retry.
    ugv_reset_client::Grant old = rejectFor(controller, ugv_reset_client::Reason::GoalOccupied, "old");
    old.session = first;
    controller.onResetGrant(old);
    controller.update(1.023);
    EXPECT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::Reset);
}

TEST(UnicycleSm, ALostVehicleStateStandsStillAndEndsTheSessionUnhealthy) {
    ros::Time::init();
    UgvState state;
    UnicycleUgvController controller(state);
    enterReset(controller, state, 1.0);
    controller.onResetGrant(goFor(controller, {0, 0, 0}, {}, {1.0, 0.0, 0.0}, 1e9));
    for (int i = 0; i < 10; ++i) {
        setPose(state, 1.02 + 0.02 * i, 0.0, 0.0, 0.0);
        controller.update(1.02 + 0.02 * i);
    }
    ASSERT_GT(controller.command().linear_speed, 0.0);
    // The pose stops arriving: the health monitor takes the controller to SelfCheck at once.
    controller.update(2.0);
    controller.update(2.002);
    EXPECT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::SelfCheck);
    ugv_reset_client::Result result;
    ASSERT_TRUE(controller.takeResetResult(&result));
    EXPECT_EQ(result.outcome, ugv_reset_client::Outcome::AbortedUnhealthy);
    EXPECT_DOUBLE_EQ(controller.cmdVel().linear_x, 0.0);
}

TEST(UnicycleSm, ResetOutcomesAreLoggedWithTheirReason) {
    ros::Time::init();
    setLogSink(&captureSink);
    {
        capturedLog().clear();
        UgvState state;
        UnicycleUgvController controller(state);
        enterReset(controller, state, 1.0);
        controller.onResetGrant(rejectFor(controller, ugv_reset_client::Reason::GoalOccupied,
                                          "target occupied by ugv3"));
        controller.update(1.014);
        controller.update(1.016);
        EXPECT_TRUE(logged("Reset ended after")) << "no rejection line";
        EXPECT_TRUE(logged("INFEASIBLE")) << "the typed outcome is dropped";
        EXPECT_TRUE(logged("target occupied by ugv3")) << "the coordinator's reason is dropped";
    }
    {
        capturedLog().clear();
        UgvState state;
        UnicycleUgvController controller(state);
        enterReset(controller, state, 0.0);
        controller.onResetGrant(goFor(controller, {0, 0, 0}, {}, {0.0, 0.0, 0.0}, 1e9));
        double t = 1.02;
        for (int i = 0; i < 100; ++i, t += 0.02) {
            setPose(state, t, 0.0, 0.0, 0.0);
            controller.update(t);
        }
        EXPECT_TRUE(logged("Reset arrived after"));
    }
    {
        capturedLog().clear();
        UgvState state;
        UnicycleUgvController controller(state);
        enterReset(controller, state, 1.0);
        postCommand(controller, event_type::STOP_REQUESTED, 1.016);
        controller.update(1.016);
        EXPECT_TRUE(logged("Reset ended after"));
        EXPECT_TRUE(logged("stopped"));
        EXPECT_FALSE(logged("Reset ended after") && logged("INFEASIBLE"));
    }
    {
        capturedLog().clear();
        UgvState state;
        UnicycleUgvController controller(state);
        enterReset(controller, state, 1.0);
        auto config = controller.config();
        config.reset_timeout = 0.5;
        controller.setConfig(config);
        setPose(state, 2.0, 0.0, 0.0, 0.0);
        controller.update(2.0);
        controller.update(2.002);
        EXPECT_TRUE(logged("Reset timed out after"));
    }
    {
        // What the coordinator says while it holds the vehicle is logged once per change.
        capturedLog().clear();
        UgvState state;
        UnicycleUgvController controller(state);
        enterReset(controller, state, 1.0);
        ugv_reset_client::Grant hold;
        hold.session = controller.resetExecutor().sessionId();
        hold.request_generation=controller.resetExecutor().requestGeneration();
        hold.grant_id=300U;hold.publication_seq=nextResetPublication();
        hold.state = ugv_reset_client::GrantState::Hold;
        hold.reason = ugv_reset_client::Reason::WaitingForPeer;
        hold.detail = "ugv2 passes first";
        hold.blockers = {"ugv2"};
        controller.onResetGrant(hold);
        controller.update(1.014);
        hold.publication_seq=nextResetPublication();
        controller.onResetGrant(hold);
        controller.update(1.016);
        int lines = 0;
        for (const auto& line : capturedLog()) {
            lines += line.find("held by the coordinator") != std::string::npos ? 1 : 0;
        }
        EXPECT_EQ(lines, 1);
        EXPECT_TRUE(logged("ugv2 passes first"));
    }
    setLogSink(nullptr);
}

TEST(UnicycleSm, CmdVelSaturatesByMode) {
    ros::Time::init();
    UgvState state;
    UnicycleUgvController controller(state);
    makeCustom1Ready(controller, state);
    auto config = controller.config();
    config.max_linear_speed = 3.0;
    config.min_linear_speed = -1.5;
    config.max_angular_speed = 2.5;
    config.chassis_max_linear_speed = 1.05;
    config.chassis_max_yaw_rate = 1.05;
    controller.setConfig(config);
    ControlCommand command;
    command.valid = true;
    command.stamp = Time(1.01);
    command.linear_speed = 5.0;
    command.angular_speed = -9.0;
    controller.setCommand(command);
    // NMPC tracking is limited by the solver bounds, not by the chassis limits.
    EXPECT_DOUBLE_EQ(controller.cmdVel().linear_x, 3.0);
    EXPECT_DOUBLE_EQ(controller.cmdVel().angular_z, -2.5);
    command.linear_speed = -5.0;
    controller.setCommand(command);
    EXPECT_DOUBLE_EQ(controller.cmdVel().linear_x, -1.5);
    command.valid = false;
    controller.setCommand(command);
    EXPECT_DOUBLE_EQ(controller.cmdVel().linear_x, 0.0);
    EXPECT_DOUBLE_EQ(controller.cmdVel().angular_z, 0.0);
    command.valid = true;
    command.linear_speed = std::nan("");
    controller.setCommand(command);
    EXPECT_DOUBLE_EQ(controller.cmdVel().linear_x, 0.0);

    // Flatness tracking and everything else outside Custom1 stay within the chassis.
    config.tracking_strategy = TrackingStrategy::FLATNESS;
    controller.setConfig(config);
    command.linear_speed = 5.0;
    command.angular_speed = -9.0;
    controller.setCommand(command);
    EXPECT_DOUBLE_EQ(controller.cmdVel().linear_x, 1.05);
    EXPECT_DOUBLE_EQ(controller.cmdVel().angular_z, -1.05);
}

TEST(UnicycleCommands, TheOperatorTextIsReadCaseInsensitively) {
    for (const char* text : {"track", "Tracking", "CUSTOM", "custom1", "Start"}) {
        EXPECT_EQ(parseOperatorCommand(text), OperatorCommand::kCustom1) << text;
    }
    for (const char* text : {"hold", "STOP", "Stop"}) {
        EXPECT_EQ(parseOperatorCommand(text), OperatorCommand::kStop) << text;
    }
    for (const char* text : {"reset", "Reset", "RESET"}) {
        EXPECT_EQ(parseOperatorCommand(text), OperatorCommand::kReset) << text;
    }
    for (const char* text : {"", " reset", "resets", "go", "custom2"}) {
        EXPECT_EQ(parseOperatorCommand(text), OperatorCommand::kUnknown) << text;
    }
}


TEST(UnicycleSm, NewResetReplacesActiveSessionWithoutRetirementOrOutcomeOverwrite) {
    UgvState state;UnicycleUgvController controller(state);enterReset(controller,state,1.0);
    const auto old_session=controller.resetExecutor().sessionId();
    const auto old_generation=controller.resetExecutor().requestGeneration();
    const auto old_go=goFor(controller,{0,0,0},{},{1,0,0},1e9);
    ASSERT_TRUE(controller.postEvent(::state_machine::Event(event_type::RESET_REQUESTED,
                    ::state_machine::EventTimestamp{1.02})).ok());
    setPose(state,1.02,0,0,0);controller.update(1.02);
    EXPECT_EQ(controller.stateMachine().currentState(region_type::CONTROL),state_type::Reset);
    EXPECT_NE(controller.resetExecutor().sessionId(),old_session);
    EXPECT_GT(controller.resetExecutor().requestGeneration(),old_generation);
    controller.onResetGrant(old_go);EXPECT_FALSE(controller.resetExecutor().tracker().active());
    EXPECT_TRUE(controller.resetExecutor().command().zero());
    ugv_reset_client::Result result;ASSERT_TRUE(controller.peekResetResult(&result));
    EXPECT_EQ(result.session,old_session);EXPECT_EQ(result.outcome,ugv_reset_client::Outcome::Stopped);
    EXPECT_EQ(controller.postEvent(::state_machine::Event(event_type::CUSTOM1_REQUESTED,
                    ::state_machine::EventTimestamp{1.03})).code,::state_machine::ErrorCode::kTransitionRejected);
    EXPECT_EQ(controller.postEvent(::state_machine::Event(event_type::RESET_REQUESTED,
                    ::state_machine::EventTimestamp{1.03})).code,::state_machine::ErrorCode::kLimitReached);
    ASSERT_TRUE(controller.peekResetResult(&result));EXPECT_EQ(result.session,old_session);
}

}  // namespace unicycle_ugv_controller
