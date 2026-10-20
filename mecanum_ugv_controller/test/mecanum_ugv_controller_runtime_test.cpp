#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include "mecanum_ugv_controller/common/core_log.h"
#include "mecanum_ugv_controller/common/types.h"
#include "mecanum_ugv_controller/common/wall_clock.h"
#include "mecanum_ugv_controller/mecanum_ugv_controller.h"

namespace mecanum_ugv_controller {
namespace {

bool hasOutputEvent(MecanumUgvController& controller, ::state_machine::EventId id) {
    const auto& events = controller.stateMachine().currentOutputEvents();
    return std::any_of(events.begin(), events.end(),
                       [id](const ::state_machine::Event& event) { return event.id == id; });
}

void setPose(UgvState& state, double t, double x, double y, double yaw) {
    state.received = true;
    state.stamp = Time(t);
    state.x = x;
    state.y = y;
    state.yaw = yaw;
}

void postCommand(MecanumUgvController& controller, ::state_machine::EventId id, double t) {
    ::state_machine::Event event(id, ::state_machine::EventTimestamp{t});
    event.source = "test";
    event.category = ::state_machine::EventCategory::kInput;
    ASSERT_TRUE(controller.postEvent(std::move(event)).ok());
}

void goReady(MecanumUgvController& controller, UgvState& state, double t) {
    setPose(state, t, 0.0, 0.0, 0.0);
    controller.update(t);
    controller.update(t + 0.002);
    ASSERT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::Ready);
}

// FLU holonomic kinematics: body vx, vy, ω. No wheel or motor dynamics.
void plantWorldVelocity(const UgvState& state, const ControlCommand& command, double& vwx,
                        double& vwy) {
    const double c = std::cos(state.yaw);
    const double s = std::sin(state.yaw);
    const double vx = command.valid ? command.linear_x : 0.0;
    const double vy = command.valid ? command.linear_y : 0.0;
    vwx = c * vx - s * vy;
    vwy = s * vx + c * vy;
}

void stepHolonomicPlant(UgvState& state, const ControlCommand& command, double dt) {
    const double c = std::cos(state.yaw);
    const double s = std::sin(state.yaw);
    const double vx = command.valid ? command.linear_x : 0.0;
    const double vy = command.valid ? command.linear_y : 0.0;
    const double omega = command.valid ? command.angular_z : 0.0;
    const double vwx = c * vx - s * vy;
    const double vwy = s * vx + c * vy;
    state.x += vwx * dt;
    state.y += vwy * dt;
    state.yaw = wrapAngle(state.yaw + omega * dt);
    state.stamp = Time(state.stamp.toSec() + dt);
}

void enterCustom1(MecanumUgvController& controller, UgvState& state,
                  WorldVelocityReference& reference, double t) {
    goReady(controller, state, t);
    controller.setWorldReference(reference);
    postCommand(controller, event_type::CUSTOM1_REQUESTED, t + 0.01);
    setPose(state, t + 0.01, state.x, state.y, state.yaw);
    controller.update(t + 0.01);
    ASSERT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::Custom1);
}

}  // namespace

TEST(MecanumLaw, ProductHealthTimeoutIsHalfSecond) {
    EXPECT_DOUBLE_EQ(ControllerConfig{}.state_timeout, 0.5);
}

TEST(MecanumLaw, BoxSaturateClampsEachFluAxis) {
    UgvState state;
    WorldVelocityReference reference;
    reference.valid = true;
    reference.vx = 3.0;
    reference.vy = -3.0;
    ControllerConfig config;
    const HolonomicTrackOutput output = computeHolonomicTrackCommand(state, reference, config);
    EXPECT_NEAR(output.linear_x, 1.0, 1.0e-9);
    EXPECT_NEAR(output.linear_y, -1.0, 1.0e-9);
}

TEST(MecanumLaw, TrackRotatesWorldVelocityWithoutAssumingYawZero) {
    UgvState state;
    state.yaw = 1.5707963267948966;
    WorldVelocityReference reference;
    reference.valid = true;
    reference.vx = 1.0;
    const HolonomicTrackOutput output =
        computeHolonomicTrackCommand(state, reference, ControllerConfig{});
    EXPECT_NEAR(output.linear_x, 0.0, 1.0e-6);
    EXPECT_NEAR(output.linear_y, -1.0, 1.0e-6);
}

TEST(MecanumLaw, HeadingPUsesShortestAngleAcrossPi) {
    EXPECT_NEAR(headingRateToTarget(3.1, -3.1, 1.0, 1.0), wrapAngle(-3.1 - 3.1), 1.0e-6);
}

TEST(MecanumSm, SelfCheckPublishesFiveHertzZero) {
    UgvState state;
    MecanumUgvController controller(state);
    controller.update(1.0);
    EXPECT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::SelfCheck);
    EXPECT_TRUE(hasOutputEvent(controller, output_event_type::PUBLISH_ZERO_CMD_VEL));
}

TEST(MecanumSm, FreshPoseInsideFenceMovesSelfCheckToReady) {
    UgvState state;
    MecanumUgvController controller(state);
    goReady(controller, state, 1.0);
}

TEST(MecanumSm, MissingPoseStaysSelfCheck) {
    UgvState state;
    MecanumUgvController controller(state);
    controller.update(1.0);
    controller.update(2.0);
    EXPECT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::SelfCheck);
}

TEST(MecanumSm, BriefPoseDropoutKeepsReady) {
    UgvState state;
    MecanumUgvController controller(state);
    goReady(controller, state, 1.0);
    controller.update(1.3);
    EXPECT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::Ready);
}

TEST(MecanumSm, PoseTimeoutHalfSecondGoesSelfCheck) {
    UgvState state;
    MecanumUgvController controller(state);
    goReady(controller, state, 1.0);
    controller.update(1.51);
    EXPECT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::SelfCheck);
}

TEST(MecanumSm, Custom1BriefPoseDropoutStaysTracking) {
    UgvState state;
    MecanumUgvController controller(state);
    WorldVelocityReference reference;
    reference.valid = true;
    reference.vx = 0.2;
    reference.stamp = Time(1.0);
    enterCustom1(controller, state, reference, 1.0);
    controller.update(1.31);
    EXPECT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::Custom1);
}

TEST(MecanumSm, Custom1PoseTimeoutGoesSelfCheck) {
    UgvState state;
    MecanumUgvController controller(state);
    WorldVelocityReference reference;
    reference.valid = true;
    reference.vx = 0.2;
    reference.stamp = Time(1.0);
    enterCustom1(controller, state, reference, 1.0);
    controller.update(1.52);
    EXPECT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::SelfCheck);
}

TEST(MecanumSm, NonFinitePoseIsDirty) {
    UgvState state;
    MecanumUgvController controller(state);
    goReady(controller, state, 1.0);
    setPose(state, 1.05, std::nan(""), 0.0, 0.0);
    controller.update(1.05);
    EXPECT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::SelfCheck);
}

TEST(MecanumSm, OutsideFenceGoesSelfCheck) {
    UgvState state;
    MecanumUgvController controller(state);
    goReady(controller, state, 1.0);
    setPose(state, 1.05, 50.0, 0.0, 0.0);
    controller.update(1.05);
    EXPECT_FALSE(controller.healthReady());
    EXPECT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::SelfCheck);
}

TEST(MecanumSm, ReadyStopIsNotAtInitialPose) {
    UgvState state;
    MecanumUgvController controller(state);
    setPose(state, 1.0, 1.5, -0.8, 0.4);
    controller.update(1.0);
    controller.update(1.002);
    EXPECT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::Ready);
}

TEST(MecanumSm, ResetStopReturnsReady) {
    UgvState state;
    MecanumUgvController controller(state);
    goReady(controller, state, 1.0);
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

TEST(MecanumSm, ResetDoesNotJumpToCustom1) {
    UgvState state;
    MecanumUgvController controller(state);
    goReady(controller, state, 1.0);
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

TEST(MecanumSm, Custom1StopReturnsReady) {
    UgvState state;
    MecanumUgvController controller(state);
    goReady(controller, state, 1.0);
    WorldVelocityReference reference;
    reference.valid = true;
    reference.vx = 0.2;
    reference.stamp = Time(1.0);
    controller.setWorldReference(reference);
    postCommand(controller, event_type::CUSTOM1_REQUESTED, 1.01);
    setPose(state, 1.01, 0.0, 0.0, 0.0);
    controller.update(1.01);
    ASSERT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::Custom1);
    postCommand(controller, event_type::STOP_REQUESTED, 1.02);
    setPose(state, 1.02, 0.0, 0.0, 0.0);
    controller.update(1.02);
    EXPECT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::Ready);
}

TEST(MecanumSm, Custom1DoesNotValidateAlgorithmTimestamp) {
    UgvState state;
    MecanumUgvController controller(state);
    WorldVelocityReference reference;
    reference.valid = true;
    reference.vx = 0.2;
    reference.stamp = Time(1001.0);
    enterCustom1(controller, state, reference, 1.0);

    setPose(state, 1.1, state.x, state.y, state.yaw);
    controller.update(1.1);
    EXPECT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::Custom1);
}

TEST(MecanumSm, Custom1CanReset) {
    UgvState state;
    MecanumUgvController controller(state);
    goReady(controller, state, 1.0);
    WorldVelocityReference reference;
    reference.valid = true;
    reference.vx = 0.2;
    reference.stamp = Time(1.0);
    controller.setWorldReference(reference);
    postCommand(controller, event_type::CUSTOM1_REQUESTED, 1.01);
    setPose(state, 1.01, 0.0, 0.0, 0.0);
    controller.update(1.01);
    ASSERT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::Custom1);
    ResetTarget goal;
    goal.x = 1.0;
    goal.valid = true;
    controller.setResetTarget(goal);
    postCommand(controller, event_type::RESET_REQUESTED, 1.02);
    setPose(state, 1.02, 0.0, 0.0, 0.0);
    controller.update(1.02);
    EXPECT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::Reset);
}

TEST(MecanumSm, ResetPoseDoesNotDriveStateMachine) {
    UgvState state;
    MecanumUgvController controller(state);
    goReady(controller, state, 1.0);
    ResetTarget goal;
    goal.x = 2.0;
    goal.valid = true;
    controller.setResetTarget(goal);
    setPose(state, 1.02, 0.0, 0.0, 0.0);
    controller.update(1.02);
    EXPECT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::Ready);
}

TEST(MecanumSm, ResetTimeoutReturnsReady) {
    UgvState state;
    MecanumUgvController controller(state);
    auto config = controller.config();
    config.reset_timeout = 0.05;
    controller.setConfig(config);
    goReady(controller, state, 1.0);
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

TEST(MecanumSm, InvalidFenceStaysSelfCheck) {
    UgvState state;
    MecanumUgvController controller(state);
    auto config = controller.config();
    config.fence_x_min = 1.0;
    config.fence_x_max = -1.0;
    controller.setConfig(config);
    setPose(state, 1.0, 0.0, 0.0, 0.0);
    controller.update(1.0);
    controller.update(1.002);
    EXPECT_FALSE(controller.healthReady());
    EXPECT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::SelfCheck);
}

TEST(MecanumSm, SelfCheckDoesNotJumpToCustom1OrReset) {
    UgvState state;
    MecanumUgvController controller(state);
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

TEST(MecanumSm, ResetWithoutTargetStaysResetAndLogs) {
    UgvState state;
    MecanumUgvController controller(state);
    goReady(controller, state, 1.0);
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

TEST(MecanumLaw, IdealPlantCustom1HeadingAndWorldVelocity) {
    UgvState state;
    MecanumUgvController controller(state);
    auto config = controller.config();
    config.command_publish_rate_hz = 500.0;
    controller.setConfig(config);
    goReady(controller, state, 1.0);
    setPose(state, 1.0, 0.0, 0.0, 0.6);
    WorldVelocityReference reference;
    reference.valid = true;
    reference.vx = 0.4;
    reference.vy = 0.0;
    reference.stamp = Time(1.0);
    controller.setWorldReference(reference);
    postCommand(controller, event_type::CUSTOM1_REQUESTED, 1.01);
    controller.update(1.01);
    ASSERT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::Custom1);
    double t = 1.01;
    const double dt = 0.02;
    for (int i = 0; i < 250; ++i) {
        t += dt;
        reference.stamp = Time(t);
        controller.setWorldReference(reference);
        stepHolonomicPlant(state, controller.command(), dt);
        controller.update(t);
    }
    EXPECT_NEAR(wrapAngle(state.yaw), 0.0, 0.08);
    EXPECT_GT(state.x, 0.5);
}

TEST(MecanumLaw, IdealPlantCustom1TracksSettledWorldVelocity) {
    UgvState state;
    MecanumUgvController controller(state);
    auto config = controller.config();
    config.command_publish_rate_hz = 500.0;
    controller.setConfig(config);
    setPose(state, 1.0, 0.0, 0.0, 0.0);
    WorldVelocityReference reference;
    reference.valid = true;
    reference.vx = 0.5;
    reference.vy = -0.3;
    reference.stamp = Time(1.0);
    enterCustom1(controller, state, reference, 1.0);
    double t = 1.01;
    const double dt = 0.02;
    double vwx = 0.0;
    double vwy = 0.0;
    for (int i = 0; i < 25; ++i) {
        t += dt;
        reference.stamp = Time(t);
        controller.setWorldReference(reference);
        plantWorldVelocity(state, controller.command(), vwx, vwy);
        stepHolonomicPlant(state, controller.command(), dt);
        controller.update(t);
    }
    EXPECT_NEAR(wrapAngle(state.yaw), 0.0, 1.0e-3);
    EXPECT_NEAR(vwx, 0.5, 1.0e-6);
    EXPECT_NEAR(vwy, -0.3, 1.0e-6);
}

TEST(MecanumLaw, IdealPlantCustom1TracksWorldVelocityStep) {
    UgvState state;
    MecanumUgvController controller(state);
    auto config = controller.config();
    config.command_publish_rate_hz = 500.0;
    controller.setConfig(config);
    setPose(state, 1.0, 0.0, 0.0, 0.0);
    WorldVelocityReference reference;
    reference.valid = true;
    reference.vx = 0.4;
    reference.vy = 0.0;
    reference.stamp = Time(1.0);
    enterCustom1(controller, state, reference, 1.0);
    double t = 1.01;
    const double dt = 0.02;
    for (int i = 0; i < 10; ++i) {
        t += dt;
        reference.stamp = Time(t);
        controller.setWorldReference(reference);
        stepHolonomicPlant(state, controller.command(), dt);
        controller.update(t);
    }
    reference.vx = 0.2;
    reference.vy = 0.35;
    double vwx = 0.0;
    double vwy = 0.0;
    for (int i = 0; i < 10; ++i) {
        t += dt;
        reference.stamp = Time(t);
        controller.setWorldReference(reference);
        plantWorldVelocity(state, controller.command(), vwx, vwy);
        stepHolonomicPlant(state, controller.command(), dt);
        controller.update(t);
    }
    EXPECT_NEAR(vwx, 0.2, 1.0e-6);
    EXPECT_NEAR(vwy, 0.35, 1.0e-6);
}

TEST(MecanumLaw, IdealPlantCustom1TracksSlowWorldVelocitySine) {
    UgvState state;
    MecanumUgvController controller(state);
    auto config = controller.config();
    config.command_publish_rate_hz = 500.0;
    controller.setConfig(config);
    setPose(state, 1.0, 0.0, 0.0, 0.0);
    WorldVelocityReference reference;
    reference.valid = true;
    reference.vx = 0.4;
    reference.vy = 0.0;
    reference.stamp = Time(1.0);
    enterCustom1(controller, state, reference, 1.0);
    double t = 1.01;
    const double dt = 0.02;
    double max_err = 0.0;
    for (int i = 0; i < 250; ++i) {
        t += dt;
        reference.vx = 0.4 * std::cos(0.4 * t);
        reference.vy = 0.4 * std::sin(0.4 * t);
        reference.stamp = Time(t);
        controller.setWorldReference(reference);
        controller.update(t);
        double vwx = 0.0;
        double vwy = 0.0;
        plantWorldVelocity(state, controller.command(), vwx, vwy);
        max_err = std::max(max_err, std::hypot(vwx - reference.vx, vwy - reference.vy));
        stepHolonomicPlant(state, controller.command(), dt);
    }
    EXPECT_NEAR(wrapAngle(state.yaw), 0.0, 0.02);
    EXPECT_LT(max_err, 0.02);
}

TEST(MecanumLaw, IdealPlantCustom1TracksCircleWithHeadingDisturbance) {
    UgvState state;
    MecanumUgvController controller(state);
    auto config = controller.config();
    config.command_publish_rate_hz = 500.0;
    controller.setConfig(config);
    constexpr double kRadius = 0.8;
    constexpr double kOmega = 0.5;
    WorldVelocityReference reference;
    reference.valid = true;
    reference.vx = 0.0;
    reference.vy = kRadius * kOmega;
    reference.stamp = Time(1.0);
    enterCustom1(controller, state, reference, 1.0);
    setPose(state, 1.01, kRadius, 0.0, 0.0);
    controller.update(1.01);
    double t = 1.01;
    const double dt = 0.02;
    double max_radial = 0.0;
    for (int i = 0; i < 650; ++i) {
        t += dt;
        const double angle = kOmega * (t - 1.01);
        reference.vx = -kRadius * kOmega * std::sin(angle);
        reference.vy = kRadius * kOmega * std::cos(angle);
        reference.stamp = Time(t);
        controller.setWorldReference(reference);
        controller.update(t);
        stepHolonomicPlant(state, controller.command(), dt);
        if (i % 75 == 37) {
            state.yaw = wrapAngle(state.yaw + 0.1);
        }
        max_radial = std::max(max_radial, std::abs(std::hypot(state.x, state.y) - kRadius));
    }
    EXPECT_LT(max_radial, 0.05);
    EXPECT_LT(std::abs(wrapAngle(state.yaw)), 0.15);
}

TEST(MecanumLaw, IdealPlantCustom1SaturatesWorldVelocityOnFluBox) {
    UgvState state;
    MecanumUgvController controller(state);
    auto config = controller.config();
    config.command_publish_rate_hz = 500.0;
    controller.setConfig(config);
    setPose(state, 1.0, 0.0, 0.0, 0.0);
    WorldVelocityReference reference;
    reference.valid = true;
    reference.vx = 2.0;
    reference.vy = -2.0;
    reference.stamp = Time(1.0);
    enterCustom1(controller, state, reference, 1.0);
    double t = 1.01;
    const double dt = 0.02;
    t += dt;
    reference.stamp = Time(t);
    controller.setWorldReference(reference);
    controller.update(t);
    double vwx = 0.0;
    double vwy = 0.0;
    plantWorldVelocity(state, controller.command(), vwx, vwy);
    EXPECT_NEAR(vwx, 1.0, 1.0e-9);
    EXPECT_NEAR(vwy, -1.0, 1.0e-9);
}

namespace {

ugv_reset_client::Grant goFor(const MecanumUgvController& controller, const ugv_reset_client::Pose& start,
                              const std::vector<ugv_reset_client::Point>& via,
                              const ugv_reset_client::Pose& goal, double s_limit, uint32_t id = 1U) {
    ugv_reset_client::Grant g;
    g.session = controller.resetExecutor().sessionId();
    g.state = ugv_reset_client::GrantState::Go;
    g.grant_id = id;
    g.start = start;
    g.via = via;
    g.goal = goal;
    g.s_limit = s_limit;
    g.ttl = 1.0;
    return g;
}

ugv_reset_client::Grant rejectFor(const MecanumUgvController& controller, ugv_reset_client::Reason reason,
                                  const std::string& detail) {
    ugv_reset_client::Grant g;
    g.session = controller.resetExecutor().sessionId();
    g.state = ugv_reset_client::GrantState::Reject;
    g.reason = reason;
    g.detail = detail;
    return g;
}

void enterReset(MecanumUgvController& controller, UgvState& state, double goal_x) {
    goReady(controller, state, 1.0);
    ResetTarget goal;
    goal.x = goal_x;
    goal.valid = true;
    controller.setResetTarget(goal);
    postCommand(controller, event_type::RESET_REQUESTED, 1.01);
    setPose(state, 1.01, 0.0, 0.0, 0.0);
    controller.update(1.01);
    controller.update(1.012);
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

TEST(MecanumSm, ResetBeginsASessionThatDeclaresTheVehicleToTheCoordinator) {
    UgvState state;
    MecanumUgvController controller(state);
    enterReset(controller, state, 1.0);
    ASSERT_TRUE(controller.resetExecutor().active());
    ugv_reset_client::Session session;
    ASSERT_TRUE(controller.takeResetSession(&session));
    EXPECT_EQ(session.phase, ugv_reset_client::Phase::Requesting);
    EXPECT_EQ(session.spec.model, ugv_reset_client::Model::Omni);
    EXPECT_DOUBLE_EQ(session.spec.footprint.half_length, 0.345);  // 0.41 x 0.36 m plus 0.14 m a side
    EXPECT_DOUBLE_EQ(session.spec.footprint.half_width, 0.32);
    EXPECT_DOUBLE_EQ(session.spec.ell_bound, 0.0);
    EXPECT_DOUBLE_EQ(session.target.x, 1.0);
}

TEST(MecanumSm, AGrantedRouteDrivesTheTwistAndAStopEndsTheSessionWithOneResult) {
    UgvState state;
    MecanumUgvController controller(state);
    enterReset(controller, state, 1.0);
    controller.onResetGrant(goFor(controller, {0, 0, 0}, {}, {1.0, 0.0, 0.0}, 1e9));
    for (int i = 0; i < 20; ++i) {
        setPose(state, 1.02 + 0.02 * i, 0.0, 0.0, 0.0);
        controller.update(1.02 + 0.02 * i);
    }
    EXPECT_GT(controller.command().linear_x, 0.05);
    EXPECT_LE(controller.command().linear_x, 0.35 + 1e-9);
    postCommand(controller, event_type::STOP_REQUESTED, 1.5);
    setPose(state, 1.5, 0.1, 0.0, 0.0);
    controller.update(1.5);
    EXPECT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::Ready);
    ugv_reset_client::Result result;
    ASSERT_TRUE(controller.takeResetResult(&result));
    EXPECT_EQ(result.outcome, ugv_reset_client::Outcome::Stopped);
    EXPECT_FALSE(controller.takeResetResult(&result));
    // A grant that arrives after the session ended never moves the vehicle.
    controller.onResetGrant(goFor(controller, {0, 0, 0}, {}, {1.0, 0.0, 0.0}, 1e9, 2U));
    controller.update(1.51);
    EXPECT_DOUBLE_EQ(controller.command().linear_x, 0.0);
}

TEST(MecanumSm, ArrivalAtTheTargetEndsResetThroughTheCoordinatorsRoute) {
    UgvState state;
    MecanumUgvController controller(state);
    enterReset(controller, state, 0.0);
    double t = 1.02;
    for (int i = 0; i < 100 && controller.stateMachine().currentState(region_type::CONTROL) ==
                                   state_type::Reset;
         ++i, t += 0.02) {
        if (i % 10 == 0) {
            controller.onResetGrant(goFor(controller, {0, 0, 0}, {}, {0.0, 0.0, 0.0}, 1e9));
        }
        setPose(state, t, 0.0, 0.0, 0.0);
        controller.update(t);
    }
    EXPECT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::Ready);
    ugv_reset_client::Result result;
    ASSERT_TRUE(controller.takeResetResult(&result));
    EXPECT_EQ(result.outcome, ugv_reset_client::Outcome::Arrived);
}

TEST(MecanumSm, ARejectionByTheCoordinatorReturnsReadyAndAFreshRetryGetsANewSession) {
    UgvState state;
    MecanumUgvController controller(state);
    enterReset(controller, state, 1.0);
    const uint32_t first = controller.resetExecutor().sessionId();
    controller.onResetGrant(rejectFor(controller, ugv_reset_client::Reason::NoRoute, "no route"));
    setPose(state, 1.014, 0.0, 0.0, 0.0);
    controller.update(1.014);
    controller.update(1.016);
    ASSERT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::Ready);
    EXPECT_TRUE(controller.healthReady());
    ugv_reset_client::Result result;
    ASSERT_TRUE(controller.takeResetResult(&result));
    EXPECT_EQ(result.outcome, ugv_reset_client::Outcome::Infeasible);
    postCommand(controller, event_type::RESET_REQUESTED, 1.02);
    setPose(state, 1.02, 0.0, 0.0, 0.0);
    controller.update(1.02);
    controller.update(1.022);
    ASSERT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::Reset);
    EXPECT_GT(controller.resetExecutor().sessionId(), first);
}

TEST(MecanumSm, TheResetTimeoutIsNinetySecondsUnlessConfiguredOtherwise) {
    UgvState state;
    MecanumUgvController controller(state);
    EXPECT_DOUBLE_EQ(controller.config().reset_timeout, 90.0);
    EXPECT_DOUBLE_EQ(controller.resetExecutor().config().timeout, 90.0);
    auto config = controller.config();
    config.reset_timeout = 12.0;
    controller.setConfig(config);
    EXPECT_DOUBLE_EQ(controller.resetExecutor().config().timeout, 12.0);
}

TEST(MecanumSm, ResetOutcomesAreLoggedWithTheirReason) {
    setLogSink(&captureSink);
    {
        capturedLog().clear();
        UgvState state;
        MecanumUgvController controller(state);
        enterReset(controller, state, 1.0);
        controller.onResetGrant(rejectFor(controller, ugv_reset_client::Reason::GoalOccupied,
                                          "target occupied by ugv3"));
        controller.update(1.014);
        controller.update(1.016);
        EXPECT_TRUE(logged("Reset ended after"));
        EXPECT_TRUE(logged("INFEASIBLE"));
        EXPECT_TRUE(logged("target occupied by ugv3")) << "the coordinator's reason is dropped";
    }
    {
        capturedLog().clear();
        UgvState state;
        MecanumUgvController controller(state);
        enterReset(controller, state, 0.0);
        double t = 1.02;
        for (int i = 0; i < 100; ++i, t += 0.02) {
            if (i % 10 == 0) {
                controller.onResetGrant(goFor(controller, {0, 0, 0}, {}, {0.0, 0.0, 0.0}, 1e9));
            }
            setPose(state, t, 0.0, 0.0, 0.0);
            controller.update(t);
        }
        EXPECT_TRUE(logged("Reset arrived after"));
    }
    {
        capturedLog().clear();
        UgvState state;
        MecanumUgvController controller(state);
        enterReset(controller, state, 1.0);
        postCommand(controller, event_type::STOP_REQUESTED, 1.016);
        controller.update(1.016);
        EXPECT_TRUE(logged("Reset ended after"));
        EXPECT_TRUE(logged("stopped"));
    }
    {
        capturedLog().clear();
        UgvState state;
        MecanumUgvController controller(state);
        enterReset(controller, state, 1.0);
        auto config = controller.config();
        config.reset_timeout = 0.5;
        controller.setConfig(config);
        setPose(state, 2.0, 0.0, 0.0, 0.0);
        controller.update(2.0);
        controller.update(2.002);
        EXPECT_TRUE(logged("Reset timed out after"));
    }
    setLogSink(nullptr);
}

}  // namespace mecanum_ugv_controller
