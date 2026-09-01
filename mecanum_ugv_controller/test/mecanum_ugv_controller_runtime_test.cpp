#include <gtest/gtest.h>

#include "mecanum_ugv_controller/common/types.h"
#include "mecanum_ugv_controller/mecanum_ugv_controller.h"

namespace mecanum_ugv_controller {

TEST(MecanumUgvControllerRuntime, HolonomicResetCommandsBodyVxVyOmegaTogether) {
    UgvState state;
    state.x = 0.0;
    state.y = 0.0;
    state.yaw = 0.0;
    state.vx = 0.0;
    state.vy = 0.0;
    state.yaw_rate = 0.0;
    ResetTarget goal;
    goal.x = 1.0;
    goal.y = 0.5;
    goal.yaw = 0.4;
    goal.valid = true;
    ControllerConfig config;
    const HolonomicResetOutput output = computeHolonomicResetCommand(state, goal, config);
    EXPECT_GT(output.linear_x, 0.0);
    EXPECT_GT(output.linear_y, 0.0);
    EXPECT_GT(output.angular_z, 0.0);
    EXPECT_FALSE(output.settled);
}

TEST(MecanumUgvControllerRuntime, HolonomicResetRotatesWorldErrorIntoBodyVxVy) {
    UgvState state;
    state.x = 0.0;
    state.y = 0.0;
    state.yaw = 1.5707963267948966;
    ResetTarget goal;
    goal.x = 1.0;
    goal.y = 0.0;
    goal.yaw = 1.5707963267948966;
    goal.valid = true;
    ControllerConfig config;
    const HolonomicResetOutput output = computeHolonomicResetCommand(state, goal, config);
    EXPECT_NEAR(output.linear_x, 0.0, 1.0e-6);
    EXPECT_LT(output.linear_y, 0.0);
    EXPECT_NEAR(output.angular_z, 0.0, 1.0e-6);
}

TEST(MecanumUgvControllerRuntime, HolonomicResetSettlesWithLooseResidual) {
    UgvState state;
    state.x = 1.20;
    state.y = 0.08;
    state.yaw = 0.35;
    state.vx = 0.01;
    state.vy = -0.02;
    state.yaw_rate = 0.02;
    ResetTarget goal;
    goal.x = 1.0;
    goal.y = 0.0;
    goal.yaw = 0.0;
    goal.valid = true;
    ControllerConfig config;
    const HolonomicResetOutput output = computeHolonomicResetCommand(state, goal, config);
    EXPECT_TRUE(output.position_ok);
    EXPECT_TRUE(output.yaw_ok);
    EXPECT_TRUE(output.settled);
}

TEST(MecanumUgvControllerRuntime, CanonicalPoseMovesSelfCheckToReady) {
    ros::Time::init();
    UgvState state;
    state.received = true;
    state.stamp = ros::Time(1.0);
    state.x = 0.47;
    state.y = 2.5;
    MecanumUgvController controller(state);
    controller.update(1.0);
    controller.update(1.01);
    EXPECT_TRUE(controller.healthReady());
    EXPECT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::Ready);
}

TEST(MecanumUgvControllerRuntime, ResetTimeoutReturnsReady) {
    ros::Time::init();
    UgvState state;
    state.received = true;
    state.stamp = ros::Time(1.0);
    MecanumUgvController controller(state);
    auto config = controller.config();
    config.reset_timeout = 0.05;
    controller.setConfig(config);
    controller.update(1.0);
    controller.update(1.01);
    ResetTarget goal;
    goal.x = 4.0;
    goal.y = 0.0;
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

TEST(MecanumUgvControllerRuntime, ResetArrivesWithLooseResidualThenReturnsReady) {
    ros::Time::init();
    UgvState state;
    state.received = true;
    state.stamp = ros::Time(1.0);
    state.x = 0.05;
    state.y = -0.04;
    state.yaw = 0.10;
    MecanumUgvController controller(state);
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

TEST(MecanumUgvControllerRuntime, TrackCommandHoldsHeadingOnWorldX) {
    UgvState state;
    state.yaw = 0.4;
    WorldVelocityReference reference;
    reference.valid = true;
    ControllerConfig config;
    const HolonomicTrackOutput output = computeHolonomicTrackCommand(state, reference, config);
    EXPECT_NEAR(output.linear_x, 0.0, 1.0e-6);
    EXPECT_NEAR(output.linear_y, 0.0, 1.0e-6);
    EXPECT_LT(output.angular_z, 0.0);
}

TEST(MecanumUgvControllerRuntime, TrackCommandRotatesWorldVelocityIntoBodyForwardLeft) {
    UgvState state;
    state.yaw = 1.5707963267948966;
    WorldVelocityReference reference;
    reference.valid = true;
    reference.vx = 1.0;
    reference.vy = 0.0;
    ControllerConfig config;
    config.track_max_speed = 2.0;
    const HolonomicTrackOutput output = computeHolonomicTrackCommand(state, reference, config);
    EXPECT_NEAR(output.linear_x, 0.0, 1.0e-6);
    EXPECT_NEAR(output.linear_y, -1.0, 1.0e-6);
    EXPECT_NEAR(output.angular_z,
                headingRateToTarget(state.yaw, 0.0, config.track_kp_yaw, config.track_max_yaw_rate),
                1.0e-6);
}

TEST(MecanumUgvControllerRuntime, TrackingFollowsCustomThenHoldCommandReturnsHold) {
    ros::Time::init();
    UgvState state;
    state.received = true;
    state.stamp = ros::Time(1.0);
    MecanumUgvController controller(state);
    controller.update(1.0);
    controller.update(1.01);
    ASSERT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::Ready);

    WorldVelocityReference reference;
    reference.valid = true;
    reference.vx = 0.4;
    reference.stamp = ros::Time(1.0);
    controller.setWorldReference(reference);
    ::state_machine::Event track(event_type::TRACKING_REQUESTED,
                                 ::state_machine::EventTimestamp{1.02});
    track.source = "test";
    ASSERT_TRUE(controller.postEvent(std::move(track)).ok());
    state.stamp = ros::Time(1.02);
    controller.update(1.02);
    ASSERT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::Tracking);

    ::state_machine::Event hold(event_type::HOLD_REQUESTED, ::state_machine::EventTimestamp{1.03});
    hold.source = "test";
    ASSERT_TRUE(controller.postEvent(std::move(hold)).ok());
    state.stamp = ros::Time(1.03);
    controller.update(1.03);
    EXPECT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::Hold);
}

}  // namespace mecanum_ugv_controller
