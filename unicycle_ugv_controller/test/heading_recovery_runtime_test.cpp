#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "unicycle_ugv_controller/common/types.h"

namespace unicycle_ugv_controller {
namespace {
UgvState stoppedState() {
    UgvState state;
    state.received = true;
    state.velocity_valid = true;
    return state;
}
WorldPvaReference lateralReference(double y = 0.3) {
    WorldPvaReference ref;
    ref.valid = true;
    ref.y = y;
    return ref;
}

TEST(HeadingRecoveryRuntime, DisabledRetainsZeroSpeedLaw) {
    auto state = stoppedState();
    auto out = computeFlatnessCommand(state, lateralReference(), 0.0, 0.01, ControllerConfig{});
    ASSERT_TRUE(out.valid);
    EXPECT_DOUBLE_EQ(out.linear_speed, 0.0);
    EXPECT_DOUBLE_EQ(out.angular_speed, 0.0);
}

TEST(HeadingRecoveryRuntime, LateralRestHasYawAuthorityWithoutLongitudinalJump) {
    auto state = stoppedState();
    ControllerConfig cfg;
    cfg.heading_recovery.gain = 1.0;
    const auto left = computeFlatnessCommand(state, lateralReference(), 0.0, 0.01, cfg);
    const auto right = computeFlatnessCommand(state, lateralReference(-0.3), 0.0, 0.01, cfg);
    ASSERT_TRUE(left.valid && right.valid);
    EXPECT_GT(left.angular_speed, 0.0);
    EXPECT_NEAR(left.angular_speed, -right.angular_speed, 1e-14);
    EXPECT_DOUBLE_EQ(left.linear_speed, 0.0);
}

TEST(HeadingRecoveryRuntime, ReverseAxisRemainsAvailable) {
    auto state = stoppedState();
    auto ref = lateralReference(0.0);
    ref.x = -0.3;
    ControllerConfig cfg;
    cfg.heading_recovery.gain = 1.0;
    const auto out = computeFlatnessCommand(state, ref, 0.0, 0.01, cfg);
    ASSERT_TRUE(out.valid);
    EXPECT_LT(out.linear_speed, 0.0);
    EXPECT_DOUBLE_EQ(out.angular_speed, 0.0);
}

TEST(HeadingRecoveryRuntime, InvalidRateAndGainAreRejectedBeforeClamping) {
    auto state = stoppedState();
    ControllerConfig cfg;
    cfg.heading_recovery.gain = 1.0;
    state.yaw_rate = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(computeFlatnessCommand(state, lateralReference(), 0.0, 0.01, cfg).valid);
    state.yaw_rate = 0.0;
    cfg.heading_recovery.gain = -1.0;
    EXPECT_FALSE(computeFlatnessCommand(state, lateralReference(), 0.0, 0.01, cfg).valid);
}

TEST(HeadingRecoveryRuntime, ExistingCommandBoxStillApplies) {
    auto state = stoppedState();
    ControllerConfig cfg;
    cfg.heading_recovery.gain = 1e3;
    const auto out = computeFlatnessCommand(state, lateralReference(), 0.0, 0.01, cfg);
    ASSERT_TRUE(out.valid);
    EXPECT_DOUBLE_EQ(out.angular_speed, cfg.chassis_max_yaw_rate);
}

TEST(HeadingRecoveryRuntime, ContinuousThroughRestAndLongitudinalPathUnchanged) {
    ControllerConfig base, revised;
    revised.heading_recovery.gain = 1.0;
    auto ref = lateralReference();
    ref.vx = 0.02;
    ref.ax = 0.03;
    auto state = stoppedState();
    double yaw_command[2]{};
    for (int k = 0; k < 2; ++k) {
        state.vx = k == 0 ? -1e-9 : 1e-9;
        const auto old = computeFlatnessCommand(state, ref, 0.1, 0.01, base);
        const auto out = computeFlatnessCommand(state, ref, 0.1, 0.01, revised);
        ASSERT_TRUE(old.valid && out.valid);
        EXPECT_DOUBLE_EQ(old.linear_speed, out.linear_speed);
        EXPECT_DOUBLE_EQ(old.accel, out.accel);
        yaw_command[k] = out.angular_speed;
    }
    EXPECT_NEAR(yaw_command[0], yaw_command[1], 1e-7);
}
}  // namespace
}  // namespace unicycle_ugv_controller
