#include <estimator_vrpn_ugv_state/PlanarStateEstimate.h>
#include <gtest/gtest.h>

#include "unicycle_ugv_controller/unicycle_ugv_controller.h"

namespace unicycle_ugv_controller {
namespace {

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
    state.estimator_state = estimator_vrpn_ugv_state::PlanarStateEstimate::STATE_RUNNING;
    state.estimator_flags = 0U;

    UnicycleUgvController controller(state);
    controller.update(1.0);
    controller.update(1.01);
    EXPECT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::Ready);
}

TEST(UnicycleUgvControllerRuntime, FaultFlagReturnsToSelfCheck) {
    ros::Time::init();
    UgvState state;
    state.received = true;
    state.stamp = ros::Time(1.0);
    state.estimator_state = estimator_vrpn_ugv_state::PlanarStateEstimate::STATE_RUNNING;
    state.estimator_flags = 0U;

    UnicycleUgvController controller(state);
    controller.update(1.0);
    controller.update(1.01);
    ASSERT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::Ready);

    state.estimator_flags = estimator_vrpn_ugv_state::PlanarStateEstimate::FLAG_FAULT;
    controller.update(1.02);
    controller.update(1.03);
    EXPECT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::SelfCheck);
}

}  // namespace
}  // namespace unicycle_ugv_controller
