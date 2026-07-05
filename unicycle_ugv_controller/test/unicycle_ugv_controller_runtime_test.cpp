#include <gtest/gtest.h>
#include <rigid_state_estimator_msgs/PlanarStateEstimate.h>
#include <unicycle_reference_trajectory_msgs/AnalyticReference.h>

#include "unicycle_ugv_controller/common/reference_cache.h"
#include "unicycle_ugv_controller/common/types.h"
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
    state.estimator_state = rigid_state_estimator_msgs::PlanarStateEstimate::STATE_RUNNING;
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
    state.estimator_state = rigid_state_estimator_msgs::PlanarStateEstimate::STATE_RUNNING;
    state.estimator_flags = 0U;

    UnicycleUgvController controller(state);
    controller.update(1.0);
    controller.update(1.01);
    ASSERT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::Ready);

    state.estimator_flags = rigid_state_estimator_msgs::PlanarStateEstimate::FLAG_FAULT;
    controller.update(1.02);
    controller.update(1.03);
    EXPECT_EQ(controller.stateMachine().currentState(region_type::CONTROL), state_type::SelfCheck);
}

TEST(UnicycleUgvControllerRuntime, AutoStartTrackingWhenStateAndReferenceAreReady) {
    ros::Time::init();
    UgvState state;
    state.received = true;
    state.stamp = ros::Time(1.0);
    state.estimator_state = rigid_state_estimator_msgs::PlanarStateEstimate::STATE_RUNNING;
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

}  // namespace
}  // namespace unicycle_ugv_controller
