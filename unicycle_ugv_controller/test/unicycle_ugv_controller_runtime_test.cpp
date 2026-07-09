#include <gtest/gtest.h>
#include <rigid_state_estimator_msgs/PlanarStateEstimate.h>
#include <unicycle_reference_trajectory_msgs/AnalyticReference.h>

#include "unicycle_ugv_controller/common/reference_cache.h"
#include "unicycle_ugv_controller/common/types.h"
#include "unicycle_ugv_controller/nmpc/unicycle_nmpc_solver.h"
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

TEST(UnicycleUgvControllerRuntime, NmpcSolverUsesConfiguredPhysicalBounds) {
    UnicycleNmpcSolver solver;
    ASSERT_TRUE(solver.configureBounds(-1.5, 1.5, 2.0, 0.5235));
    ASSERT_TRUE(solver.initialize());

    Se2StateVector x0 = Se2StateVector::Zero();
    std::vector<Se2Reference> refs(static_cast<size_t>(UnicycleNmpcSolver::horizonSteps() + 1));
    for (auto& ref : refs) {
        ref.state.position << 3.0, 3.0;
        ref.state.yaw = 1.5;
        ref.state.linear_speed = 3.0;
        ref.control.linear_acceleration = 2.0;
        ref.control.yaw_rate = 2.5;
    }

    ASSERT_TRUE(solver.solve(x0, refs));
    EXPECT_LE(std::abs(solver.optimalControl()(1)), 0.5235 + 1.0e-6);
    for (size_t i = 1; i < solver.predictedStateCount(); ++i) {
        EXPECT_GE(solver.predictedStates()[i](3), -1.5 - 1.0e-6);
        EXPECT_LE(solver.predictedStates()[i](3), 1.5 + 1.0e-6);
    }
}

TEST(UnicycleUgvControllerRuntime, NmpcSolverRejectsInvalidBounds) {
    UnicycleNmpcSolver solver;
    EXPECT_FALSE(solver.configureBounds(1.5, 1.5, 2.0, 0.5235));
    EXPECT_FALSE(solver.configureBounds(-1.5, 1.5, 0.0, 0.5235));
    EXPECT_FALSE(solver.configureBounds(-1.5, 1.5, 2.0, 0.0));
}

}  // namespace
}  // namespace unicycle_ugv_controller
