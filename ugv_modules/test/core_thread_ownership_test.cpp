// What a core does when it is not called on the thread that built it, and why an instance of a
// module has to be.
//
// The state machine of the cores belongs to the thread that built it and refuses every other one.
// Nothing throws: the reference runtime drops to SelfCheck with the invalid-input flag, and the
// controller's state machine stays where it was. The modules rely on the host to keep an instance
// on one thread (manifest affinity "sticky") and check it; this test shows what the check protects
// against, and fails if the library ever stops refusing, which would make the check and the host's
// affinity unnecessary.

#include <gtest/gtest.h>

#include <memory>

#include "test_host.h"
#include "unicycle_reference_trajectory/reference_driver.h"
#include "unicycle_ugv_controller/common/types.h"
#include "unicycle_ugv_controller/unicycle_ugv_controller.h"

namespace {

namespace urt = unicycle_reference_trajectory;
namespace ugv = unicycle_ugv_controller;
using ugv_modules_test::RotatingThreads;

TEST(CoreThreads, TheReferenceRuntimeRefusesAThreadThatDidNotBuildIt) {
    RotatingThreads threads(2);  // both live throughout: no two calls share an id
    std::unique_ptr<urt::ReferenceTrajectoryDriver> driver;
    threads.run([&] {  // thread 0 builds it
        driver = std::make_unique<urt::ReferenceTrajectoryDriver>();
        driver->configure(urt::ReferenceTrajectoryConfig{}, urt::DefaultAnalyticReferenceConfig{});
    });
    double now = 1000.0;
    auto update_for = [&](int count) {
        for (int k = 0; k < count; ++k) {
            now += 0.01;
            (void)driver->update(now);
        }
    };
    threads.run([&] { update_for(100); });  // thread 1: a second of refusals
    EXPECT_EQ(driver->runtime().currentState(), urt::reference::ReferenceStatus::STATE_SELF_CHECK);
    EXPECT_NE(driver->runtime().flags() & xgc2_math::trajectory::kFlagInvalidInput, 0u);

    // on the thread that built it the same generator, restarted, runs
    threads.run([&] {  // thread 0 again
        driver->configure(urt::ReferenceTrajectoryConfig{}, urt::DefaultAnalyticReferenceConfig{});
        update_for(100);
    });
    EXPECT_EQ(driver->runtime().currentState(), urt::reference::ReferenceStatus::STATE_READY);
    EXPECT_EQ(driver->runtime().flags(), 0u);
}

TEST(CoreThreads, TheControllerStaysWhereItWasOnAThreadThatDidNotBuildIt) {
    RotatingThreads threads(2);
    ugv::UgvState state;
    std::unique_ptr<ugv::UnicycleUgvController> controller;
    threads.run([&] { controller = std::make_unique<ugv::UnicycleUgvController>(state); });
    double now = 10.0;
    auto update_for = [&](int count) {
        for (int k = 0; k < count; ++k) {
            now += 0.002;
            state.stamp = ugv::Time(now);
            state.received = true;
            state.estimator_state = 3;  // RigidStateEstimate::STATE_RUNNING
            controller->update(now);
        }
    };
    threads.run([&] { update_for(500); });  // thread 1: a healthy vehicle for a second
    EXPECT_EQ(controller->stateMachine().currentState(ugv::region_type::CONTROL),
              ugv::state_type::SelfCheck);
    threads.run([&] { update_for(500); });  // thread 0: the builder
    EXPECT_EQ(controller->stateMachine().currentState(ugv::region_type::CONTROL),
              ugv::state_type::Ready);
}

}  // namespace
