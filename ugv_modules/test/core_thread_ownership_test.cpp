// Cores have one serial owner; construction on that owner's thread is not a business gate.
// These calls alternate live threads but never overlap. Native module affinity and concurrent
// completion publication are tested separately at the actual transport boundary.

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

TEST(CoreThreads, ReferenceUsesSerialOwnerWithoutAConstructionThreadGate) {
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
    threads.run([&] { update_for(100); });  // thread 1: the serial owner now
    EXPECT_EQ(driver->runtime().currentState(), urt::reference::ReferenceStatus::STATE_READY);
    EXPECT_EQ(driver->runtime().flags(), 0u);

    // The same generator can be reconfigured on the next serial owner.
    threads.run([&] {  // thread 0 again
        driver->configure(urt::ReferenceTrajectoryConfig{}, urt::DefaultAnalyticReferenceConfig{});
        update_for(100);
    });
    EXPECT_EQ(driver->runtime().currentState(), urt::reference::ReferenceStatus::STATE_READY);
    EXPECT_EQ(driver->runtime().flags(), 0u);
}

TEST(CoreThreads, ControllerUsesSerialOwnerWithoutAConstructionThreadGate) {
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
            controller->update(now, now);
        }
    };
    threads.run([&] { update_for(500); });  // thread 1: a healthy vehicle for a second
    EXPECT_EQ(controller->mode(), ugv::ControlMode::Ready);
    threads.run([&] { update_for(500); });  // thread 0: the builder
    EXPECT_EQ(controller->mode(), ugv::ControlMode::Ready);
}

}  // namespace
