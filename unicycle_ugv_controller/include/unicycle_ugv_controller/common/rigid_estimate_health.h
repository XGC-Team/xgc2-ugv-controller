#pragma once

#include <cstdint>

namespace unicycle_ugv_controller {

// The rigid state estimator's status values the core reads
// (rigid_state_estimator_msgs/RigidStateEstimate STATE_RUNNING and
// FLAG_FAULT); rigid_to_unicycle.h checks them against the message.
constexpr uint8_t kRigidEstimatorStateRunning = 3U;
constexpr uint32_t kRigidEstimatorFlagFault = 256U;

inline bool rigidEstimateHealthy(uint8_t estimator_state, uint32_t flags) {
    return estimator_state == kRigidEstimatorStateRunning &&
           (flags & kRigidEstimatorFlagFault) == 0U;
}

}  // namespace unicycle_ugv_controller
