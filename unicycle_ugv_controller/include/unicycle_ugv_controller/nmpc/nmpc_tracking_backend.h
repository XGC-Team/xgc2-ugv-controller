#pragma once

#include <ros/time.h>

#include <array>
#include <vector>

#include "unicycle_ugv_controller/common/types.h"
#include "unicycle_ugv_controller/nmpc/unicycle_nmpc_solver.h"

namespace unicycle_ugv_controller {

class NmpcTrackingBackend {
   public:
    void configure(const ControllerConfig& config);
    bool enter();
    void exit();
    bool compute(const UgvState& state, const std::vector<Se2Reference>& refs, const ros::Time& now,
                 ControlCommand& command);
    int status() const {
        return status_;
    }
    double solveTimeMs() const {
        return solve_time_ms_;
    }
    const std::array<NmpcStateVector, UNICYCLE_NMPC_N + 1>& predictedStates() const {
        return solver_.predictedStates();
    }
    size_t predictedStateCount() const {
        return solver_.predictedStateCount();
    }

   private:
    ControllerConfig config_{};
    UnicycleNmpcSolver solver_;
    bool bounds_configured_{false};
    bool entered_{false};
    int status_{-1};
    double solve_time_ms_{0.0};
};

}  // namespace unicycle_ugv_controller
