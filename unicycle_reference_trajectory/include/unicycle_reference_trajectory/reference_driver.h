#pragma once

#include <optional>
#include <string>
#include <vector>

#include "unicycle_reference_trajectory/default_analytic.h"
#include "unicycle_reference_trajectory/reference_types.h"
#include "unicycle_reference_trajectory/unicycle_reference_trajectory_runtime.h"

namespace unicycle_reference_trajectory {

// The same typed input and default-reference policy for ROS and native forms.
class ReferenceTrajectoryDriver {
   public:
    enum class Request { kAccepted, kRejected };

    struct UpdateResult {
        // Output events of this update, in order.
        std::vector<ReferencePublication> events;
        // The outcome of the default analytic request, when this update issued it.
        // A refused request is issued again at the next update.
        std::optional<Request> default_analytic;
    };

    ReferenceTrajectoryDriver() = default;

    // Applies the runtime configuration (which restarts the runtime) and the default request.
    void configure(const ReferenceTrajectoryConfig& config,
                   const DefaultAnalyticReferenceConfig& default_analytic);

    ReferenceTrajectoryRuntime& runtime() {
        return runtime_;
    }
    const ReferenceTrajectoryRuntime& runtime() const {
        return runtime_;
    }

    // Producers run on the owner. `now_sec` records arrival time at the transport boundary.
    Request acceptAnalytic(const reference::AnalyticReference& msg, double now_sec);
    Request acceptSampled(const reference::SampledReference& msg, double now_sec);
    Request acceptWaypoint(const reference::WaypointReferenceRequest& msg, double now_sec);
    // Restarts the runtime and re-arms the default request.
    void reset(double now_sec);

    // One main-loop iteration at `now_sec`.
    UpdateResult update(double now_sec);

   private:
    ReferenceTrajectoryRuntime runtime_;
    DefaultAnalyticReferenceConfig default_analytic_;
    bool default_analytic_sent_{false};
};

}  // namespace unicycle_reference_trajectory
