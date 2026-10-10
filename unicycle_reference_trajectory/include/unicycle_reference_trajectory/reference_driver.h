#pragma once

#include <optional>
#include <string>
#include <vector>

#include "unicycle_reference_trajectory/default_analytic.h"
#include "unicycle_reference_trajectory/reference_types.h"
#include "unicycle_reference_trajectory/unicycle_reference_trajectory_runtime.h"

namespace unicycle_reference_trajectory {

// The runtime with the input side every transport shares: a request is accepted
// into the runtime and, when accepted, posts its input event; the default
// analytic request is issued once when the runtime becomes Ready; one update
// is the runtime update followed by that request. The ROS node and the
// aggregator module both drive this class, with the time they read from their
// own clock, so the request handling exists once.
class ReferenceTrajectoryDriver {
   public:
    enum class Request { kPosted, kRejected, kPostFailed };

    struct UpdateResult {
        // Output events of this update, in order.
        std::vector<::state_machine::Event> events;
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

    // Producers. `now_sec` stamps the posted event: the time the request arrived.
    Request acceptAnalytic(const reference::AnalyticReference& msg, double now_sec);
    Request acceptSampled(const reference::SampledReference& msg, double now_sec);
    Request acceptWaypoint(const reference::WaypointReferenceRequest& msg, double now_sec);
    // Restarts the runtime and re-arms the default request. Returns the post status.
    ::state_machine::Status reset(double now_sec);

    // One main-loop iteration at `now_sec`.
    UpdateResult update(double now_sec);

    // Message of the last failed event post, for the caller's log.
    const std::string& postError() const {
        return post_error_;
    }

   private:
    Request accept(const reference::AnalyticReference& msg, const char* source, double now_sec);
    bool post(uint32_t event_id, const char* source, double now_sec);

    ReferenceTrajectoryRuntime runtime_;
    DefaultAnalyticReferenceConfig default_analytic_;
    bool default_analytic_sent_{false};
    std::string post_error_;
};

}  // namespace unicycle_reference_trajectory
