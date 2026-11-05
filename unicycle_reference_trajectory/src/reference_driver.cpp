#include "unicycle_reference_trajectory/reference_driver.h"

#include <cmath>

namespace unicycle_reference_trajectory {
void ReferenceTrajectoryDriver::configure(const ReferenceTrajectoryConfig& config,
                                          const DefaultAnalyticReferenceConfig& defaults) {
    runtime_.setConfig(config);
    default_analytic_ = defaults;
    default_analytic_sent_ = false;
}
ReferenceTrajectoryDriver::Request ReferenceTrajectoryDriver::acceptAnalytic(
    const reference::AnalyticReference& msg, double now) {
    return runtime_.acceptAnalytic(msg, now) ? Request::kAccepted : Request::kRejected;
}
ReferenceTrajectoryDriver::Request ReferenceTrajectoryDriver::acceptSampled(
    const reference::SampledReference& msg, double now) {
    return runtime_.acceptSampled(msg, now) ? Request::kAccepted : Request::kRejected;
}
ReferenceTrajectoryDriver::Request ReferenceTrajectoryDriver::acceptWaypoint(
    const reference::WaypointReferenceRequest& msg, double now) {
    return runtime_.acceptWaypoint(msg, now) ? Request::kAccepted : Request::kRejected;
}
void ReferenceTrajectoryDriver::reset(double) {
    runtime_.reset();
    default_analytic_sent_ = false;
}
ReferenceTrajectoryDriver::UpdateResult ReferenceTrajectoryDriver::update(double now) {
    runtime_.update(now);
    UpdateResult result;
    if (default_analytic_.enabled && !default_analytic_sent_ && std::isfinite(now) &&
        runtime_.currentState() == reference::ReferenceStatus::STATE_READY) {
        result.default_analytic = acceptAnalytic(makeDefaultAnalytic(default_analytic_, now), now);
        default_analytic_sent_ = *result.default_analytic != Request::kRejected;
    }
    result.events = runtime_.outputs();
    return result;
}
}  // namespace unicycle_reference_trajectory
