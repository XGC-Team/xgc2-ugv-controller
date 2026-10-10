#include "unicycle_reference_trajectory/reference_driver.h"

#include <cmath>
#include <utility>

namespace unicycle_reference_trajectory {

void ReferenceTrajectoryDriver::configure(const ReferenceTrajectoryConfig& config,
                                          const DefaultAnalyticReferenceConfig& default_analytic) {
    runtime_.setConfig(config);
    default_analytic_ = default_analytic;
    default_analytic_sent_ = false;
}

ReferenceTrajectoryDriver::Request ReferenceTrajectoryDriver::acceptAnalytic(
    const reference::AnalyticReference& msg, double now_sec) {
    return accept(msg, "analytic_reference", now_sec);
}

ReferenceTrajectoryDriver::Request ReferenceTrajectoryDriver::acceptSampled(
    const reference::SampledReference& msg, double now_sec) {
    if (!runtime_.acceptSampled(msg)) {
        return Request::kRejected;
    }
    return post(event_type::SAMPLED_RECEIVED, "sampled_reference", now_sec) ? Request::kPosted
                                                                            : Request::kPostFailed;
}

ReferenceTrajectoryDriver::Request ReferenceTrajectoryDriver::acceptWaypoint(
    const reference::WaypointReferenceRequest& msg, double now_sec) {
    if (!runtime_.acceptWaypoint(msg)) {
        return Request::kRejected;
    }
    return post(event_type::WAYPOINT_RECEIVED, "waypoint_reference", now_sec)
               ? Request::kPosted
               : Request::kPostFailed;
}

::state_machine::Status ReferenceTrajectoryDriver::reset(double now_sec) {
    runtime_.reset();
    default_analytic_sent_ = false;
    post(event_type::RESET_REQUESTED, "reset", now_sec);
    return post_error_.empty() ? ::state_machine::Status{}
                               : ::state_machine::Status::error(
                                     ::state_machine::ErrorCode::kInvalidLifecycle, post_error_);
}

ReferenceTrajectoryDriver::UpdateResult ReferenceTrajectoryDriver::update(double now_sec) {
    UpdateResult result;
    runtime_.update(now_sec);
    if (default_analytic_.enabled && !default_analytic_sent_ && std::isfinite(now_sec) &&
        runtime_.currentState() == reference::ReferenceStatus::STATE_READY) {
        result.default_analytic = accept(makeDefaultAnalytic(default_analytic_, now_sec),
                                         "default_analytic_reference", now_sec);
        default_analytic_sent_ = *result.default_analytic != Request::kRejected;
    }
    result.events = runtime_.stateMachine().currentOutputEvents();
    return result;
}

ReferenceTrajectoryDriver::Request ReferenceTrajectoryDriver::accept(
    const reference::AnalyticReference& msg, const char* source, double now_sec) {
    if (!runtime_.acceptAnalytic(msg)) {
        return Request::kRejected;
    }
    return post(event_type::ANALYTIC_RECEIVED, source, now_sec) ? Request::kPosted
                                                                : Request::kPostFailed;
}

bool ReferenceTrajectoryDriver::post(uint32_t event_id, const char* source, double now_sec) {
    ::state_machine::Event event(event_id, ::state_machine::EventTimestamp{now_sec});
    event.source = source;
    const auto status = runtime_.postEvent(std::move(event));
    post_error_ = status.ok() ? std::string() : status.message;
    return status.ok();
}

}  // namespace unicycle_reference_trajectory
