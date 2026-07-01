#include "unicycle_reference_trajectory/state_machine/active_state.h"

#include "unicycle_reference_trajectory/unicycle_reference_trajectory_runtime.h"

namespace unicycle_reference_trajectory {

ActiveState::ActiveState(ReferenceTrajectoryRuntime& runtime) : runtime_(runtime) {}

::state_machine::ActionResult ActiveState::onEnter(::state_machine::StateContext& ctx) {
    runtime_.enterState(ReferenceStatus::STATE_ACTIVE);
    status_gate_.reset();
    active_gate_.reset();
    if (!runtime_.activatePending()) {
        ::state_machine::Event event(event_type::PLAN_FAILED,
                                     ::state_machine::EventTimestamp{runtime_.currentTime()});
        event.category = ::state_machine::EventCategory::kInternal;
        event.source = "active_state";
        ctx.postInternalEvent(std::move(event));
        return {};
    }
    publishStatusIfDue(ctx);
    publishActiveIfDue(ctx);
    return {};
}

::state_machine::ActionResult ActiveState::onTick(::state_machine::StateContext& ctx) {
    if (!runtime_.activatePending()) {
        ::state_machine::Event event(event_type::PLAN_FAILED,
                                     ::state_machine::EventTimestamp{runtime_.currentTime()});
        event.category = ::state_machine::EventCategory::kInternal;
        event.source = "active_state";
        ctx.postInternalEvent(std::move(event));
        return {};
    }
    if (runtime_.activeExpired(runtime_.currentTime())) {
        ::state_machine::Event event(event_type::TRAJECTORY_EXPIRED,
                                     ::state_machine::EventTimestamp{runtime_.currentTime()});
        event.category = ::state_machine::EventCategory::kInternal;
        event.source = "active_state";
        ctx.postInternalEvent(std::move(event));
        return {};
    }
    publishStatusIfDue(ctx);
    publishActiveIfDue(ctx);
    return {};
}

void ActiveState::publishStatusIfDue(::state_machine::StateContext& ctx) {
    if (!status_gate_.due(runtime_.currentTime(), 1.0 / runtime_.config().status_rate_hz)) {
        return;
    }
    ::state_machine::Event event(output_event_type::PUBLISH_STATUS,
                                 ::state_machine::EventTimestamp{runtime_.currentTime()});
    event.category = ::state_machine::EventCategory::kOutput;
    event.source = "active_state";
    ctx.emitOutput(std::move(event));
}

void ActiveState::publishActiveIfDue(::state_machine::StateContext& ctx) {
    if (!active_gate_.due(runtime_.currentTime(), 1.0 / runtime_.config().active_publish_rate_hz)) {
        return;
    }
    uint32_t event_id = output_event_type::PUBLISH_ACTIVE_POLYNOMIAL;
    if (runtime_.activeType() == trajectory::TrajectoryModelType::kAnalytic) {
        event_id = output_event_type::PUBLISH_ACTIVE_ANALYTIC;
    } else if (runtime_.activeType() == trajectory::TrajectoryModelType::kSampled) {
        event_id = output_event_type::PUBLISH_ACTIVE_SAMPLED;
    }
    ::state_machine::Event event(event_id, ::state_machine::EventTimestamp{runtime_.currentTime()});
    event.category = ::state_machine::EventCategory::kOutput;
    event.source = "active_state";
    ctx.emitOutput(std::move(event));
}

}  // namespace unicycle_reference_trajectory
