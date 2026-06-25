#include "unicycle_reference_trajectory/state_machine/ready_state.h"

#include "unicycle_reference_trajectory/unicycle_reference_trajectory_runtime.h"

namespace unicycle_reference_trajectory {

ReadyState::ReadyState(ReferenceTrajectoryRuntime& runtime) : runtime_(runtime) {}

::state_machine::ActionResult ReadyState::onEnter(::state_machine::StateContext& ctx) {
    runtime_.enterState(ReferenceStatus::STATE_READY);
    status_gate_.reset();
    publishStatusIfDue(ctx);
    return {};
}

::state_machine::ActionResult ReadyState::onTick(::state_machine::StateContext& ctx) {
    publishStatusIfDue(ctx);
    return {};
}

void ReadyState::publishStatusIfDue(::state_machine::StateContext& ctx) {
    if (!status_gate_.due(runtime_.currentTime(), 1.0 / runtime_.config().status_rate_hz)) {
        return;
    }
    ::state_machine::Event event(output_event_type::PUBLISH_STATUS,
                                 ::state_machine::EventTimestamp{runtime_.currentTime()});
    event.category = ::state_machine::EventCategory::kOutput;
    event.source = "ready_state";
    ctx.emitOutput(std::move(event));
}

}  // namespace unicycle_reference_trajectory
