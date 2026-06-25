#include "unicycle_reference_trajectory/state_machine/fault_state.h"

#include "unicycle_reference_trajectory/unicycle_reference_trajectory_runtime.h"

namespace unicycle_reference_trajectory {

FaultState::FaultState(ReferenceTrajectoryRuntime& runtime) : runtime_(runtime) {}

::state_machine::ActionResult FaultState::onEnter(::state_machine::StateContext& ctx) {
    runtime_.enterState(ReferenceStatus::STATE_FAULT);
    status_gate_.reset();
    onTick(ctx);
    return {};
}

::state_machine::ActionResult FaultState::onTick(::state_machine::StateContext& ctx) {
    if (!status_gate_.due(runtime_.currentTime(), 1.0 / runtime_.config().status_rate_hz)) {
        return {};
    }
    ::state_machine::Event event(output_event_type::PUBLISH_STATUS,
                                 ::state_machine::EventTimestamp{runtime_.currentTime()});
    event.category = ::state_machine::EventCategory::kOutput;
    event.source = "fault_state";
    ctx.emitOutput(std::move(event));
    return {};
}

}  // namespace unicycle_reference_trajectory
