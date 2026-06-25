#include "unicycle_reference_trajectory/state_machine/self_check_state.h"

#include "unicycle_reference_trajectory/unicycle_reference_trajectory_runtime.h"

namespace unicycle_reference_trajectory {

SelfCheckState::SelfCheckState(ReferenceTrajectoryRuntime& runtime) : runtime_(runtime) {}

::state_machine::ActionResult SelfCheckState::onEnter(::state_machine::StateContext& ctx) {
    (void)ctx;
    runtime_.enterState(ReferenceStatus::STATE_SELF_CHECK);
    return {};
}

::state_machine::ActionResult SelfCheckState::onTick(::state_machine::StateContext& ctx) {
    ::state_machine::Event event(event_type::CONFIG_READY,
                                 ::state_machine::EventTimestamp{runtime_.currentTime()});
    event.category = ::state_machine::EventCategory::kInternal;
    event.source = "self_check_state";
    ctx.postInternalEvent(std::move(event));
    return {};
}

}  // namespace unicycle_reference_trajectory
