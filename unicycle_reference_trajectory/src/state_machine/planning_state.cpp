#include "unicycle_reference_trajectory/state_machine/planning_state.h"

#include "unicycle_reference_trajectory/unicycle_reference_trajectory_runtime.h"

namespace unicycle_reference_trajectory {

PlanningState::PlanningState(ReferenceTrajectoryRuntime& runtime) : runtime_(runtime) {}

::state_machine::ActionResult PlanningState::onEnter(::state_machine::StateContext& ctx) {
    runtime_.enterState(unicycle_reference_trajectory_msgs::ReferenceStatus::STATE_PLANNING);
    const bool ok = runtime_.planPendingWaypoint();
    ::state_machine::Event event(ok ? event_type::PLAN_SUCCEEDED : event_type::PLAN_FAILED,
                                 ::state_machine::EventTimestamp{runtime_.currentTime()});
    event.category = ::state_machine::EventCategory::kInternal;
    event.source = "planning_state";
    ctx.postInternalEvent(std::move(event));
    return {};
}

}  // namespace unicycle_reference_trajectory
