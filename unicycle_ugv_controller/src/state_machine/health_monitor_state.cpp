#include "unicycle_ugv_controller/state_machine/health_monitor_state.h"

#include <utility>

#include "unicycle_ugv_controller/common/types.h"
#include "unicycle_ugv_controller/unicycle_ugv_controller.h"

namespace unicycle_ugv_controller {

HealthMonitorState::HealthMonitorState(UnicycleUgvController& controller)
    : controller_(controller) {}

::state_machine::ActionResult HealthMonitorState::onTick(::state_machine::StateContext& ctx) {
    const bool ready = controller_.healthReady();
    if (ready != last_ready_) {
        postHealthEvent(ctx, ready ? event_type::HEALTH_READY : event_type::HEALTH_FAULT);
        last_ready_ = ready;
    }
    return {};
}

void HealthMonitorState::postHealthEvent(::state_machine::StateContext& ctx,
                                         ::state_machine::EventId id) const {
    ::state_machine::Event event(id, ::state_machine::EventTimestamp{controller_.currentTime()});
    event.source = "health_monitor";
    event.category = ::state_machine::EventCategory::kInternal;
    (void)ctx.postInternalEvent(std::move(event));
}

}  // namespace unicycle_ugv_controller
