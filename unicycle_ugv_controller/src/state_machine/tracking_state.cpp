#include "unicycle_ugv_controller/state_machine/tracking_state.h"

#include <utility>

#include "unicycle_ugv_controller/common/types.h"
#include "unicycle_ugv_controller/unicycle_ugv_controller.h"

namespace unicycle_ugv_controller {

TrackingState::TrackingState(UnicycleUgvController& controller) : controller_(controller) {}

::state_machine::ActionResult TrackingState::onEnter(::state_machine::StateContext& ctx) {
    (void)ctx;
    solve_gate_.reset();
    sequence_ = 0U;
    return {};
}

::state_machine::ActionResult TrackingState::onEvent(::state_machine::StateContext& ctx,
                                                     const ::state_machine::Event& event) {
    if (event.id == event_type::INPUT_NMPC_SOLVE_SUCCEEDED) {
        ctx.emitOutput(
            ::state_machine::Event(output_event_type::PUBLISH_CMD_VEL,
                                   ::state_machine::EventTimestamp{controller_.currentTime()}));
    } else if (event.id == event_type::INPUT_NMPC_SOLVE_FAILED) {
        emitZeroCommand(ctx);
    }
    return {};
}

::state_machine::ActionResult TrackingState::onTick(::state_machine::StateContext& ctx) {
    if (!controller_.referenceReady()) {
        ::state_machine::Event event(event_type::INPUT_REFERENCE_LOST,
                                     ::state_machine::EventTimestamp{controller_.currentTime()});
        event.source = "tracking_state";
        event.category = ::state_machine::EventCategory::kInternal;
        (void)ctx.postInternalEvent(std::move(event));
        emitZeroCommand(ctx);
        return {};
    }
    requestSolveIfDue(ctx);
    return {};
}

void TrackingState::requestSolveIfDue(::state_machine::StateContext& ctx) {
    const auto cfg = controller_.config();
    if (!solve_gate_.due(controller_.currentTime(), 1.0 / cfg.nmpc_request_rate_hz)) {
        return;
    }
    ::state_machine::Event event(output_event_type::REQUEST_NMPC_SOLVE,
                                 ::state_machine::EventTimestamp{controller_.currentTime()});
    event.source = "tracking_state";
    event.category = ::state_machine::EventCategory::kOutput;
    event.correlation_id = ++sequence_;
    ctx.emitOutput(std::move(event));
}

void TrackingState::emitZeroCommand(::state_machine::StateContext& ctx) const {
    ctx.emitOutput(
        ::state_machine::Event(output_event_type::PUBLISH_ZERO_CMD_VEL,
                               ::state_machine::EventTimestamp{controller_.currentTime()}));
}

::state_machine::ActionResult TrackingState::onExit(::state_machine::StateContext& ctx) {
    (void)ctx;
    solve_gate_.reset();
    return {};
}

}  // namespace unicycle_ugv_controller
