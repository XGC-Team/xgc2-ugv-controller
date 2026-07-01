#include "unicycle_ugv_controller/state_machine/tracking_state.h"

#include <utility>

#include "unicycle_ugv_controller/common/types.h"
#include "unicycle_ugv_controller/unicycle_ugv_controller.h"

namespace unicycle_ugv_controller {

TrackingState::TrackingState(UnicycleUgvController& controller) : controller_(controller) {}

::state_machine::ActionResult TrackingState::onEnter(::state_machine::StateContext& ctx) {
    (void)ctx;
    solve_gate_.reset();
    command_gate_.reset();
    request_sequence_ = 0U;
    in_flight_sequence_ = 0U;
    request_in_flight_ = false;
    request_deadline_ = 0.0;
    return {};
}

::state_machine::ActionResult TrackingState::onEvent(::state_machine::StateContext& ctx,
                                                     const ::state_machine::Event& event) {
    if ((event.id == event_type::INPUT_NMPC_SOLVE_SUCCEEDED ||
         event.id == event_type::INPUT_NMPC_SOLVE_FAILED) &&
        event.correlation_id == in_flight_sequence_) {
        request_in_flight_ = false;
    }
    if (event.id == event_type::INPUT_NMPC_SOLVE_SUCCEEDED && hasCommand()) {
        emitCurrentCommand(ctx);
    }
    return {};
}

::state_machine::ActionResult TrackingState::onTick(::state_machine::StateContext& ctx) {
    if (request_in_flight_ && controller_.currentTime() > request_deadline_) {
        request_in_flight_ = false;
    }

    if (!controller_.referenceReady()) {
        ::state_machine::Event event(event_type::INPUT_REFERENCE_LOST,
                                     ::state_machine::EventTimestamp{controller_.currentTime()});
        event.source = "tracking_state";
        event.category = ::state_machine::EventCategory::kInternal;
        (void)ctx.postInternalEvent(std::move(event));
        controller_.clearCommand();
        emitZeroCommand(ctx);
        return {};
    }
    requestSolveIfDue(ctx);
    publishCommandIfDue(ctx);
    return {};
}

void TrackingState::requestSolveIfDue(::state_machine::StateContext& ctx) {
    const auto cfg = controller_.config();
    if (request_in_flight_) {
        return;
    }
    if (!solve_gate_.due(controller_.currentTime(), 1.0 / cfg.nmpc_request_rate_hz)) {
        return;
    }
    ++request_sequence_;
    in_flight_sequence_ = request_sequence_;
    request_in_flight_ = true;
    request_deadline_ = controller_.currentTime() + cfg.solve_timeout;

    ::state_machine::Event event(output_event_type::REQUEST_NMPC_SOLVE,
                                 ::state_machine::EventTimestamp{controller_.currentTime()});
    event.source = "tracking_state";
    event.category = ::state_machine::EventCategory::kOutput;
    event.correlation_id = request_sequence_;
    ctx.emitOutput(std::move(event));
}

void TrackingState::publishCommandIfDue(::state_machine::StateContext& ctx) {
    const auto cfg = controller_.config();
    if (!command_gate_.due(controller_.currentTime(), 1.0 / cfg.command_publish_rate_hz)) {
        return;
    }
    if (hasCommand()) {
        emitCurrentCommand(ctx);
    } else {
        controller_.clearCommand();
        emitZeroCommand(ctx);
    }
}

bool TrackingState::hasCommand() const {
    return controller_.command().valid;
}

void TrackingState::emitZeroCommand(::state_machine::StateContext& ctx) const {
    ctx.emitOutput(
        ::state_machine::Event(output_event_type::PUBLISH_ZERO_CMD_VEL,
                               ::state_machine::EventTimestamp{controller_.currentTime()}));
}

void TrackingState::emitCurrentCommand(::state_machine::StateContext& ctx) const {
    ctx.emitOutput(
        ::state_machine::Event(output_event_type::PUBLISH_CMD_VEL,
                               ::state_machine::EventTimestamp{controller_.currentTime()}));
}

::state_machine::ActionResult TrackingState::onExit(::state_machine::StateContext& ctx) {
    (void)ctx;
    solve_gate_.reset();
    command_gate_.reset();
    request_in_flight_ = false;
    return {};
}

}  // namespace unicycle_ugv_controller
