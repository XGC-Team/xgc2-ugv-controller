#include "unicycle_ugv_controller/state_machine/custom1_state.h"

#include <utility>

#include "unicycle_ugv_controller/common/types.h"
#include "unicycle_ugv_controller/unicycle_ugv_controller.h"

namespace unicycle_ugv_controller {

Custom1State::Custom1State(UnicycleUgvController& controller) : controller_(controller) {}

::state_machine::ActionResult Custom1State::onEnter(::state_machine::StateContext& ctx) {
    (void)ctx;
    controller_.clearCommand();
    solve_gate_.reset();
    command_gate_.reset();
    request_sequence_ = 0U;
    in_flight_sequence_ = 0U;
    request_in_flight_ = false;
    request_deadline_ = 0.0;
    had_reference_ = false;
    const UgvState snapshot = controller_.controlState();
    body_speed_ = snapshot.speed;
    last_tick_time_ = controller_.currentTime();
    have_tick_time_ = true;
    return {};
}

::state_machine::ActionResult Custom1State::onEvent(::state_machine::StateContext& ctx,
                                                    const ::state_machine::Event& event) {
    if (controller_.config().tracking_strategy != TrackingStrategy::NMPC) {
        return {};
    }
    if ((event.id == event_type::INPUT_NMPC_SOLVE_SUCCEEDED ||
         event.id == event_type::INPUT_NMPC_SOLVE_FAILED) &&
        event.correlation_id == in_flight_sequence_) {
        request_in_flight_ = false;
    }
    if (event.id == event_type::INPUT_NMPC_SOLVE_SUCCEEDED && hasCommand()) {
        controller_.setCommand(controller_.command());
        ctx.emitOutput(
            ::state_machine::Event(output_event_type::PUBLISH_CMD_VEL,
                                   ::state_machine::EventTimestamp{controller_.currentTime()}));
    } else if ((event.id == event_type::INPUT_NMPC_SOLVE_SUCCEEDED ||
                event.id == event_type::INPUT_NMPC_SOLVE_FAILED) &&
               !hasCommand()) {
        emitZero(ctx);
    }
    return {};
}

::state_machine::ActionResult Custom1State::onTick(::state_machine::StateContext& ctx) {
    if (controller_.config().tracking_strategy == TrackingStrategy::FLATNESS) {
        tickFlatness(ctx);
        return {};
    }
    if (request_in_flight_ && controller_.currentTime() > request_deadline_) {
        request_in_flight_ = false;
    }
    if (!controller_.referenceReady()) {
        emitZero(ctx);
        if (had_reference_) {
            postLost(ctx);
        }
        return {};
    }
    had_reference_ = true;
    requestSolveIfDue(ctx);
    publishNmpcCommandIfDue(ctx);
    return {};
}

void Custom1State::tickFlatness(::state_machine::StateContext& ctx) {
    const double now = controller_.currentTime();
    if (!controller_.worldPvaReady()) {
        emitZero(ctx);
        if (had_reference_) {
            postLost(ctx);
        }
        return;
    }
    had_reference_ = true;
    const UgvState snapshot = controller_.controlState();
    double dt = 0.0;
    if (have_tick_time_) {
        dt = now - last_tick_time_;
    }
    last_tick_time_ = now;
    have_tick_time_ = true;
    const WorldPvaReference lifted = controller_.liftedWorldPva();
    const FlatnessCommandOutput output =
        computeFlatnessCommand(snapshot, lifted, body_speed_, dt, controller_.config());
    if (!output.valid) {
        emitZero(ctx);
        return;
    }
    body_speed_ = output.linear_speed;
    ControlCommand command;
    command.stamp = ros::Time(now);
    command.linear_speed = output.linear_speed;
    command.angular_speed = output.angular_speed;
    command.valid = true;
    controller_.setCommand(command);
    emitCommandIfDue(ctx);
}

void Custom1State::requestSolveIfDue(::state_machine::StateContext& ctx) {
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
    event.source = "custom1_state";
    event.category = ::state_machine::EventCategory::kOutput;
    event.correlation_id = request_sequence_;
    ctx.emitOutput(std::move(event));
}

void Custom1State::publishNmpcCommandIfDue(::state_machine::StateContext& ctx) {
    if (hasCommand()) {
        emitCommandIfDue(ctx);
    } else {
        const auto cfg = controller_.config();
        if (command_gate_.due(controller_.currentTime(), 1.0 / cfg.command_publish_rate_hz)) {
            emitZero(ctx);
        }
    }
}

bool Custom1State::hasCommand() const {
    return controller_.commandReady();
}

void Custom1State::emitCommandIfDue(::state_machine::StateContext& ctx) {
    const auto cfg = controller_.config();
    if (!command_gate_.due(controller_.currentTime(), 1.0 / cfg.command_publish_rate_hz)) {
        return;
    }
    ctx.emitOutput(
        ::state_machine::Event(output_event_type::PUBLISH_CMD_VEL,
                               ::state_machine::EventTimestamp{controller_.currentTime()}));
}

void Custom1State::emitZero(::state_machine::StateContext& ctx) {
    controller_.clearCommand();
    ctx.emitOutput(
        ::state_machine::Event(output_event_type::PUBLISH_ZERO_CMD_VEL,
                               ::state_machine::EventTimestamp{controller_.currentTime()}));
}

void Custom1State::postLost(::state_machine::StateContext& ctx) {
    ::state_machine::Event event(event_type::INPUT_REFERENCE_LOST,
                                 ::state_machine::EventTimestamp{controller_.currentTime()});
    event.source = "custom1_state";
    event.category = ::state_machine::EventCategory::kInternal;
    (void)ctx.postInternalEvent(std::move(event));
}

::state_machine::ActionResult Custom1State::onExit(::state_machine::StateContext& ctx) {
    emitZero(ctx);
    solve_gate_.reset();
    command_gate_.reset();
    request_in_flight_ = false;
    had_reference_ = false;
    have_tick_time_ = false;
    return {};
}

}  // namespace unicycle_ugv_controller
