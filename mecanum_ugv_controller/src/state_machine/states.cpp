#include <utility>

#include "mecanum_ugv_controller/common/types.h"
#include "mecanum_ugv_controller/mecanum_ugv_controller.h"
#include "mecanum_ugv_controller/state_machine/custom1_state.h"
#include "mecanum_ugv_controller/state_machine/health_monitor_state.h"
#include "mecanum_ugv_controller/state_machine/ready_state.h"
#include "mecanum_ugv_controller/state_machine/reset_state.h"
#include "mecanum_ugv_controller/state_machine/self_check_state.h"

namespace mecanum_ugv_controller {
namespace {

void emitZeroIfDue(MecanumUgvController& controller, PeriodicGate& gate,
                   ::state_machine::StateContext& ctx, double rate_hz) {
    const double period = rate_hz > 0.0 ? 1.0 / rate_hz : 0.0;
    if (gate.due(controller.currentTime(), period)) {
        ctx.emitOutput(
            ::state_machine::Event(output_event_type::PUBLISH_ZERO_CMD_VEL,
                                   ::state_machine::EventTimestamp{controller.currentTime()}));
    }
}

}  // namespace

HealthMonitorState::HealthMonitorState(MecanumUgvController& controller)
    : controller_(controller) {}

::state_machine::ActionResult HealthMonitorState::onTick(::state_machine::StateContext& ctx) {
    const bool ready = controller_.healthReady();
    if (ready != last_ready_) {
        postHealthEvent(ctx, ready ? event_type::HEALTH_READY : event_type::HEALTH_UNHEALTHY);
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

SelfCheckState::SelfCheckState(MecanumUgvController& controller) : controller_(controller) {}

::state_machine::ActionResult SelfCheckState::onEnter(::state_machine::StateContext& ctx) {
    (void)ctx;
    controller_.clearCommand();
    command_gate_.reset();
    return {};
}

::state_machine::ActionResult SelfCheckState::onTick(::state_machine::StateContext& ctx) {
    emitZeroIfDue(controller_, command_gate_, ctx, controller_.config().idle_cmd_rate_hz);
    return {};
}

::state_machine::ActionResult SelfCheckState::onExit(::state_machine::StateContext& ctx) {
    (void)ctx;
    command_gate_.reset();
    return {};
}

ReadyState::ReadyState(MecanumUgvController& controller) : controller_(controller) {}

::state_machine::ActionResult ReadyState::onEnter(::state_machine::StateContext& ctx) {
    (void)ctx;
    controller_.clearCommand();
    command_gate_.reset();
    return {};
}

::state_machine::ActionResult ReadyState::onTick(::state_machine::StateContext& ctx) {
    emitZeroIfDue(controller_, command_gate_, ctx, controller_.config().idle_cmd_rate_hz);
    return {};
}

::state_machine::ActionResult ReadyState::onExit(::state_machine::StateContext& ctx) {
    (void)ctx;
    command_gate_.reset();
    return {};
}

ResetState::ResetState(MecanumUgvController& controller) : controller_(controller) {}

::state_machine::ActionResult ResetState::onEnter(::state_machine::StateContext& ctx) {
    (void)ctx;
    controller_.clearCommand();
    command_gate_.reset();
    enter_time_ = controller_.currentTime();
    return {};
}

::state_machine::ActionResult ResetState::onTick(::state_machine::StateContext& ctx) {
    const auto cfg = controller_.config();
    const double now = controller_.currentTime();
    if (cfg.reset_timeout > 0.0 && now - enter_time_ >= cfg.reset_timeout) {
        emitZero(ctx);
        postDone(ctx, event_type::RESET_TIMEOUT);
        return {};
    }
    if (!controller_.resetTargetReady()) {
        emitZero(ctx);
        return {};
    }
    const HolonomicResetOutput output =
        computeHolonomicResetCommand(controller_.state(), controller_.resetTarget(), cfg);
    if (output.position_ok) {
        emitZero(ctx);
        postDone(ctx, event_type::RESET_ARRIVED);
        return {};
    }
    ControlCommand command;
    command.stamp = ros::Time(now);
    command.linear_x = output.linear_x;
    command.linear_y = output.linear_y;
    command.angular_z = output.angular_z;
    command.valid = true;
    emitCommand(ctx, command);
    return {};
}

::state_machine::ActionResult ResetState::onExit(::state_machine::StateContext& ctx) {
    emitZero(ctx);
    command_gate_.reset();
    return {};
}

void ResetState::emitCommand(::state_machine::StateContext& ctx, const ControlCommand& command) {
    const auto cfg = controller_.config();
    if (!command_gate_.due(controller_.currentTime(), 1.0 / cfg.command_publish_rate_hz)) {
        return;
    }
    controller_.setCommand(command);
    ctx.emitOutput(
        ::state_machine::Event(output_event_type::PUBLISH_CMD_VEL,
                               ::state_machine::EventTimestamp{controller_.currentTime()}));
}

void ResetState::emitZero(::state_machine::StateContext& ctx) {
    controller_.clearCommand();
    ctx.emitOutput(
        ::state_machine::Event(output_event_type::PUBLISH_ZERO_CMD_VEL,
                               ::state_machine::EventTimestamp{controller_.currentTime()}));
}

void ResetState::postDone(::state_machine::StateContext& ctx, ::state_machine::EventId id) {
    ::state_machine::Event event(id, ::state_machine::EventTimestamp{controller_.currentTime()});
    event.source = "reset_state";
    event.category = ::state_machine::EventCategory::kInternal;
    (void)ctx.postInternalEvent(std::move(event));
}

Custom1State::Custom1State(MecanumUgvController& controller) : controller_(controller) {}

::state_machine::ActionResult Custom1State::onEnter(::state_machine::StateContext& ctx) {
    (void)ctx;
    controller_.clearCommand();
    command_gate_.reset();
    return {};
}

::state_machine::ActionResult Custom1State::onTick(::state_machine::StateContext& ctx) {
    if (!controller_.worldReferenceReady()) {
        emitZero(ctx);
        return {};
    }
    const HolonomicTrackOutput output = computeHolonomicTrackCommand(
        controller_.state(), controller_.worldReference(), controller_.config());
    ControlCommand command;
    command.stamp = ros::Time(controller_.currentTime());
    command.linear_x = output.linear_x;
    command.linear_y = output.linear_y;
    command.angular_z = output.angular_z;
    command.valid = true;
    emitCommand(ctx, command);
    return {};
}

::state_machine::ActionResult Custom1State::onExit(::state_machine::StateContext& ctx) {
    emitZero(ctx);
    command_gate_.reset();
    return {};
}

void Custom1State::emitCommand(::state_machine::StateContext& ctx, const ControlCommand& command) {
    const auto cfg = controller_.config();
    if (!command_gate_.due(controller_.currentTime(), 1.0 / cfg.command_publish_rate_hz)) {
        return;
    }
    controller_.setCommand(command);
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

}  // namespace mecanum_ugv_controller
