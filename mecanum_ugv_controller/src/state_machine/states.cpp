#include <cmath>
#include <cstdio>
#include <utility>

#include "mecanum_ugv_controller/common/core_log.h"
#include "mecanum_ugv_controller/common/types.h"
#include "mecanum_ugv_controller/common/wall_clock.h"
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
    controller_.clearCommand();
    command_gate_.reset();
    ended_ = false;
    enter_time_ = controller_.currentTime();
    const auto target = controller_.resetTarget();
    controller_.beginResetSession(target);
    if (controller_.resetTargetReady()) {
        controller_.setResetHoldReason({});
    } else {
        // The executor holds the session for 2 s and ends it REJECTED_NO_TARGET.
        controller_.setResetHoldReason("no target: reset_pose cache and reset_initial_* missing");
        MECANUM_LOG_ERROR(
            "[MecanumUgvController] Reset entered without a valid goal; "
            "it ends after 2 s unless a cached initialPose arrives first");
    }
    emitZero(ctx, true);
    return {};
}

::state_machine::ActionResult ResetState::onTick(::state_machine::StateContext& ctx) {
    const auto step = controller_.resetExecutor().step(controller_.resetMeasurement());
    if (step.ended) {
        // The executor ended the session with its one typed result: arrival, the vehicle's own
        // deadline, or a fail-stop. The state machine leaves Reset for Ready (T11) either way.
        const auto& result = controller_.resetExecutor().result();
        emitZero(ctx);
        postDone(ctx,
                 result.outcome == ugv_reset_client::Outcome::Arrived
                     ? event_type::RESET_ARRIVED
                     : result.outcome == ugv_reset_client::Outcome::Timeout
                           ? event_type::RESET_TIMEOUT
                           : event_type::RESET_REJECTED,
                 result);
        return {};
    }
    if (!controller_.healthReady()) {
        emitZero(ctx);
        return {};
    }
    ControlCommand command;
    command.stamp = Time(controller_.currentTime());
    command.valid = true;
    command.linear_x = step.command.vx;
    command.linear_y = step.command.vy;
    command.angular_z = step.command.wz;
    // No grant (or a held one) commands exact zero.
    emitCommand(ctx, command);
    return {};
}

::state_machine::ActionResult ResetState::onExit(::state_machine::StateContext& ctx) {
    if (!ended_) {
        // Left without an outcome of its own: Stop, or the health monitor.
        const bool healthy = controller_.healthReady();
        controller_.resetExecutor().abort(
            healthy ? ugv_reset_client::Outcome::Stopped
                    : ugv_reset_client::Outcome::AbortedUnhealthy,
            healthy ? "stop command" : "the vehicle state is lost or outside the fence",
            controller_.resetMeasurement());
        MECANUM_LOG_WARN("[MecanumUgvController] Reset ended after %.1f s: %s",
                         controller_.currentTime() - enter_time_,
                         healthy ? "stopped" : "unhealthy");
    }
    controller_.resetExecutor().finish();
    emitZero(ctx, true);
    command_gate_.reset();
    return {};
}

void ResetState::emitCommand(::state_machine::StateContext& ctx, const ControlCommand& command) {
    const auto cfg = controller_.config();
    controller_.setCommand(command);
    if (!command_gate_.due(monotonicSeconds(), 1.0 / cfg.command_publish_rate_hz)) {
        return;
    }
    ctx.emitOutput(
        ::state_machine::Event(output_event_type::PUBLISH_CMD_VEL,
                               ::state_machine::EventTimestamp{controller_.currentTime()}));
}

void ResetState::emitZero(::state_machine::StateContext& ctx, bool force) {
    const auto cfg = controller_.config();
    controller_.clearCommand();
    const double period =
        cfg.idle_cmd_rate_hz > 0.0 ? 1.0 / cfg.idle_cmd_rate_hz : 1.0 / cfg.command_publish_rate_hz;
    const bool due = command_gate_.due(monotonicSeconds(), period);
    if (!force && !due) {
        return;
    }
    ctx.emitOutput(
        ::state_machine::Event(output_event_type::PUBLISH_ZERO_CMD_VEL,
                               ::state_machine::EventTimestamp{controller_.currentTime()}));
}

void ResetState::postDone(::state_machine::StateContext& ctx, ::state_machine::EventId id,
                          const ugv_reset_client::Result& result) {
    // Every outcome is on the log with its reason: Ready alone does not say how Reset ended.
    const double seconds = result.duration;
    if (result.outcome == ugv_reset_client::Outcome::Arrived) {
        MECANUM_LOG_INFO("[MecanumUgvController] Reset arrived after %.1f s (%.3f m, %.1f deg)",
                         seconds, result.final_planar_error,
                         result.final_yaw_error * 180.0 / 3.14159265358979);
    } else if (result.outcome == ugv_reset_client::Outcome::Timeout) {
        MECANUM_LOG_ERROR(
            "[MecanumUgvController] Reset timed out after %.1f s without arriving (%.3f m, %.1f "
            "deg "
            "from the target)",
            seconds, result.final_planar_error, result.final_yaw_error * 180.0 / 3.14159265358979);
    } else {
        MECANUM_LOG_ERROR("[MecanumUgvController] Reset ended after %.1f s: %s%s%s", seconds,
                          ugv_reset_client::name(result.outcome), result.detail.empty() ? "" : ": ",
                          result.detail.c_str());
    }
    ended_ = true;
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
    command.stamp = Time(controller_.currentTime());
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
