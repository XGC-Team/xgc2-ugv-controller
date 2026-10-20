#include "unicycle_ugv_controller/state_machine/reset_state.h"

#include <cmath>
#include <cstdio>

#include "unicycle_ugv_controller/common/core_log.h"
#include "unicycle_ugv_controller/common/types.h"
#include "unicycle_ugv_controller/common/wall_clock.h"
#include "unicycle_ugv_controller/unicycle_ugv_controller.h"

namespace unicycle_ugv_controller {

ResetState::ResetState(UnicycleUgvController& controller) : controller_(controller) {}

::state_machine::ActionResult ResetState::onEnter(::state_machine::StateContext& ctx) {
    controller_.clearCommand();
    command_gate_.reset();
    ended_ = false;
    enter_time_ = controller_.currentTime();
    enter_wall_ = monotonicSeconds();
    const auto target = controller_.resetTarget();
    if (controller_.resetTargetReady()) {
        controller_.beginResetSession(target);
        controller_.setResetHoldReason({});
    } else {
        controller_.setResetHoldReason("no target: reset_pose cache and reset_initial_* missing");
        UGV_LOG_ERROR(
            "[UnicycleUgvController] Reset entered without a valid goal; "
            "holding Reset until timeout/Stop or a cached initialPose");
    }
    emitZero(ctx, true);
    return {};
}

::state_machine::ActionResult ResetState::onTick(::state_machine::StateContext& ctx) {
    const auto cfg = controller_.config();
    const double now = controller_.currentTime();
    const double wall = monotonicSeconds();
    if (cfg.reset_timeout > 0.0 &&
        (now - enter_time_ >= cfg.reset_timeout || wall - enter_wall_ >= cfg.reset_timeout)) {
        emitZero(ctx);
        postDone(ctx, event_type::RESET_TIMEOUT);
        return {};
    }
    if (!controller_.healthReady()) {
        emitZero(ctx);
        return {};
    }
    if (!controller_.resetSession().active) {
        if (controller_.resetTargetReady()) {
            const auto target = controller_.resetTarget();
            controller_.beginResetSession(target);
            controller_.setResetHoldReason({});
        } else {
            emitZero(ctx);
            UGV_LOG_ERROR_THROTTLE(
                1.0,
                "[UnicycleUgvController] Reset holding with no target (topic=/command "
                "CONTROL=Reset reject=missing-initialPose)");
            return {};
        }
    }
    const auto feedback = controller_.resetFeedback(Time(now).toNSec(), wall);
    if (feedback.valid && feedback.status != ResetClearance::RUNNING) {
        emitZero(ctx);
        postDone(ctx,
                 feedback.status == ResetClearance::ARRIVED ? event_type::RESET_ARRIVED
                                                            : event_type::RESET_REJECTED,
                 feedback.reason);
        return {};
    }
    ControlCommand command;
    command.stamp = Time(now);
    command.valid = true;
    if (feedback.valid) {
        // Refuse a command outside chassis limits; do not silently saturate.
        if (std::abs(feedback.linear_x) > cfg.chassis_max_linear_speed ||
            feedback.linear_y != 0.0 || std::abs(feedback.yaw_rate) > cfg.chassis_max_yaw_rate) {
            emitZero(ctx);
            char detail[160];
            std::snprintf(detail, sizeof(detail),
                          "the coordinator command is outside the chassis limits "
                          "(vx=%.3f vy=%.3f wz=%.3f; limits %.3f m/s, vy 0, %.3f rad/s)",
                          feedback.linear_x, feedback.linear_y, feedback.yaw_rate,
                          cfg.chassis_max_linear_speed, cfg.chassis_max_yaw_rate);
            postDone(ctx, event_type::RESET_REJECTED, detail);
            return {};
        }
        command.linear_speed = feedback.linear_x;
        command.angular_speed = feedback.yaw_rate;
    }
    // Missing/expired coordinator response commands zero, including /clock pause.
    emitCommand(ctx, command);
    return {};
}

::state_machine::ActionResult ResetState::onExit(::state_machine::StateContext& ctx) {
    if (!ended_) {
        // Left without an outcome of its own: Stop, or the health monitor.
        UGV_LOG_WARN("[UnicycleUgvController] Reset ended after %.1f s: stopped or unhealthy",
                     controller_.currentTime() - enter_time_);
    }
    controller_.cancelResetSession();
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
                          const std::string& detail) {
    // Every outcome is on the log with its reason: Ready alone does not say how Reset ended.
    const double seconds = controller_.currentTime() - enter_time_;
    if (id == event_type::RESET_ARRIVED) {
        UGV_LOG_INFO("[UnicycleUgvController] Reset arrived after %.1f s", seconds);
    } else if (id == event_type::RESET_TIMEOUT) {
        UGV_LOG_ERROR("[UnicycleUgvController] Reset timed out after %.1f s without arriving",
                      seconds);
    } else {
        UGV_LOG_ERROR("[UnicycleUgvController] Reset rejected after %.1f s: %s", seconds,
                      detail.empty() ? "the coordinator gave no reason" : detail.c_str());
    }
    ended_ = true;
    ::state_machine::Event event(id, ::state_machine::EventTimestamp{controller_.currentTime()});
    event.source = "reset_state";
    event.category = ::state_machine::EventCategory::kInternal;
    (void)ctx.postInternalEvent(std::move(event));
}

}  // namespace unicycle_ugv_controller
