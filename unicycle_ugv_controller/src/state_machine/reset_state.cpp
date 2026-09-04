#include "unicycle_ugv_controller/state_machine/reset_state.h"

#include <cmath>

#include "unicycle_ugv_controller/common/types.h"
#include "unicycle_ugv_controller/unicycle_ugv_controller.h"

namespace unicycle_ugv_controller {

ResetState::ResetState(UnicycleUgvController& controller) : controller_(controller) {}

::state_machine::ActionResult ResetState::onEnter(::state_machine::StateContext& ctx) {
    (void)ctx;
    controller_.clearCommand();
    command_gate_.reset();
    enter_time_ = controller_.currentTime();
    last_track_time_ = enter_time_;
    t_along_ = 0.0;
    have_track_time_ = true;
    plan_ = UnicycleBezierPlan{};
    if (!controller_.resetTargetReady()) {
        return {};
    }
    plan_ = planUnicycleReset(controller_.controlState(), controller_.resetTarget(),
                              controller_.config());
    if (plan_.already_arrived) {
        emitZero(ctx);
        postDone(ctx, event_type::RESET_ARRIVED);
        return {};
    }
    if (!plan_.valid) {
        emitZero(ctx);
        postDone(ctx, event_type::RESET_PLAN_FAILED);
        return {};
    }
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
    if (!plan_.valid && !plan_.already_arrived) {
        plan_ = planUnicycleReset(controller_.controlState(), controller_.resetTarget(), cfg);
        if (plan_.already_arrived) {
            emitZero(ctx);
            postDone(ctx, event_type::RESET_ARRIVED);
            return {};
        }
        if (!plan_.valid) {
            emitZero(ctx);
            postDone(ctx, event_type::RESET_PLAN_FAILED);
            return {};
        }
        t_along_ = 0.0;
        last_track_time_ = now;
        have_track_time_ = true;
    }
    if (have_track_time_) {
        const double dt = now - last_track_time_;
        if (std::isfinite(dt) && dt > 0.0) {
            t_along_ += dt;
        }
    }
    last_track_time_ = now;
    have_track_time_ = true;
    const UnicycleResetOutput output =
        trackUnicycleReset(controller_.controlState(), plan_, t_along_, cfg);
    if (output.position_ok) {
        emitZero(ctx);
        postDone(ctx, event_type::RESET_ARRIVED);
        return {};
    }
    ControlCommand command;
    command.stamp = ros::Time(now);
    command.linear_speed = output.linear_speed;
    command.angular_speed = output.angular_speed;
    command.valid = true;
    emitCommand(ctx, command);
    return {};
}

::state_machine::ActionResult ResetState::onExit(::state_machine::StateContext& ctx) {
    emitZero(ctx);
    command_gate_.reset();
    plan_ = UnicycleBezierPlan{};
    have_track_time_ = false;
    t_along_ = 0.0;
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

}  // namespace unicycle_ugv_controller
