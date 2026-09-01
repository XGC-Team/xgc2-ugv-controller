#include "unicycle_ugv_controller/state_machine/ready_state.h"

#include "unicycle_ugv_controller/common/types.h"
#include "unicycle_ugv_controller/unicycle_ugv_controller.h"

namespace unicycle_ugv_controller {

ReadyState::ReadyState(UnicycleUgvController& controller) : controller_(controller) {}

::state_machine::ActionResult ReadyState::onEnter(::state_machine::StateContext& ctx) {
    (void)ctx;
    controller_.clearCommand();
    command_gate_.reset();
    return {};
}

::state_machine::ActionResult ReadyState::onTick(::state_machine::StateContext& ctx) {
    emitZeroCommandIfDue(ctx);
    return {};
}

void ReadyState::emitZeroCommandIfDue(::state_machine::StateContext& ctx) {
    const auto cfg = controller_.config();
    if (cfg.placement_idle_silent) {
        return;
    }
    if (command_gate_.due(controller_.currentTime(), 1.0 / cfg.command_publish_rate_hz)) {
        ctx.emitOutput(
            ::state_machine::Event(output_event_type::PUBLISH_ZERO_CMD_VEL,
                                   ::state_machine::EventTimestamp{controller_.currentTime()}));
    }
}

::state_machine::ActionResult ReadyState::onExit(::state_machine::StateContext& ctx) {
    (void)ctx;
    command_gate_.reset();
    return {};
}

}  // namespace unicycle_ugv_controller
