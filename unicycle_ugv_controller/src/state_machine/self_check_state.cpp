#include "unicycle_ugv_controller/state_machine/self_check_state.h"

#include "unicycle_ugv_controller/common/types.h"
#include "unicycle_ugv_controller/unicycle_ugv_controller.h"

namespace unicycle_ugv_controller {

SelfCheckState::SelfCheckState(UnicycleUgvController& controller) : controller_(controller) {}

::state_machine::ActionResult SelfCheckState::onEnter(::state_machine::StateContext& ctx) {
    (void)ctx;
    controller_.clearCommand();
    command_gate_.reset();
    return {};
}

::state_machine::ActionResult SelfCheckState::onTick(::state_machine::StateContext& ctx) {
    const auto cfg = controller_.config();
    const double period = cfg.idle_cmd_rate_hz > 0.0 ? 1.0 / cfg.idle_cmd_rate_hz : 0.0;
    if (command_gate_.due(controller_.currentTime(), period)) {
        ctx.emitOutput(
            ::state_machine::Event(output_event_type::PUBLISH_ZERO_CMD_VEL,
                                   ::state_machine::EventTimestamp{controller_.currentTime()}));
    }
    return {};
}

::state_machine::ActionResult SelfCheckState::onExit(::state_machine::StateContext& ctx) {
    (void)ctx;
    command_gate_.reset();
    return {};
}

}  // namespace unicycle_ugv_controller
