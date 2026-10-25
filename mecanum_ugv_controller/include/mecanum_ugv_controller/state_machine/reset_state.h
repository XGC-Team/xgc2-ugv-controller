#pragma once

#include <ugv_reset_client/reset_protocol.h>

#include <state_machine/state_machine.hpp>
#include <string>

#include "mecanum_ugv_controller/common/types.h"
#include "mecanum_ugv_controller/state_machine/periodic_gate.h"

namespace mecanum_ugv_controller {

class MecanumUgvController;

class ResetState final : public ::state_machine::State {
   public:
    explicit ResetState(MecanumUgvController& controller);
    std::string name() const override {
        return "Reset";
    }
    ::state_machine::ActionResult onEnter(::state_machine::StateContext& ctx) override;
    ::state_machine::ActionResult onTick(::state_machine::StateContext& ctx) override;
    ::state_machine::ActionResult onExit(::state_machine::StateContext& ctx) override;

   private:
    void emitCommand(::state_machine::StateContext& ctx, const ControlCommand& command);
    void emitZero(::state_machine::StateContext& ctx, bool force = false);
    void postDone(::state_machine::StateContext& ctx, ::state_machine::EventId id,
                  const ugv_reset_client::Result& result);

    MecanumUgvController& controller_;
    PeriodicGate command_gate_;
    double enter_time_{0.0};
    bool ended_{false};  // the session already ended with a logged outcome
};

}  // namespace mecanum_ugv_controller
