#pragma once

#include <state_machine/state_machine.hpp>
#include <string>

#include "mecanum_ugv_controller/common/types.h"
#include "mecanum_ugv_controller/state_machine/periodic_gate.h"

namespace mecanum_ugv_controller {

class MecanumUgvController;

class Custom1State final : public ::state_machine::State {
   public:
    explicit Custom1State(MecanumUgvController& controller);
    std::string name() const override {
        return "Custom1";
    }
    ::state_machine::ActionResult onEnter(::state_machine::StateContext& ctx) override;
    ::state_machine::ActionResult onTick(::state_machine::StateContext& ctx) override;
    ::state_machine::ActionResult onExit(::state_machine::StateContext& ctx) override;

   private:
    void emitCommand(::state_machine::StateContext& ctx, const ControlCommand& command);
    void emitZero(::state_machine::StateContext& ctx);

    MecanumUgvController& controller_;
    PeriodicGate command_gate_;
};

}  // namespace mecanum_ugv_controller
