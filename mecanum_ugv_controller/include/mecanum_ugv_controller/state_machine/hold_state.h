#pragma once

#include <state_machine/state_machine.hpp>

#include "mecanum_ugv_controller/state_machine/periodic_gate.h"

namespace mecanum_ugv_controller {

class MecanumUgvController;

class HoldState final : public ::state_machine::State {
   public:
    explicit HoldState(MecanumUgvController& controller);
    std::string name() const override { return "Hold"; }
    ::state_machine::ActionResult onEnter(::state_machine::StateContext& ctx) override;
    ::state_machine::ActionResult onTick(::state_machine::StateContext& ctx) override;
    ::state_machine::ActionResult onExit(::state_machine::StateContext& ctx) override;

   private:
    MecanumUgvController& controller_;
    PeriodicGate command_gate_;
};

}  // namespace mecanum_ugv_controller
