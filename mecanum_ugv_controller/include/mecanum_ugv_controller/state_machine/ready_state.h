#pragma once

#include <state_machine/state_machine.hpp>

#include "mecanum_ugv_controller/state_machine/periodic_gate.h"

namespace mecanum_ugv_controller {

class MecanumUgvController;

class ReadyState final : public ::state_machine::State {
   public:
    explicit ReadyState(MecanumUgvController& controller);
    std::string name() const override { return "Ready"; }
    ::state_machine::ActionResult onEnter(::state_machine::StateContext& ctx) override;
    ::state_machine::ActionResult onTick(::state_machine::StateContext& ctx) override;
    ::state_machine::ActionResult onExit(::state_machine::StateContext& ctx) override;

   private:
    MecanumUgvController& controller_;
    PeriodicGate command_gate_;
};

}  // namespace mecanum_ugv_controller
