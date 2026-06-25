#pragma once

#include <state_machine/state_machine.hpp>

#include "unicycle_ugv_controller/state_machine/periodic_gate.h"

namespace unicycle_ugv_controller {

class UnicycleUgvController;

class SelfCheckState final : public ::state_machine::State {
   public:
    explicit SelfCheckState(UnicycleUgvController& controller);
    std::string name() const override {
        return "SelfCheck";
    }
    ::state_machine::ActionResult onEnter(::state_machine::StateContext& ctx) override;
    ::state_machine::ActionResult onTick(::state_machine::StateContext& ctx) override;
    ::state_machine::ActionResult onExit(::state_machine::StateContext& ctx) override;

   private:
    void emitZeroCommandIfDue(::state_machine::StateContext& ctx);

    UnicycleUgvController& controller_;
    PeriodicGate command_gate_;
};

}  // namespace unicycle_ugv_controller
