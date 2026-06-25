#pragma once

#include <state_machine/state_machine.hpp>

#include "unicycle_ugv_controller/state_machine/periodic_gate.h"

namespace unicycle_ugv_controller {

class UnicycleUgvController;

class TrackingState final : public ::state_machine::State {
   public:
    explicit TrackingState(UnicycleUgvController& controller);
    std::string name() const override {
        return "Tracking";
    }
    ::state_machine::ActionResult onEnter(::state_machine::StateContext& ctx) override;
    ::state_machine::ActionResult onEvent(::state_machine::StateContext& ctx,
                                          const ::state_machine::Event& event) override;
    ::state_machine::ActionResult onTick(::state_machine::StateContext& ctx) override;
    ::state_machine::ActionResult onExit(::state_machine::StateContext& ctx) override;

   private:
    void requestSolveIfDue(::state_machine::StateContext& ctx);
    void emitZeroCommand(::state_machine::StateContext& ctx) const;

    UnicycleUgvController& controller_;
    PeriodicGate solve_gate_;
    uint64_t sequence_{0U};
};

}  // namespace unicycle_ugv_controller
