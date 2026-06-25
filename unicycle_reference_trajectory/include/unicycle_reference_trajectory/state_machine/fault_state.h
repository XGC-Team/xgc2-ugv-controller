#pragma once

#include <state_machine/state_machine.hpp>

#include "unicycle_reference_trajectory/state_machine/periodic_gate.h"

namespace unicycle_reference_trajectory {

class ReferenceTrajectoryRuntime;

class FaultState final : public ::state_machine::State {
   public:
    explicit FaultState(ReferenceTrajectoryRuntime& runtime);
    std::string name() const override {
        return "Fault";
    }

   protected:
    ::state_machine::ActionResult onEnter(::state_machine::StateContext& ctx) override;
    ::state_machine::ActionResult onTick(::state_machine::StateContext& ctx) override;

   private:
    ReferenceTrajectoryRuntime& runtime_;
    PeriodicGate status_gate_;
};

}  // namespace unicycle_reference_trajectory
