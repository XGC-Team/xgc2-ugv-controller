#pragma once

#include <state_machine/state_machine.hpp>

#include "unicycle_reference_trajectory/state_machine/periodic_gate.h"

namespace unicycle_reference_trajectory {

class ReferenceTrajectoryRuntime;

class ActiveState final : public ::state_machine::State {
   public:
    explicit ActiveState(ReferenceTrajectoryRuntime& runtime);
    std::string name() const override {
        return "Active";
    }

   protected:
    ::state_machine::ActionResult onEnter(::state_machine::StateContext& ctx) override;
    ::state_machine::ActionResult onTick(::state_machine::StateContext& ctx) override;

   private:
    void publishStatusIfDue(::state_machine::StateContext& ctx);
    void publishActiveIfDue(::state_machine::StateContext& ctx);
    ReferenceTrajectoryRuntime& runtime_;
    PeriodicGate status_gate_;
    PeriodicGate active_gate_;
};

}  // namespace unicycle_reference_trajectory
