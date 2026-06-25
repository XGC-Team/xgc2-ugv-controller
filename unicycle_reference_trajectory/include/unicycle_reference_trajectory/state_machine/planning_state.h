#pragma once

#include <state_machine/state_machine.hpp>

namespace unicycle_reference_trajectory {

class ReferenceTrajectoryRuntime;

class PlanningState final : public ::state_machine::State {
   public:
    explicit PlanningState(ReferenceTrajectoryRuntime& runtime);
    std::string name() const override {
        return "Planning";
    }

   protected:
    ::state_machine::ActionResult onEnter(::state_machine::StateContext& ctx) override;

   private:
    ReferenceTrajectoryRuntime& runtime_;
};

}  // namespace unicycle_reference_trajectory
