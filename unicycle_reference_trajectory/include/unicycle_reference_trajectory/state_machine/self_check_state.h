#pragma once

#include <state_machine/state_machine.hpp>

namespace unicycle_reference_trajectory {

class ReferenceTrajectoryRuntime;

class SelfCheckState final : public ::state_machine::State {
   public:
    explicit SelfCheckState(ReferenceTrajectoryRuntime& runtime);
    std::string name() const override {
        return "SelfCheck";
    }

   protected:
    ::state_machine::ActionResult onEnter(::state_machine::StateContext& ctx) override;
    ::state_machine::ActionResult onTick(::state_machine::StateContext& ctx) override;

   private:
    ReferenceTrajectoryRuntime& runtime_;
};

}  // namespace unicycle_reference_trajectory
