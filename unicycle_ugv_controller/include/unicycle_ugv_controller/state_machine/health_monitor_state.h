#pragma once

#include <state_machine/state_machine.hpp>

namespace unicycle_ugv_controller {

class UnicycleUgvController;

class HealthMonitorState final : public ::state_machine::State {
   public:
    explicit HealthMonitorState(UnicycleUgvController& controller);
    std::string name() const override {
        return "HealthMonitor";
    }
    ::state_machine::ActionResult onTick(::state_machine::StateContext& ctx) override;

   private:
    void postHealthEvent(::state_machine::StateContext& ctx, ::state_machine::EventId id) const;

    UnicycleUgvController& controller_;
    bool last_ready_{false};
};

}  // namespace unicycle_ugv_controller
