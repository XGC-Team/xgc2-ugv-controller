#pragma once

#include <state_machine/state_machine.hpp>

namespace mecanum_ugv_controller {

class MecanumUgvController;

class HealthMonitorState final : public ::state_machine::State {
   public:
    explicit HealthMonitorState(MecanumUgvController& controller);
    std::string name() const override { return "HealthMonitor"; }
    ::state_machine::ActionResult onTick(::state_machine::StateContext& ctx) override;

   private:
    void postHealthEvent(::state_machine::StateContext& ctx, ::state_machine::EventId id) const;
    MecanumUgvController& controller_;
    bool last_ready_{false};
};

}  // namespace mecanum_ugv_controller
