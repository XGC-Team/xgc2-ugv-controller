#pragma once

#include <memory>
#include <mutex>
#include <state_machine/state_machine.hpp>
#include <string>

#include "mecanum_ugv_controller/common/types.h"

namespace mecanum_ugv_controller {

class MecanumUgvController {
   public:
    explicit MecanumUgvController(const UgvState& state);
    void update(double now_sec);
    ::state_machine::Status postEvent(::state_machine::Event event);
    const UgvState& state() const {
        return state_;
    }
    double currentTime() const {
        return current_time_sec_;
    }
    ControllerConfig config() const;
    void setConfig(const ControllerConfig& config);
    ::state_machine::StateMachine& stateMachine() {
        return *machine_;
    }
    bool healthReady() const;
    bool resetTargetReady() const;
    void setResetTarget(ResetTarget target);
    ResetTarget resetTarget() const;
    bool worldReferenceReady() const;
    void setWorldReference(WorldVelocityReference reference);
    WorldVelocityReference worldReference() const;
    void setCommand(ControlCommand command);
    ControlCommand command() const;
    void clearCommand();
    // The Reset session. The Reset state begins it on entry and cancels it on exit; the
    // transport to the station's coordinator follows resetSession() and gives every
    // validated response to setResetClearance().
    ResetSession resetSession() const;
    void beginResetSession(const ResetTarget& target);
    void cancelResetSession();
    void setResetClearance(const ResetClearance& clearance);
    // The clearance if it answers the current session and its lease holds at `now_ns`
    // (the controller's clock) and `wall` (seconds); invalid otherwise.
    ResetClearance resetFeedback(uint64_t now_ns, double wall) const;
    const std::string& lastResetAdmissionMiss() const {
        return last_reset_admission_miss_;
    }
    const std::string& lastResetHoldReason() const {
        return last_reset_hold_reason_;
    }
    void setResetHoldReason(std::string reason);

   private:
    void setupMachine();
    void maybeAutoStartCustom1();
    void noteResetAdmissionAfterUpdate();
    std::string describeResetAdmissionMiss(const std::string& source) const;

    const UgvState& state_;
    mutable std::mutex config_mutex_;
    ControllerConfig config_;
    mutable std::mutex command_mutex_;
    ControlCommand command_;
    mutable std::mutex reset_mutex_;
    ResetTarget reset_target_;
    ResetSession reset_session_;
    ResetClearance reset_clearance_;
    // Not reused after a restart while the clock is paused: the first generation is random.
    uint32_t reset_generation_;
    mutable std::mutex reference_mutex_;
    WorldVelocityReference world_reference_;
    std::unique_ptr<::state_machine::StateMachine> machine_;
    double current_time_sec_{0.0};
    bool pending_reset_requested_{false};
    std::string pending_reset_source_;
    std::string last_reset_admission_miss_;
    std::string last_reset_hold_reason_;
};

}  // namespace mecanum_ugv_controller
