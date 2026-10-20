#pragma once

#include <memory>
#include <mutex>
#include <state_machine/state_machine.hpp>
#include <string>
#include <ugv_reset_client/reset_executor.h>

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
    // The Reset session of this vehicle. The executor owns it (its motion, its deadline, its
    // fail-stops, its one typed Result); the Reset state begins it on entry and ends it on exit.
    // The transport to the station's coordinator publishes what takeResetSession() and
    // takeResetResult() hand out and gives every grant to onResetGrant().
    ugv_reset_client::ResetExecutor& resetExecutor() {
        return reset_executor_;
    }
    const ugv_reset_client::ResetExecutor& resetExecutor() const {
        return reset_executor_;
    }
    ugv_reset_client::Measurement resetMeasurement() const;
    void beginResetSession(const ResetTarget& target);
    void onResetGrant(const ugv_reset_client::Grant& grant);
    bool takeResetSession(ugv_reset_client::Session* session);
    bool takeResetResult(ugv_reset_client::Result* result);
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
    void applyResetSpec();
    void logResetHold();
    std::string describeResetAdmissionMiss(const std::string& source) const;

    const UgvState& state_;
    mutable std::mutex config_mutex_;
    ControllerConfig config_;
    mutable std::mutex command_mutex_;
    ControlCommand command_;
    mutable std::mutex reset_mutex_;
    ResetTarget reset_target_;
    ugv_reset_client::ResetExecutor reset_executor_;
    // Not reused after a restart while the clock is paused: the first session id is random.
    uint32_t reset_session_counter_;
    std::string last_hold_logged_;
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
