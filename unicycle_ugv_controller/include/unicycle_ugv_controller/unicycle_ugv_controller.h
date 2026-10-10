#pragma once

#include <memory>
#include <mutex>
#include <state_machine/state_machine.hpp>
#include <string>

#include "unicycle_ugv_controller/common/reference_cache.h"
#include "unicycle_ugv_controller/common/types.h"

namespace unicycle_ugv_controller {

class UnicycleUgvController {
   public:
    explicit UnicycleUgvController(const UgvState& state);
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
    ReferenceCache& referenceCache() {
        return reference_cache_;
    }
    const ReferenceCache& referenceCache() const {
        return reference_cache_;
    }
    ::state_machine::StateMachine& stateMachine() {
        return *machine_;
    }
    bool healthReady() const;
    bool referenceReady() const;
    bool commandReady() const;
    bool resetTargetReady() const;
    void setResetTarget(ResetTarget target);
    ResetTarget resetTarget() const;
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
    // The twist for the chassis from the current command, after the mode's saturation.
    CmdVel cmdVel() const;
    bool worldPvaReady() const;
    void setWorldPva(WorldPvaReference reference);
    WorldPvaReference worldPva() const;
    WorldPvaReference liftedWorldPva() const;
    const PoseVelocityEstimator& poseVelocity() const {
        return pose_velocity_;
    }
    UgvState controlState() const;
    bool velocityValid() const;
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
    void maybeUpdatePoseVelocity();
    void noteResetAdmissionAfterUpdate();
    std::string describeResetAdmissionMiss(const std::string& source) const;

    const UgvState& state_;
    mutable std::mutex config_mutex_;
    ControllerConfig config_;
    ReferenceCache reference_cache_;
    mutable std::mutex command_mutex_;
    ControlCommand command_;
    mutable std::mutex reset_mutex_;
    ResetTarget reset_target_;
    ResetSession reset_session_;
    ResetClearance reset_clearance_;
    // Not reused after a restart while the clock is paused: the first generation is random.
    uint32_t reset_generation_;
    mutable std::mutex pva_mutex_;
    WorldPvaReference world_pva_;
    PoseVelocityEstimator pose_velocity_{};
    std::unique_ptr<::state_machine::StateMachine> machine_;
    double current_time_sec_{0.0};
    double last_pose_stamp_{0.0};
    bool have_pose_stamp_{false};
    bool pending_reset_requested_{false};
    std::string pending_reset_source_;
    std::string last_reset_admission_miss_;
    std::string last_reset_hold_reason_;
};

}  // namespace unicycle_ugv_controller
