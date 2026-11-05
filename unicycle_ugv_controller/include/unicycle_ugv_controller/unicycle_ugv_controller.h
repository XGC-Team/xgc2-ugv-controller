#pragma once

#include <ugv_reset_client/reset_executor.h>

#include <memory>
#include <mutex>
#include <optional>
#include <state_machine/adt_machine.hpp>
#include <string>
#include <vector>

#include "unicycle_ugv_controller/common/operator_command.h"
#include "unicycle_ugv_controller/common/reference_cache.h"
#include "unicycle_ugv_controller/common/types.h"
#include "unicycle_ugv_controller/state_machine/periodic_gate.h"

namespace unicycle_ugv_controller {
class UnicycleUgvController {
   public:
    explicit UnicycleUgvController(const UgvState& state);
    void update(double domain_now, double wall_now);
    void observeInputs(double domain_now, double wall_now);
    void setControlPhase(int64_t phase_ns) {
        control_phase_ns_ = phase_ns;
    }
    // The ROS owner supplies its original wall sampling phase and domain-clock kind.
    void setWallControlPhase(int64_t phase_ns, bool external_domain) {
        wall_control_ = ugv_reset_client::WallControlCadence{phase_ns, config_.control_rate_hz,
                                                             external_domain};
    }
    bool statusDue() const;
    void statusPublished();
    CommandOutcome command(OperatorCommand command, double domain_now, double wall_now,
                           const std::string& source = "command");
    // The only cross-thread handoff is the actual NMPC worker's typed result.
    void postCompletion(NmpcCompletion result);
    const std::vector<ControllerOutput>& outputs() const {
        return outputs_;
    }
    ControlMode mode() const;
    const char* modeName() const;
    ugv_reset_client::NextWake nextWake() const;
    const UgvState& state() const {
        return state_;
    }
    double currentTime() const {
        return current_time_sec_;
    }
    double wallTime() const {
        return wall_time_sec_;
    }
    ControllerConfig config() const {
        return config_;
    }
    void setConfig(const ControllerConfig& config);
    ReferenceCache& referenceCache() {
        return reference_cache_;
    }
    const ReferenceCache& referenceCache() const {
        return reference_cache_;
    }
    bool healthReady() const;
    bool referenceReady() const;
    bool commandReady() const;
    bool resetTargetReady() const;
    void setResetTarget(ResetTarget target) {
        reset_target_ = std::move(target);
    }
    ResetTarget resetTarget() const {
        return reset_target_;
    }
    void setCommand(ControlCommand command);
    ControlCommand command() const {
        return command_;
    }
    void clearCommand() {
        command_ = {};
    }
    ugv_reset_client::ResetExecutor& resetExecutor() {
        return reset_executor_;
    }
    const ugv_reset_client::ResetExecutor& resetExecutor() const {
        return reset_executor_;
    }
    ugv_reset_client::Measurement resetMeasurement() const;
    void onResetGrant(const ugv_reset_client::Grant& grant);
    bool takeResetSession(ugv_reset_client::Session* session);
    bool takeResetResult(ugv_reset_client::Result* result);
    bool peekResetResult(ugv_reset_client::Result* result) const;
    void resetResultPublished();
    void resetSessionPublished();
    void notifyResetZeroPublished(double now, uint32_t generation = 0, uint32_t brake = 0);
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

   private:
    struct SelfCheck {
        PeriodicGate command;
    };
    struct Ready {
        PeriodicGate command;
    };
    struct SolveTicket {
        uint64_t sequence;
        double deadline;
    };
    struct Custom1 {
        PeriodicGate solve, command, zero;
        std::optional<SolveTicket> in_flight;
        WorldPvaReference pva;
        double body_speed, last_tick;
        std::optional<double> sample_wall{};
    };
    struct Reset {
        ugv_reset_client::ResetExecutor::RunPtr run;
        PeriodicGate command;
    };
    struct SetPva {
        WorldPvaReference value;
    };
    struct Tick {};
    struct Exit {};
    struct Health {
        bool ready;
    };
    struct Operator {
        OperatorCommand command;
        CommandOutcome& outcome;
        std::optional<Health> health;
    };
    struct Spec {
        using State = std::variant<SelfCheck, Ready, Custom1, Reset>;
        using Time = double;
        using Change = ::state_machine::adt::Transition<SelfCheck, Ready, Custom1, Reset>;
        static void handle(SelfCheck&, SetPva, UnicycleUgvController&);
        static void handle(Ready&, SetPva, UnicycleUgvController&);
        static void handle(Custom1&, SetPva, UnicycleUgvController&);
        static void handle(Reset&, SetPva, UnicycleUgvController&);
        static bool handle(SelfCheck&, Tick, UnicycleUgvController&);
        static bool handle(Ready&, Tick, UnicycleUgvController&);
        static bool handle(Custom1&, Tick, UnicycleUgvController&);
        static bool handle(Reset&, Tick, UnicycleUgvController&);
        static Change handle(SelfCheck&, Operator, UnicycleUgvController&);
        static Change handle(Ready&, Operator, UnicycleUgvController&);
        static Change handle(Custom1&, Operator, UnicycleUgvController&);
        static Change handle(Reset&, Operator, UnicycleUgvController&);
        static Change handle(SelfCheck&, Health, UnicycleUgvController&);
        static Change handle(Ready&, Health, UnicycleUgvController&);
        static Change handle(Custom1&, Health, UnicycleUgvController&);
        static Change handle(Reset&, Health, UnicycleUgvController&);
        static void handle(SelfCheck&, Exit, UnicycleUgvController&);
        static void handle(Ready&, Exit, UnicycleUgvController&);
        static void handle(Custom1&, Exit, UnicycleUgvController&);
        static void handle(Reset&, Exit, UnicycleUgvController&);
        static void handle(SelfCheck&, const NmpcCompletion&, UnicycleUgvController&);
        static void handle(Ready&, const NmpcCompletion&, UnicycleUgvController&);
        static void handle(Custom1&, const NmpcCompletion&, UnicycleUgvController&);
        static void handle(Reset&, const NmpcCompletion&, UnicycleUgvController&);
    };
    CommandOutcome turn(double domain_now, double wall_now, std::optional<OperatorCommand> command,
                        const std::string& source);
    bool apply(Spec::Change change);
    Spec::Change prepareReset(CommandOutcome& outcome);
    void publishIdle(PeriodicGate& gate);
    void emitZero(Custom1& state, bool force = false);
    void emitCommand(Custom1& state);
    void emitResetZero(Reset& state, bool force = false);
    void enter(SelfCheck&);
    void enter(Ready&);
    void enter(Custom1&);
    void enter(Reset&);
    void maybeUpdatePoseVelocity();
    void applyResetSpec();
    void logResetHold();
    void logResetResult() const;
    double nextControlDeadline() const;
    std::string describeResetAdmissionMiss(const std::string& source) const;

    const UgvState& state_;
    ControllerConfig config_;
    ReferenceCache reference_cache_;
    ControlCommand command_;
    ResetTarget reset_target_;
    ugv_reset_client::ResetExecutor reset_executor_;
    uint32_t reset_session_counter_;
    uint64_t request_sequence_{0};
    WorldPvaReference idle_pva_;
    PoseVelocityEstimator pose_velocity_{};
    ::state_machine::adt::AdtMachine<Spec> machine_{SelfCheck{}, 0.0};
    double current_time_sec_{0.0}, wall_time_sec_{0.0};
    double last_pose_stamp_{0.0};
    bool have_pose_stamp_{false}, last_health_ready_{false}, reset_ever_started_{false};
    std::string last_reset_admission_miss_, last_reset_hold_reason_;
    std::vector<ControllerOutput> outputs_;
    int64_t last_status_ns_{0};
    std::optional<int64_t> control_phase_ns_;
    std::optional<ugv_reset_client::WallControlCadence> wall_control_;
    std::mutex completion_mutex_;
    std::vector<NmpcCompletion> completion_inbox_, completions_;
};
}  // namespace unicycle_ugv_controller
