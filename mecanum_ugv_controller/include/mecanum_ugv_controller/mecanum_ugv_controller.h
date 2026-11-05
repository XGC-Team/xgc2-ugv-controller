#pragma once

#include <ugv_reset_client/reset_executor.h>

#include <memory>
#include <optional>
#include <state_machine/adt_machine.hpp>
#include <string>
#include <vector>

#include "mecanum_ugv_controller/common/types.h"
#include "mecanum_ugv_controller/state_machine/periodic_gate.h"

namespace mecanum_ugv_controller {
class MecanumUgvController {
   public:
    explicit MecanumUgvController(const UgvState& state);
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
    bool healthReady() const;
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
    bool worldReferenceReady() const;
    void setWorldReference(WorldVelocityReference value) {
        world_reference_ = std::move(value);
    }
    WorldVelocityReference worldReference() const {
        return world_reference_;
    }
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
    struct Custom1 {
        PeriodicGate command;
        std::optional<double> sample_wall{};
    };
    struct Reset {
        ugv_reset_client::ResetExecutor::RunPtr run;
        PeriodicGate command;
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
        static bool handle(SelfCheck&, Tick, MecanumUgvController&);
        static bool handle(Ready&, Tick, MecanumUgvController&);
        static bool handle(Custom1&, Tick, MecanumUgvController&);
        static bool handle(Reset&, Tick, MecanumUgvController&);
        static Change handle(SelfCheck&, Operator, MecanumUgvController&);
        static Change handle(Ready&, Operator, MecanumUgvController&);
        static Change handle(Custom1&, Operator, MecanumUgvController&);
        static Change handle(Reset&, Operator, MecanumUgvController&);
        static Change handle(SelfCheck&, Health, MecanumUgvController&);
        static Change handle(Ready&, Health, MecanumUgvController&);
        static Change handle(Custom1&, Health, MecanumUgvController&);
        static Change handle(Reset&, Health, MecanumUgvController&);
        static void handle(SelfCheck&, Exit, MecanumUgvController&);
        static void handle(Ready&, Exit, MecanumUgvController&);
        static void handle(Custom1&, Exit, MecanumUgvController&);
        static void handle(Reset&, Exit, MecanumUgvController&);
    };
    CommandOutcome turn(double domain_now, double wall_now, std::optional<OperatorCommand> input,
                        const std::string& source);
    bool apply(Spec::Change change);
    Spec::Change prepareReset(CommandOutcome& outcome);
    void publishIdle(PeriodicGate& gate);
    void emitZero();
    void emitResetZero(Reset& state, bool force = false);
    void enter(SelfCheck&);
    void enter(Ready&);
    void enter(Custom1&);
    void enter(Reset&);
    void applyResetSpec();
    void logResetHold();
    void logResetResult() const;
    double nextControlDeadline() const;
    std::string describeResetAdmissionMiss(const std::string& source) const;

    const UgvState& state_;
    ControllerConfig config_;
    ControlCommand command_;
    ResetTarget reset_target_;
    ugv_reset_client::ResetExecutor reset_executor_;
    uint32_t reset_session_counter_;
    WorldVelocityReference world_reference_;
    ::state_machine::adt::AdtMachine<Spec> machine_{SelfCheck{}, 0.0};
    double current_time_sec_{0.0}, wall_time_sec_{0.0};
    bool last_health_ready_{false}, reset_ever_started_{false};
    std::string last_reset_admission_miss_, last_reset_hold_reason_;
    std::vector<ControllerOutput> outputs_;
    PeriodicGate status_gate_;
    std::optional<int64_t> control_phase_ns_;
    std::optional<ugv_reset_client::WallControlCadence> wall_control_;
};
}  // namespace mecanum_ugv_controller
