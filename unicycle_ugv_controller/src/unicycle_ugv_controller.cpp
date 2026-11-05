#include "unicycle_ugv_controller/unicycle_ugv_controller.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <sstream>
#include <type_traits>
#include <utility>

#include "unicycle_ugv_controller/common/core_log.h"
#include "unicycle_ugv_controller/common/rigid_estimate_health.h"

namespace unicycle_ugv_controller {
namespace adt = ::state_machine::adt;
namespace rc = ugv_reset_client;
UnicycleUgvController::UnicycleUgvController(const UgvState& state)
    : state_(state), reset_session_counter_(std::random_device{}() & 0x7fffffffU) {
    applyResetSpec();
}

void UnicycleUgvController::setConfig(const ControllerConfig& config) {
    config_ = config;
    applyResetSpec();
}
void UnicycleUgvController::setCommand(ControlCommand command) {
    command_ = reset_executor_.requiresZeroCommand() ? ControlCommand{} : command;
}
bool UnicycleUgvController::resetTargetReady() const {
    return reset_target_.valid && std::isfinite(reset_target_.x) &&
           std::isfinite(reset_target_.y) && std::isfinite(reset_target_.yaw);
}
void UnicycleUgvController::postCompletion(NmpcCompletion result) {
    std::lock_guard<std::mutex> lock(completion_mutex_);
    completion_inbox_.push_back(result);
}
void UnicycleUgvController::observeInputs(double now, double wall) {
    current_time_sec_ = now;
    wall_time_sec_ = wall;
    if (!control_phase_ns_) {
        control_phase_ns_ = static_cast<int64_t>(unicycle_ugv_controller::Time(now).toNSec());
    }
    setCoreTime(now);
    maybeUpdatePoseVelocity();
    if (state_.received) {
        reset_executor_.observe(resetMeasurement());
    }
    reset_executor_.poll(resetMeasurement());
}
bool UnicycleUgvController::statusDue() const {
    const auto now =
        static_cast<int64_t>(unicycle_ugv_controller::Time(current_time_sec_).toNSec());
    return now > 0 &&
           (last_status_ns_ == 0 ||
            now - last_status_ns_ >= static_cast<int64_t>(1.0e9 / config_.status_publish_rate_hz));
}
void UnicycleUgvController::statusPublished() {
    last_status_ns_ =
        static_cast<int64_t>(unicycle_ugv_controller::Time(current_time_sec_).toNSec());
}
void UnicycleUgvController::update(double domain_now, double wall_now) {
    (void)turn(domain_now, wall_now, std::nullopt, "");
}
CommandOutcome UnicycleUgvController::command(OperatorCommand command, double domain_now,
                                              double wall_now, const std::string& source) {
    return turn(domain_now, wall_now, command, source);
}
CommandOutcome UnicycleUgvController::turn(double now, double wall,
                                           std::optional<OperatorCommand> input,
                                           const std::string& source) {
    observeInputs(now, wall);
    outputs_.clear();
    completions_.clear();
    {
        std::lock_guard<std::mutex> lock(completion_mutex_);
        completions_.swap(completion_inbox_);
    }
    if (!input && !reset_ever_started_ && !reset_executor_.permissionPending() &&
        config_.auto_start_tracking && healthReady() && referenceReady() &&
        mode() == ControlMode::Ready) {
        input = OperatorCommand::kCustom1;
    }
    const bool healthy = healthReady();
    const bool health_changed = healthy != last_health_ready_;
    last_health_ready_ = healthy;
    // The old control tick precedes selection. Each alternative retains its actual rule order.
    const bool finished = machine_.dispatch(Tick{}, *this);
    CommandOutcome outcome;
    bool changed = false;
    if (input) {
        if (*input == OperatorCommand::kCustom1 &&
            (reset_executor_.permissionPending() || reset_executor_.active())) {
            outcome = {
                CommandOutcome::Code::PermissionActive,
                "Reset permission is active or retirement is not acknowledged; motion refused"};
        } else {
            changed = apply(machine_.dispatch(
                Operator{*input, outcome,
                         health_changed ? std::optional<Health>{Health{healthy}} : std::nullopt},
                *this));
        }
    }
    if (!changed && health_changed) {
        changed = apply(machine_.dispatch(Health{healthy}, *this));
    }
    if (!changed && finished) {
        changed = apply(adt::Enter{Ready{}});
    }
    if (!changed) {
        for (const auto& result : completions_) {
            machine_.dispatch(result, *this);
        }
    }
    if (input == OperatorCommand::kReset) {
        last_reset_admission_miss_ = outcome.ok() ? std::string{}
                                                  : (outcome.code == CommandOutcome::Code::Unmatched
                                                         ? describeResetAdmissionMiss(source)
                                                         : outcome.message);
        if (!outcome.ok()) {
            UGV_LOG_ERROR("[UnicycleUgvController] %s", last_reset_admission_miss_.c_str());
        }
    }
    logResetHold();
    return outcome;
}

bool UnicycleUgvController::apply(Spec::Change change) {
    return std::visit(
        [this](auto& transition) {
            using T = std::decay_t<decltype(transition)>;
            if constexpr (std::is_same_v<T, adt::Stay>) {
                return false;
            } else {
                machine_.dispatch(Exit{}, *this);
                enter(transition.value);
                machine_.apply(std::move(transition), current_time_sec_);
                return true;
            }
        },
        change);
}
void UnicycleUgvController::enter(SelfCheck&) {
    clearCommand();
}
void UnicycleUgvController::enter(Ready&) {
    clearCommand();
}
void UnicycleUgvController::enter(Custom1&) {
    clearCommand();
    idle_pva_ = {};
}
void UnicycleUgvController::enter(Reset& state) {
    clearCommand();
    ++reset_session_counter_;
    reset_ever_started_ = true;
    reset_executor_.install(*state.run, resetMeasurement());
    last_reset_hold_reason_ = resetTargetReady()
                                  ? std::string{}
                                  : "no target: reset_pose cache and reset_initial_* missing";
    if (!resetTargetReady()) {
        UGV_LOG_ERROR(
            "[UnicycleUgvController] Reset entered without a valid goal; "
            "it ends after 2 s unless a cached initialPose arrives first");
    }
    emitResetZero(state, true);
}
UnicycleUgvController::Spec::Change UnicycleUgvController::prepareReset(CommandOutcome& outcome) {
    if (reset_session_counter_ == std::numeric_limits<uint32_t>::max()) {
        outcome = {CommandOutcome::Code::CreditUnavailable,
                   "Reset result credit or authority counter unavailable"};
        return adt::Stay{};
    }
    auto run = reset_executor_.prepare(reset_session_counter_ + 1,
                                       {reset_target_.x, reset_target_.y, reset_target_.yaw},
                                       resetTargetReady(), resetMeasurement());
    if (!run) {
        outcome = {CommandOutcome::Code::CreditUnavailable,
                   "Reset result credit or authority counter unavailable"};
        return adt::Stay{};
    }
    outcome.code = CommandOutcome::Code::Applied;
    return adt::Enter{Reset{std::move(run), {}}};
}
void UnicycleUgvController::publishIdle(PeriodicGate& gate) {
    if (gate.due(current_time_sec_,
                 config_.idle_cmd_rate_hz > 0 ? 1.0 / config_.idle_cmd_rate_hz : 0)) {
        outputs_.emplace_back(PublishZero{current_time_sec_});
    }
}
bool UnicycleUgvController::Spec::handle(SelfCheck& state, Tick, UnicycleUgvController& c) {
    c.publishIdle(state.command);
    return false;
}
bool UnicycleUgvController::Spec::handle(Ready& state, Tick, UnicycleUgvController& c) {
    c.publishIdle(state.command);
    return false;
}
void UnicycleUgvController::emitZero(Custom1& state, bool force) {
    if (force || command_.valid) {
        state.zero.reset();
    }
    state.body_speed = 0;
    clearCommand();
    if (state.zero.due(current_time_sec_, 1.0 / config_.idle_cmd_rate_hz)) {
        outputs_.emplace_back(PublishZero{current_time_sec_});
    }
}
void UnicycleUgvController::emitCommand(Custom1& state) {
    if (!commandReady()) {
        emitZero(state);
        return;
    }
    if (state.command.due(current_time_sec_, 1.0 / config_.command_publish_rate_hz)) {
        outputs_.emplace_back(PublishCommand{current_time_sec_});
    }
}
bool UnicycleUgvController::Spec::handle(Custom1& state, Tick, UnicycleUgvController& c) {
    const auto& cfg = c.config_;
    const double now = c.current_time_sec_;
    if (cfg.tracking_strategy == TrackingStrategy::FLATNESS) {
        if (!c.worldPvaReady()) {
            state.sample_wall.reset();
            c.emitZero(state);
            return false;
        }
        const double dt = now - state.last_tick;
        if (dt >= 0 && dt <= cfg.velocity_dt_min) {
            if (c.wall_control_ && c.wall_control_->external_domain) {
                state.sample_wall.reset();
            }
            c.emitCommand(state);
            return false;
        }
        if (c.wall_control_) {
            if (!c.wall_control_->take(state.sample_wall, c.wall_time_sec_)) {
                if (c.commandReady()) {
                    c.emitCommand(state);
                }
                return false;
            }
            if (!c.wall_control_->external_domain) {
                state.sample_wall = c.wall_control_->next(c.wall_time_sec_);
            }
        }
        state.last_tick = now;
        const auto output =
            computeFlatnessCommand(c.controlState(), c.liftedWorldPva(), state.body_speed, dt, cfg);
        if (!output.valid) {
            c.emitZero(state);
            return false;
        }
        state.body_speed = output.linear_speed;
        ControlCommand command;
        command.stamp = unicycle_ugv_controller::Time(now);
        command.linear_speed = output.linear_speed;
        command.angular_speed = output.angular_speed;
        command.valid = true;
        c.setCommand(command);
        c.emitCommand(state);
        return false;
    }
    if (state.in_flight && now > state.in_flight->deadline) {
        state.in_flight.reset();
    }
    if (!c.referenceReady()) {
        c.emitZero(state);
        return false;
    }
    if (!state.in_flight && state.solve.due(now, 1.0 / cfg.nmpc_request_rate_hz)) {
        state.in_flight = SolveTicket{++c.request_sequence_, now + cfg.solve_timeout};
        c.outputs_.emplace_back(SolveNmpc{c.request_sequence_, now});
    }
    c.emitCommand(state);
    return false;
}
void UnicycleUgvController::emitResetZero(Reset& state, bool force) {
    clearCommand();
    const double period = config_.idle_cmd_rate_hz > 0 ? 1.0 / config_.idle_cmd_rate_hz
                                                       : 1.0 / config_.command_publish_rate_hz;
    const bool due = state.command.due(wall_time_sec_, period);
    if (force || due) {
        outputs_.emplace_back(PublishZero{current_time_sec_});
    }
}
bool UnicycleUgvController::Spec::handle(Reset& state, Tick, UnicycleUgvController& c) {
    const auto step = c.reset_executor_.step(c.resetMeasurement());
    if (step.ended) {
        c.emitResetZero(state);
        c.logResetResult();
        return true;
    }
    if (!c.healthReady()) {
        c.emitResetZero(state);
        return false;
    }
    ControlCommand command;
    command.stamp = unicycle_ugv_controller::Time(c.current_time_sec_);
    command.valid = true;
    command.linear_speed = step.command.vx;
    command.angular_speed = step.command.wz;
    c.setCommand(command);
    if (state.command.due(c.wall_time_sec_, 1.0 / c.config_.command_publish_rate_hz)) {
        c.outputs_.emplace_back(PublishCommand{c.current_time_sec_});
    }
    return false;
}
UnicycleUgvController::Spec::Change UnicycleUgvController::Spec::handle(SelfCheck&, Operator,
                                                                        UnicycleUgvController&) {
    return adt::Stay{};
}
UnicycleUgvController::Spec::Change UnicycleUgvController::Spec::handle(Ready&, Operator input,
                                                                        UnicycleUgvController& c) {
    if (input.command == OperatorCommand::kReset) {
        return c.prepareReset(input.outcome);
    }
    if (input.command == OperatorCommand::kCustom1) {
        input.outcome.code = CommandOutcome::Code::Applied;
        return adt::Enter{
            Custom1{{}, {}, {}, std::nullopt, {}, c.controlState().speed, c.current_time_sec_}};
    }
    return adt::Stay{};
}
UnicycleUgvController::Spec::Change UnicycleUgvController::Spec::handle(Custom1&, Operator input,
                                                                        UnicycleUgvController& c) {
    if (input.command == OperatorCommand::kReset) {
        return c.prepareReset(input.outcome);
    }
    // Actual old registered order is Reset, HealthUnhealthy, Stop. The old global
    // sort did not consistently order same-source rules by the declared priority.
    if (input.health && !input.health->ready) {
        return adt::Enter{SelfCheck{}};
    }
    if (input.command == OperatorCommand::kStop) {
        input.outcome.code = CommandOutcome::Code::Applied;
        return adt::Enter{Ready{}};
    }
    return adt::Stay{};
}
UnicycleUgvController::Spec::Change UnicycleUgvController::Spec::handle(Reset&, Operator input,
                                                                        UnicycleUgvController& c) {
    if (input.command == OperatorCommand::kReset) {
        return c.prepareReset(input.outcome);
    }
    if (input.command == OperatorCommand::kStop) {
        input.outcome.code = CommandOutcome::Code::Applied;
        return adt::Enter{Ready{}};
    }
    return adt::Stay{};
}
UnicycleUgvController::Spec::Change UnicycleUgvController::Spec::handle(SelfCheck&, Health health,
                                                                        UnicycleUgvController&) {
    if (health.ready) {
        return adt::Enter{Ready{}};
    }
    return adt::Stay{};
}
UnicycleUgvController::Spec::Change UnicycleUgvController::Spec::handle(Ready&, Health health,
                                                                        UnicycleUgvController&) {
    if (!health.ready) {
        return adt::Enter{SelfCheck{}};
    }
    return adt::Stay{};
}
UnicycleUgvController::Spec::Change UnicycleUgvController::Spec::handle(Custom1&, Health health,
                                                                        UnicycleUgvController&) {
    if (!health.ready) {
        return adt::Enter{SelfCheck{}};
    }
    return adt::Stay{};
}
UnicycleUgvController::Spec::Change UnicycleUgvController::Spec::handle(Reset&, Health health,
                                                                        UnicycleUgvController&) {
    if (!health.ready) {
        return adt::Enter{SelfCheck{}};
    }
    return adt::Stay{};
}
void UnicycleUgvController::Spec::handle(SelfCheck&, Exit, UnicycleUgvController&) {}
void UnicycleUgvController::Spec::handle(Ready&, Exit, UnicycleUgvController&) {}
void UnicycleUgvController::Spec::handle(Custom1& state, Exit, UnicycleUgvController& c) {
    c.emitZero(state, true);
    c.idle_pva_ = state.pva;
}
void UnicycleUgvController::Spec::handle(Reset& state, Exit, UnicycleUgvController& c) {
    if (c.reset_executor_.active()) {
        const bool healthy = c.healthReady();
        c.reset_executor_.abort(
            healthy ? rc::Outcome::Stopped : rc::Outcome::AbortedUnhealthy,
            healthy ? "stop command" : "the vehicle state is lost or outside the fence",
            c.resetMeasurement());
        UGV_LOG_WARN("[UnicycleUgvController] Reset ended after %.1f s: %s",
                     c.current_time_sec_ - state.run->enteredDomain(),
                     healthy ? "stopped" : "unhealthy");
    }
    c.reset_executor_.finish();
    c.emitResetZero(state, true);
}
void UnicycleUgvController::Spec::handle(SelfCheck&, const NmpcCompletion&,
                                         UnicycleUgvController&) {}
void UnicycleUgvController::Spec::handle(Ready&, const NmpcCompletion&, UnicycleUgvController&) {}
void UnicycleUgvController::Spec::handle(Reset&, const NmpcCompletion&, UnicycleUgvController&) {}
void UnicycleUgvController::Spec::handle(Custom1& state, const NmpcCompletion& result,
                                         UnicycleUgvController& c) {
    if (c.config_.tracking_strategy != TrackingStrategy::NMPC || !state.in_flight ||
        result.sequence != state.in_flight->sequence) {
        return;
    }
    const double deadline = state.in_flight->deadline;
    state.in_flight.reset();
    if (c.current_time_sec_ > deadline) {
        return;
    }
    if (result.command) {
        const auto& command = *result.command;
        const double stamp = command.stamp.toSec();
        if (std::isfinite(stamp) && stamp >= 0 && std::isfinite(command.linear_speed) &&
            std::isfinite(command.angular_speed)) {
            const double age = c.current_time_sec_ - stamp;
            if (age >= -0.05 && age <= c.config_.result_timeout) {
                c.setCommand(command);
                c.emitCommand(state);
                return;
            }
        }
    }
    if (!c.commandReady()) {
        c.emitZero(state);
    }
}

ControlMode UnicycleUgvController::mode() const {
    return std::visit(
        [](const auto& state) {
            using T = std::decay_t<decltype(state)>;
            if constexpr (std::is_same_v<T, SelfCheck>) {
                return ControlMode::SelfCheck;
            } else if constexpr (std::is_same_v<T, Ready>) {
                return ControlMode::Ready;
            } else if constexpr (std::is_same_v<T, Custom1>) {
                return ControlMode::Custom1;
            } else {
                return ControlMode::Reset;
            }
        },
        machine_.state());
}
const char* UnicycleUgvController::modeName() const {
    switch (mode()) {
        case ControlMode::SelfCheck:
            return "SelfCheck";
        case ControlMode::Ready:
            return "Ready";
        case ControlMode::Custom1:
            return "Custom1";
        case ControlMode::Reset:
            return "Reset";
    }
    return "";
}
WorldPvaReference UnicycleUgvController::worldPva() const {
    if (const auto* active = std::get_if<Custom1>(&machine_.state())) {
        return active->pva;
    }
    return idle_pva_;
}
void UnicycleUgvController::setWorldPva(WorldPvaReference reference) {
    machine_.dispatch(SetPva{reference}, *this);
}
void UnicycleUgvController::Spec::handle(SelfCheck&, SetPva input, UnicycleUgvController& c) {
    c.idle_pva_ = input.value;
}
void UnicycleUgvController::Spec::handle(Ready&, SetPva input, UnicycleUgvController& c) {
    c.idle_pva_ = input.value;
}
void UnicycleUgvController::Spec::handle(Reset&, SetPva input, UnicycleUgvController& c) {
    c.idle_pva_ = input.value;
}
void UnicycleUgvController::Spec::handle(Custom1& state, SetPva input, UnicycleUgvController&) {
    state.pva = input.value;
}
bool UnicycleUgvController::worldPvaReady() const {
    return ::unicycle_ugv_controller::worldPvaReady(worldPva());
}
WorldPvaReference UnicycleUgvController::liftedWorldPva() const {
    return liftWorldPva(worldPva(), current_time_sec_);
}
std::string UnicycleUgvController::describeResetAdmissionMiss(const std::string& source) const {
    std::ostringstream text;
    text << "RESET_REQUESTED unmatched topic=" << source << " CONTROL=" << modeName()
         << " health=" << (healthReady() ? "ready" : "unhealthy")
         << " target=" << (resetTargetReady() ? "ready" : "missing");
    if (mode() == ControlMode::SelfCheck && healthReady()) {
        text << " reason=health-ready-but-stuck-in-SelfCheck";
    } else if (mode() == ControlMode::SelfCheck) {
        text << " reason=unhealthy-SelfCheck-has-no-Reset-edge";
    } else {
        text << " reason=no-matching-transition";
    }
    return text.str();
}
void UnicycleUgvController::logResetResult() const {
    const auto& result = reset_executor_.result();
    // Every outcome is on the log with its reason: Ready alone does not say how Reset ended.
    const double seconds = result.duration;
    if (result.outcome == ugv_reset_client::Outcome::Arrived) {
        UGV_LOG_INFO("[UnicycleUgvController] Reset arrived after %.1f s (%.3f m, %.1f deg)",
                     seconds, result.final_planar_error,
                     result.final_yaw_error * 180.0 / 3.14159265358979);
    } else if (result.outcome == ugv_reset_client::Outcome::Timeout) {
        UGV_LOG_ERROR(
            "[UnicycleUgvController] Reset timed out after %.1f s without arriving (%.3f m, %.1f "
            "deg "
            "from the target)",
            seconds, result.final_planar_error, result.final_yaw_error * 180.0 / 3.14159265358979);
    } else {
        UGV_LOG_ERROR("[UnicycleUgvController] Reset ended after %.1f s: %s%s%s", seconds,
                      ugv_reset_client::name(result.outcome), result.detail.empty() ? "" : ": ",
                      result.detail.c_str());
    }
}
void UnicycleUgvController::logResetHold() {
    if (!reset_executor_.active()) {
        return;
    }
    auto* state = std::get_if<Reset>(&machine_.state());
    if (!state) {
        return;
    }
    auto& last = state->run->last_hold_logged;
    std::string text = rc::name(reset_executor_.holdReason());
    if (text.empty()) {
        last.clear();
        return;
    }
    text += ": " + reset_executor_.holdDetail();
    for (const auto& who : reset_executor_.holdBlockers()) {
        text += " [" + who + "]";
    }
    if (text != last) {
        last = text;
        UGV_LOG_INFO("[UnicycleUgvController] Reset held by the coordinator: %s", text.c_str());
    }
}

void UnicycleUgvController::applyResetSpec() {
    const ControllerConfig cfg = config();
    ugv_reset_client::VehicleSpec spec;
    spec.model = ugv_reset_client::Model::Unicycle;
    spec.footprint = {0.5 * cfg.reset_footprint_length, 0.5 * cfg.reset_footprint_width,
                      cfg.reset_footprint_offset_x, cfg.reset_footprint_offset_y};
    spec.max_v = std::min(cfg.reset_max_linear_speed, cfg.chassis_max_linear_speed);
    spec.max_vy = 0.0;
    spec.max_w = std::min(cfg.reset_max_yaw_rate, cfg.chassis_max_yaw_rate);
    spec.acc_v = cfg.reset_linear_acceleration;
    spec.acc_w = cfg.reset_angular_acceleration;
    spec.ell_bound = cfg.reset_lateral_offset_bound;
    reset_executor_.setSpec(spec);
    reset_executor_.setTimeout(cfg.reset_timeout);
}

bool UnicycleUgvController::healthReady() const {
    const auto cfg = config();
    if (!stateFresh(state_, unicycle_ugv_controller::Time(current_time_sec_), cfg.state_timeout) ||
        !insideFence(state_, cfg)) {
        return false;
    }
    if (cfg.state_source != StateSource::STATE_ESTIMATOR) {
        return finitePose(state_);
    }
    return rigidEstimateHealthy(state_.estimator_state, state_.estimator_flags);
}

bool UnicycleUgvController::referenceReady() const {
    const auto cfg = config();
    if (cfg.tracking_strategy == TrackingStrategy::FLATNESS) {
        return worldPvaReady();
    }
    return reference_cache_.valid();
}

bool UnicycleUgvController::commandReady() const {
    constexpr double kFutureStampTolerance = 0.05;
    const auto cfg = config();
    const auto current_command = command();
    const double age = current_time_sec_ - current_command.stamp.toSec();
    return current_command.valid && std::isfinite(current_command.linear_speed) &&
           std::isfinite(current_command.angular_speed) && std::isfinite(age) &&
           age >= -kFutureStampTolerance && age <= cfg.result_timeout;
}

ugv_reset_client::Measurement UnicycleUgvController::resetMeasurement() const {
    ugv_reset_client::Measurement m;
    m.ros_now = current_time_sec_;
    m.wall_now = wall_time_sec_;
    m.pose = {state_.x, state_.y, state_.yaw};
    m.pose_stamp = state_.stamp.toSec();
    m.healthy = healthReady();
    m.pose_stamp_trusted = state_.pose_stamp_trusted;
    const auto outer = command();
    m.external_command_zero = outer.linear_speed == 0.0 && outer.angular_speed == 0.0;
    return m;
}

void UnicycleUgvController::onResetGrant(const ugv_reset_client::Grant& grant) {
    reset_executor_.onGrant(grant, resetMeasurement());
    if (reset_executor_.requiresZeroCommand()) {
        clearCommand();
    }
}

bool UnicycleUgvController::takeResetSession(ugv_reset_client::Session* session) {
    const ugv_reset_client::Measurement m = resetMeasurement();
    if (!reset_executor_.sessionDue(m)) {
        return false;
    }
    *session = reset_executor_.makeSession(m);
    return true;
}

bool UnicycleUgvController::takeResetResult(ugv_reset_client::Result* result) {
    return reset_executor_.takeResult(result);
}

bool UnicycleUgvController::peekResetResult(ugv_reset_client::Result* result) const {
    return reset_executor_.peekResult(result);
}
void UnicycleUgvController::resetResultPublished() {
    reset_executor_.commitResult();
}
void UnicycleUgvController::resetSessionPublished() {
    reset_executor_.markSessionPublished(resetMeasurement());
}
void UnicycleUgvController::notifyResetZeroPublished(double now, uint32_t generation,
                                                     uint32_t brake) {
    if (resetMeasurement().external_command_zero) {
        reset_executor_.onZeroPublished(now, generation, brake);
    }
}

CmdVel UnicycleUgvController::cmdVel() const {
    if (reset_executor_.requiresZeroCommand()) {
        return CmdVel{};
    }
    const ControllerConfig cfg = config();
    const ControlCommand current = command();
    CmdVel out;
    if (!current.valid || !std::isfinite(current.linear_speed) ||
        !std::isfinite(current.angular_speed)) {
        return out;
    }
    const auto control = mode();
    if (control == ControlMode::Custom1 && cfg.tracking_strategy == TrackingStrategy::NMPC) {
        out.linear_x = clamp(current.linear_speed, cfg.min_linear_speed, cfg.max_linear_speed);
        out.angular_z = clamp(current.angular_speed, -cfg.max_angular_speed, cfg.max_angular_speed);
    } else {
        out.linear_x = clamp(current.linear_speed, -cfg.chassis_max_linear_speed,
                             cfg.chassis_max_linear_speed);
        out.angular_z =
            clamp(current.angular_speed, -cfg.chassis_max_yaw_rate, cfg.chassis_max_yaw_rate);
    }
    return out;
}

void UnicycleUgvController::maybeUpdatePoseVelocity() {
    const auto cfg = config();
    if (!usesPoseVelocityFilter(cfg.state_source)) {
        return;
    }
    if (!state_.received || !finitePose(state_)) {
        return;
    }
    const double stamp = state_.stamp.toSec();
    if (have_pose_stamp_ && stamp == last_pose_stamp_) {
        return;
    }
    (void)updatePoseVelocityEstimator(pose_velocity_, stamp, state_.x, state_.y, state_.yaw, cfg);
    last_pose_stamp_ = stamp;
    have_pose_stamp_ = true;
}

UgvState UnicycleUgvController::controlState() const {
    UgvState snapshot = state_;
    const auto cfg = config();
    if (usesPoseVelocityFilter(cfg.state_source)) {
        snapshot.vx = pose_velocity_.vx;
        snapshot.vy = pose_velocity_.vy;
        snapshot.yaw_rate = pose_velocity_.yaw_rate;
        snapshot.speed = bodySpeedFromWorld(snapshot.yaw, snapshot.vx, snapshot.vy);
        snapshot.velocity_valid = pose_velocity_.velocity_valid;
    }
    return snapshot;
}

bool UnicycleUgvController::velocityValid() const {
    if (usesPoseVelocityFilter(config().state_source)) {
        return pose_velocity_.velocity_valid;
    }
    return state_.velocity_valid;
}

double UnicycleUgvController::nextControlDeadline() const {
    const int64_t period = static_cast<int64_t>(std::ceil(1.0e9 / config_.control_rate_hz));
    const int64_t now =
        static_cast<int64_t>(unicycle_ugv_controller::Time(current_time_sec_).toNSec());
    const int64_t origin = control_phase_ns_.value_or(now);
    const int64_t elapsed = now - origin;
    const int64_t frame = elapsed >= 0 ? elapsed / period : (elapsed - period + 1) / period;
    return static_cast<double>(origin + (frame + 1) * period) / 1.0e9;
}
ugv_reset_client::NextWake UnicycleUgvController::nextWake() const {
    auto next = reset_executor_.nextWake(resetMeasurement());
    if (current_time_sec_ > 0) {
        next.atDomain(statusDue()
                          ? current_time_sec_
                          : static_cast<double>(
                                last_status_ns_ +
                                static_cast<int64_t>(1.0e9 / config_.status_publish_rate_hz)) /
                                1.0e9);
    }
    const double never = std::numeric_limits<double>::infinity();
    // Freshness has an existing lower bound too: a future-stamped pose becomes usable at
    // stamp-50ms.
    if (state_.received && state_.stamp.toSec() - 0.05 > current_time_sec_) {
        next.atDomain(state_.stamp.toSec() - 0.05);
    }
    if (state_.received && last_health_ready_) {
        next.atDomain(std::nextafter(state_.stamp.toSec() + config_.state_timeout, never));
    }
    if (command_.valid && command_.stamp.toSec() - 0.05 > current_time_sec_) {
        next.atDomain(command_.stamp.toSec() - 0.05);
    }
    std::visit(
        [&](const auto& state) {
            using T = std::decay_t<decltype(state)>;
            if constexpr (std::is_same_v<T, SelfCheck> || std::is_same_v<T, Ready>) {
                next.atDomain(state.command.next(
                    current_time_sec_,
                    config_.idle_cmd_rate_hz > 0 ? 1.0 / config_.idle_cmd_rate_hz : 0));
            } else if constexpr (std::is_same_v<T, Custom1>) {
                if (config_.tracking_strategy == TrackingStrategy::FLATNESS && worldPvaReady()) {
                    if (wall_control_) {
                        if (state.sample_wall) {
                            next.atWall(*state.sample_wall);
                        } else if (!wall_control_->external_domain) {
                            next.atWall(wall_control_->next(wall_time_sec_));
                        }
                    } else {
                        next.atDomain(nextControlDeadline());
                    }
                }
                if (config_.tracking_strategy == TrackingStrategy::NMPC && referenceReady()) {
                    if (state.in_flight) {
                        next.atDomain(std::nextafter(state.in_flight->deadline, never));
                    } else {
                        next.atDomain(state.solve.next(current_time_sec_,
                                                       1.0 / config_.nmpc_request_rate_hz));
                    }
                }
                if (commandReady()) {
                    next.atDomain(state.command.next(current_time_sec_,
                                                     1.0 / config_.command_publish_rate_hz));
                    next.atDomain(
                        std::nextafter(command_.stamp.toSec() + config_.result_timeout, never));
                } else if (!(wall_control_ &&
                             config_.tracking_strategy == TrackingStrategy::FLATNESS &&
                             worldPvaReady() && state.sample_wall)) {
                    next.atDomain(
                        state.zero.next(current_time_sec_, 1.0 / config_.idle_cmd_rate_hz));
                }
            } else {
                const double rate = healthReady() || config_.idle_cmd_rate_hz <= 0
                                        ? config_.command_publish_rate_hz
                                        : config_.idle_cmd_rate_hz;
                next.atWall(state.command.next(wall_time_sec_, 1.0 / rate));
            }
        },
        machine_.state());
    return next;
}
}  // namespace unicycle_ugv_controller
