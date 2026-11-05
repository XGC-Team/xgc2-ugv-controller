#include "mecanum_ugv_controller/mecanum_ugv_controller.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <sstream>
#include <type_traits>
#include <utility>

#include "mecanum_ugv_controller/common/core_log.h"

namespace mecanum_ugv_controller {
namespace adt = ::state_machine::adt;
namespace rc = ugv_reset_client;
MecanumUgvController::MecanumUgvController(const UgvState& state)
    : state_(state), reset_session_counter_(std::random_device{}() & 0x7fffffffU) {
    applyResetSpec();
}

void MecanumUgvController::setConfig(const ControllerConfig& config) {
    config_ = config;
    applyResetSpec();
}
void MecanumUgvController::setCommand(ControlCommand command) {
    command_ = reset_executor_.requiresZeroCommand() ? ControlCommand{} : command;
}
bool MecanumUgvController::resetTargetReady() const {
    return reset_target_.valid && std::isfinite(reset_target_.x) &&
           std::isfinite(reset_target_.y) && std::isfinite(reset_target_.yaw);
}
void MecanumUgvController::observeInputs(double now, double wall) {
    current_time_sec_ = now;
    wall_time_sec_ = wall;
    if (!control_phase_ns_)
        control_phase_ns_ = static_cast<int64_t>(std::llround(wall * 1.0e9));
    setCoreTime(now);
    if (state_.received)
        reset_executor_.observe(resetMeasurement());
    reset_executor_.poll(resetMeasurement());
}
bool MecanumUgvController::statusDue() const {
    auto probe = status_gate_;
    return probe.due(current_time_sec_, 1.0 / config_.status_publish_rate_hz);
}
void MecanumUgvController::statusPublished() {
    status_gate_.due(current_time_sec_, 1.0 / config_.status_publish_rate_hz);
}
void MecanumUgvController::update(double domain_now, double wall_now) {
    (void)turn(domain_now, wall_now, std::nullopt, "");
}
CommandOutcome MecanumUgvController::command(OperatorCommand command, double domain_now,
                                             double wall_now, const std::string& source) {
    return turn(domain_now, wall_now, command, source);
}
CommandOutcome MecanumUgvController::turn(double now, double wall,
                                          std::optional<OperatorCommand> input,
                                          const std::string& source) {
    observeInputs(now, wall);
    outputs_.clear();
    if (!input && !reset_ever_started_ && !reset_executor_.permissionPending() &&
        config_.auto_start_tracking && healthReady() && worldReferenceReady() &&
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
    if (!changed && health_changed)
        changed = apply(machine_.dispatch(Health{healthy}, *this));
    if (!changed && finished)
        changed = apply(adt::Enter{Ready{}});
    if (input == OperatorCommand::kReset) {
        last_reset_admission_miss_ = outcome.ok() ? std::string{}
                                                  : (outcome.code == CommandOutcome::Code::Unmatched
                                                         ? describeResetAdmissionMiss(source)
                                                         : outcome.message);
        if (!outcome.ok())
            MECANUM_LOG_ERROR("[MecanumUgvController] %s", last_reset_admission_miss_.c_str());
    }
    logResetHold();
    return outcome;
}

bool MecanumUgvController::apply(Spec::Change change) {
    return std::visit(
        [this](auto& transition) {
            using T = std::decay_t<decltype(transition)>;
            if constexpr (std::is_same_v<T, adt::Stay>)
                return false;
            else {
                machine_.dispatch(Exit{}, *this);
                enter(transition.value);
                machine_.apply(std::move(transition), current_time_sec_);
                return true;
            }
        },
        change);
}
void MecanumUgvController::enter(SelfCheck&) {
    clearCommand();
}
void MecanumUgvController::enter(Ready&) {
    clearCommand();
}
void MecanumUgvController::enter(Custom1&) {
    clearCommand();
}
void MecanumUgvController::enter(Reset& state) {
    clearCommand();
    ++reset_session_counter_;
    reset_ever_started_ = true;
    reset_executor_.install(*state.run, resetMeasurement());
    last_reset_hold_reason_ = resetTargetReady()
                                  ? std::string{}
                                  : "no target: reset_pose cache and reset_initial_* missing";
    if (!resetTargetReady())
        MECANUM_LOG_ERROR(
            "[MecanumUgvController] Reset entered without a valid goal; "
            "it ends after 2 s unless a cached initialPose arrives first");
    emitResetZero(state, true);
}
MecanumUgvController::Spec::Change MecanumUgvController::prepareReset(CommandOutcome& outcome) {
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
void MecanumUgvController::publishIdle(PeriodicGate& gate) {
    if (gate.due(current_time_sec_,
                 config_.idle_cmd_rate_hz > 0 ? 1.0 / config_.idle_cmd_rate_hz : 0))
        outputs_.emplace_back(PublishZero{current_time_sec_});
}
bool MecanumUgvController::Spec::handle(SelfCheck& state, Tick, MecanumUgvController& c) {
    c.publishIdle(state.command);
    return false;
}
bool MecanumUgvController::Spec::handle(Ready& state, Tick, MecanumUgvController& c) {
    c.publishIdle(state.command);
    return false;
}
void MecanumUgvController::emitZero() {
    clearCommand();
    outputs_.emplace_back(PublishZero{current_time_sec_});
}
bool MecanumUgvController::Spec::handle(Custom1& state, Tick, MecanumUgvController& c) {
    if (!c.worldReferenceReady()) {
        if (!c.wall_control_ || c.wall_control_->take(state.sample_wall, c.wall_time_sec_)) {
            c.emitZero();
            if (c.wall_control_)
                state.sample_wall = c.wall_control_->next(c.wall_time_sec_);
        }
        return false;
    }
    state.sample_wall.reset();
    const auto output = computeHolonomicTrackCommand(c.state_, c.world_reference_, c.config_);
    ControlCommand command;
    command.stamp = mecanum_ugv_controller::Time(c.current_time_sec_);
    command.linear_x = output.linear_x;
    command.linear_y = output.linear_y;
    command.angular_z = output.angular_z;
    command.valid = true;
    if (state.command.due(c.current_time_sec_, 1.0 / c.config_.command_publish_rate_hz)) {
        c.setCommand(command);
        c.outputs_.emplace_back(PublishCommand{c.current_time_sec_});
    }
    return false;
}
void MecanumUgvController::emitResetZero(Reset& state, bool force) {
    clearCommand();
    const double period = config_.idle_cmd_rate_hz > 0 ? 1.0 / config_.idle_cmd_rate_hz
                                                       : 1.0 / config_.command_publish_rate_hz;
    const bool due = state.command.due(wall_time_sec_, period);
    if (force || due)
        outputs_.emplace_back(PublishZero{current_time_sec_});
}
bool MecanumUgvController::Spec::handle(Reset& state, Tick, MecanumUgvController& c) {
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
    command.stamp = mecanum_ugv_controller::Time(c.current_time_sec_);
    command.valid = true;
    command.linear_x = step.command.vx;
    command.linear_y = step.command.vy;
    command.angular_z = step.command.wz;
    c.setCommand(command);
    if (state.command.due(c.wall_time_sec_, 1.0 / c.config_.command_publish_rate_hz))
        c.outputs_.emplace_back(PublishCommand{c.current_time_sec_});
    return false;
}
MecanumUgvController::Spec::Change MecanumUgvController::Spec::handle(SelfCheck&, Operator,
                                                                      MecanumUgvController&) {
    return adt::Stay{};
}
MecanumUgvController::Spec::Change MecanumUgvController::Spec::handle(Ready&, Operator input,
                                                                      MecanumUgvController& c) {
    if (input.command == OperatorCommand::kReset)
        return c.prepareReset(input.outcome);
    if (input.command == OperatorCommand::kCustom1) {
        input.outcome.code = CommandOutcome::Code::Applied;
        return adt::Enter{Custom1{}};
    }
    return adt::Stay{};
}
MecanumUgvController::Spec::Change MecanumUgvController::Spec::handle(Custom1&, Operator input,
                                                                      MecanumUgvController& c) {
    if (input.command == OperatorCommand::kReset)
        return c.prepareReset(input.outcome);
    // Actual old registered order is Reset, HealthUnhealthy, Stop. The old global
    // sort did not consistently order same-source rules by the declared priority.
    if (input.health && !input.health->ready)
        return adt::Enter{SelfCheck{}};
    if (input.command == OperatorCommand::kStop) {
        input.outcome.code = CommandOutcome::Code::Applied;
        return adt::Enter{Ready{}};
    }
    return adt::Stay{};
}
MecanumUgvController::Spec::Change MecanumUgvController::Spec::handle(Reset&, Operator input,
                                                                      MecanumUgvController& c) {
    if (input.command == OperatorCommand::kReset)
        return c.prepareReset(input.outcome);
    if (input.command == OperatorCommand::kStop) {
        input.outcome.code = CommandOutcome::Code::Applied;
        return adt::Enter{Ready{}};
    }
    return adt::Stay{};
}
MecanumUgvController::Spec::Change MecanumUgvController::Spec::handle(SelfCheck&, Health health,
                                                                      MecanumUgvController&) {
    if (health.ready)
        return adt::Enter{Ready{}};
    return adt::Stay{};
}
MecanumUgvController::Spec::Change MecanumUgvController::Spec::handle(Ready&, Health health,
                                                                      MecanumUgvController&) {
    if (!health.ready)
        return adt::Enter{SelfCheck{}};
    return adt::Stay{};
}
MecanumUgvController::Spec::Change MecanumUgvController::Spec::handle(Custom1&, Health health,
                                                                      MecanumUgvController&) {
    if (!health.ready)
        return adt::Enter{SelfCheck{}};
    return adt::Stay{};
}
MecanumUgvController::Spec::Change MecanumUgvController::Spec::handle(Reset&, Health health,
                                                                      MecanumUgvController&) {
    if (!health.ready)
        return adt::Enter{SelfCheck{}};
    return adt::Stay{};
}
void MecanumUgvController::Spec::handle(SelfCheck&, Exit, MecanumUgvController&) {}
void MecanumUgvController::Spec::handle(Ready&, Exit, MecanumUgvController&) {}
void MecanumUgvController::Spec::handle(Custom1& state, Exit, MecanumUgvController& c) {
    c.emitZero();
}
void MecanumUgvController::Spec::handle(Reset& state, Exit, MecanumUgvController& c) {
    if (c.reset_executor_.active()) {
        const bool healthy = c.healthReady();
        c.reset_executor_.abort(
            healthy ? rc::Outcome::Stopped : rc::Outcome::AbortedUnhealthy,
            healthy ? "stop command" : "the vehicle state is lost or outside the fence",
            c.resetMeasurement());
        MECANUM_LOG_WARN("[MecanumUgvController] Reset ended after %.1f s: %s",
                         c.current_time_sec_ - state.run->enteredDomain(),
                         healthy ? "stopped" : "unhealthy");
    }
    c.reset_executor_.finish();
    c.emitResetZero(state, true);
}
ControlMode MecanumUgvController::mode() const {
    return std::visit(
        [](const auto& state) {
            using T = std::decay_t<decltype(state)>;
            if constexpr (std::is_same_v<T, SelfCheck>)
                return ControlMode::SelfCheck;
            else if constexpr (std::is_same_v<T, Ready>)
                return ControlMode::Ready;
            else if constexpr (std::is_same_v<T, Custom1>)
                return ControlMode::Custom1;
            else
                return ControlMode::Reset;
        },
        machine_.state());
}
const char* MecanumUgvController::modeName() const {
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
bool MecanumUgvController::worldReferenceReady() const {
    return world_reference_.valid && std::isfinite(world_reference_.vx) &&
           std::isfinite(world_reference_.vy);
}
std::string MecanumUgvController::describeResetAdmissionMiss(const std::string& source) const {
    std::ostringstream text;
    text << "RESET_REQUESTED unmatched topic=" << source << " CONTROL=" << modeName()
         << " health=" << (healthReady() ? "ready" : "unhealthy")
         << " target=" << (resetTargetReady() ? "ready" : "missing");
    if (mode() == ControlMode::SelfCheck && healthReady())
        text << " reason=health-ready-but-stuck-in-SelfCheck";
    else if (mode() == ControlMode::SelfCheck)
        text << " reason=unhealthy-SelfCheck-has-no-Reset-edge";
    else
        text << " reason=no-matching-transition";
    return text.str();
}
void MecanumUgvController::logResetResult() const {
    const auto& result = reset_executor_.result();
    // Every outcome is on the log with its reason: Ready alone does not say how Reset ended.
    const double seconds = result.duration;
    if (result.outcome == ugv_reset_client::Outcome::Arrived) {
        MECANUM_LOG_INFO("[MecanumUgvController] Reset arrived after %.1f s (%.3f m, %.1f deg)",
                         seconds, result.final_planar_error,
                         result.final_yaw_error * 180.0 / 3.14159265358979);
    } else if (result.outcome == ugv_reset_client::Outcome::Timeout) {
        MECANUM_LOG_ERROR(
            "[MecanumUgvController] Reset timed out after %.1f s without arriving (%.3f m, %.1f "
            "deg "
            "from the target)",
            seconds, result.final_planar_error, result.final_yaw_error * 180.0 / 3.14159265358979);
    } else {
        MECANUM_LOG_ERROR("[MecanumUgvController] Reset ended after %.1f s: %s%s%s", seconds,
                          ugv_reset_client::name(result.outcome), result.detail.empty() ? "" : ": ",
                          result.detail.c_str());
    }
}
void MecanumUgvController::logResetHold() {
    if (!reset_executor_.active())
        return;
    auto* state = std::get_if<Reset>(&machine_.state());
    if (!state)
        return;
    auto& last = state->run->last_hold_logged;
    std::string text = rc::name(reset_executor_.holdReason());
    if (text.empty()) {
        last.clear();
        return;
    }
    text += ": " + reset_executor_.holdDetail();
    for (const auto& who : reset_executor_.holdBlockers())
        text += " [" + who + "]";
    if (text != last) {
        last = text;
        MECANUM_LOG_INFO("[MecanumUgvController] Reset held by the coordinator: %s", text.c_str());
    }
}

void MecanumUgvController::applyResetSpec() {
    const ControllerConfig cfg = config();
    ugv_reset_client::VehicleSpec spec;
    spec.model = ugv_reset_client::Model::Omni;
    spec.footprint = {0.5 * cfg.reset_footprint_length, 0.5 * cfg.reset_footprint_width,
                      cfg.reset_footprint_offset_x, cfg.reset_footprint_offset_y};
    spec.max_v = std::min(cfg.reset_max_linear_speed, cfg.max_linear_speed);
    spec.max_vy = spec.max_v;
    spec.max_w = std::min(cfg.reset_max_yaw_rate, cfg.max_yaw_rate);
    spec.acc_v = cfg.reset_linear_acceleration;
    spec.acc_w = cfg.reset_angular_acceleration;
    spec.ell_bound = 0.0;
    reset_executor_.setSpec(spec);
    reset_executor_.setTimeout(cfg.reset_timeout);
}

bool MecanumUgvController::healthReady() const {
    const auto cfg = config();
    return stateFresh(state_, mecanum_ugv_controller::Time(current_time_sec_), cfg.state_timeout) &&
           insideFence(state_, cfg);
}

ugv_reset_client::Measurement MecanumUgvController::resetMeasurement() const {
    ugv_reset_client::Measurement m;
    m.ros_now = current_time_sec_;
    m.wall_now = wall_time_sec_;
    m.pose = {state_.x, state_.y, state_.yaw};
    m.pose_stamp = state_.stamp.toSec();
    m.healthy = healthReady();
    m.pose_stamp_trusted = state_.pose_stamp_trusted;
    const auto outer = command();
    m.external_command_zero =
        outer.linear_x == 0.0 && outer.linear_y == 0.0 && outer.angular_z == 0.0;
    return m;
}

void MecanumUgvController::onResetGrant(const ugv_reset_client::Grant& grant) {
    reset_executor_.onGrant(grant, resetMeasurement());
    if (reset_executor_.requiresZeroCommand()) {
        clearCommand();
    }
}

bool MecanumUgvController::takeResetSession(ugv_reset_client::Session* session) {
    const ugv_reset_client::Measurement m = resetMeasurement();
    if (!reset_executor_.sessionDue(m)) {
        return false;
    }
    *session = reset_executor_.makeSession(m);
    return true;
}

bool MecanumUgvController::takeResetResult(ugv_reset_client::Result* result) {
    return reset_executor_.takeResult(result);
}

bool MecanumUgvController::peekResetResult(ugv_reset_client::Result* result) const {
    return reset_executor_.peekResult(result);
}
void MecanumUgvController::resetResultPublished() {
    reset_executor_.commitResult();
}
void MecanumUgvController::resetSessionPublished() {
    reset_executor_.markSessionPublished(resetMeasurement());
}
void MecanumUgvController::notifyResetZeroPublished(double now, uint32_t generation,
                                                    uint32_t brake) {
    if (resetMeasurement().external_command_zero) {
        reset_executor_.onZeroPublished(now, generation, brake);
    }
}

double MecanumUgvController::nextControlDeadline() const {
    const int64_t period = static_cast<int64_t>(std::ceil(1.0e9 / config_.control_rate_hz));
    const int64_t now = static_cast<int64_t>(std::llround(wall_time_sec_ * 1.0e9));
    const int64_t origin = control_phase_ns_.value_or(now);
    const int64_t elapsed = now - origin;
    const int64_t frame = elapsed >= 0 ? elapsed / period : (elapsed - period + 1) / period;
    return static_cast<double>(origin + (frame + 1) * period) / 1.0e9;
}
ugv_reset_client::NextWake MecanumUgvController::nextWake() const {
    auto next = reset_executor_.nextWake(resetMeasurement());
    next.atDomain(status_gate_.next(current_time_sec_, 1.0 / config_.status_publish_rate_hz));
    const double never = std::numeric_limits<double>::infinity();
    // Freshness has an existing lower bound too: a future-stamped pose becomes usable at
    // stamp-50ms.
    if (state_.received && state_.stamp.toSec() - 0.05 > current_time_sec_)
        next.atDomain(state_.stamp.toSec() - 0.05);
    if (state_.received && last_health_ready_)
        next.atDomain(std::nextafter(state_.stamp.toSec() + config_.state_timeout, never));
    std::visit(
        [&](const auto& state) {
            using T = std::decay_t<decltype(state)>;
            if constexpr (std::is_same_v<T, SelfCheck> || std::is_same_v<T, Ready>) {
                next.atDomain(state.command.next(
                    current_time_sec_,
                    config_.idle_cmd_rate_hz > 0 ? 1.0 / config_.idle_cmd_rate_hz : 0));
            } else if constexpr (std::is_same_v<T, Custom1>) {
                // Missing-reference zero is an original output on every nominal control step.
                if (!worldReferenceReady())
                    next.atWall(state.sample_wall.value_or(wall_control_
                                                               ? wall_control_->next(wall_time_sec_)
                                                               : nextControlDeadline()));
                else
                    next.atDomain(state.command.next(current_time_sec_,
                                                     1.0 / config_.command_publish_rate_hz));
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
}  // namespace mecanum_ugv_controller
