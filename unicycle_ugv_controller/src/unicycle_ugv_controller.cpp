#include "unicycle_ugv_controller/unicycle_ugv_controller.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <sstream>
#include <stdexcept>
#include <utility>

#include "unicycle_ugv_controller/common/core_log.h"
#include "unicycle_ugv_controller/common/rigid_estimate_health.h"
#include "unicycle_ugv_controller/common/wall_clock.h"
#include "unicycle_ugv_controller/state_machine/custom1_state.h"
#include "unicycle_ugv_controller/state_machine/health_monitor_state.h"
#include "unicycle_ugv_controller/state_machine/ready_state.h"
#include "unicycle_ugv_controller/state_machine/reset_state.h"
#include "unicycle_ugv_controller/state_machine/self_check_state.h"

namespace unicycle_ugv_controller {
namespace {

namespace sm = ::state_machine;

void requireOk(const sm::Status& status, const char* operation) {
    if (!status.ok()) {
        throw std::runtime_error(std::string(operation) + ": " + status.message);
    }
}

}  // namespace

UnicycleUgvController::UnicycleUgvController(const UgvState& state)
    : state_(state), reset_session_counter_(std::random_device{}() & 0x7fffffffU) {
    applyResetSpec();
    setupMachine();
}

void UnicycleUgvController::update(double now_sec) {
    current_time_sec_ = now_sec;
    setCoreTime(now_sec);
    maybeUpdatePoseVelocity();
    // The rotation-centre estimator and the measured velocity run in every CONTROL state: the
    // turns of Custom1 teach the offset before a Reset starts.
    if (state_.received) {
        reset_executor_.observe(resetMeasurement());
    }
    reset_executor_.poll(resetMeasurement());
    maybeAutoStartCustom1();
    if (machine_) {
        (void)machine_->update();
        noteResetAdmissionAfterUpdate();
        logResetHold();
    }
}

::state_machine::Status UnicycleUgvController::postEvent(::state_machine::Event event) {
    const auto id = event.id;
    const std::string source = event.source;
    if (id == event_type::CUSTOM1_REQUESTED && (reset_executor_.permissionPending() || reset_executor_.active())) {
        return sm::Status::error(sm::ErrorCode::kTransitionRejected,
                                 "Reset permission is active or retirement is not acknowledged; motion refused");
    }
    if (id == event_type::RESET_REQUESTED) {
        if (pending_reset_requested_ || reset_session_counter_ == std::numeric_limits<uint32_t>::max() ||
            !reset_executor_.reserveBegin()) {
            return sm::Status::error(sm::ErrorCode::kLimitReached,
                                     "Reset result credit or authority counter unavailable");
        }
        pending_reset_session_ = reset_executor_.sessionId();
        pending_reset_requested_ = true;
        pending_reset_source_ = source.empty() ? "command" : source;
        last_reset_admission_miss_.clear();
    }
    auto status = machine_->postEvent(std::move(event));
    if (id == event_type::RESET_REQUESTED && !status.ok()) {
        reset_executor_.cancelBeginReservation();
        pending_reset_requested_ = false;
        last_reset_admission_miss_ = std::string("Failed to post command event: ") +
                                     status.message + " source=" + pending_reset_source_ +
                                     " CONTROL=" + machine_->currentStateName(region_type::CONTROL);
        UGV_LOG_ERROR("[UnicycleUgvController] %s", last_reset_admission_miss_.c_str());
    }
    return status;
}

void UnicycleUgvController::setResetHoldReason(std::string reason) {
    last_reset_hold_reason_ = std::move(reason);
}

void UnicycleUgvController::noteResetAdmissionAfterUpdate() {
    if (!pending_reset_requested_) {
        return;
    }
    pending_reset_requested_ = false;
    if (reset_executor_.sessionId() != pending_reset_session_) {
        last_reset_admission_miss_.clear();
        return;
    }
    reset_executor_.cancelBeginReservation();
    last_reset_admission_miss_ = describeResetAdmissionMiss(pending_reset_source_);
    UGV_LOG_ERROR("[UnicycleUgvController] %s", last_reset_admission_miss_.c_str());
}

std::string UnicycleUgvController::describeResetAdmissionMiss(const std::string& source) const {
    const std::string control = machine_->currentStateName(region_type::CONTROL);
    const bool health = healthReady();
    const bool target = resetTargetReady();
    std::ostringstream oss;
    oss << "RESET_REQUESTED unmatched topic=" << source << " CONTROL=" << control
        << " health=" << (health ? "ready" : "unhealthy")
        << " target=" << (target ? "ready" : "missing");
    if (control == "SelfCheck" && health) {
        oss << " reason=health-ready-but-stuck-in-SelfCheck";
    } else if (control == "SelfCheck") {
        oss << " reason=unhealthy-SelfCheck-has-no-Reset-edge";
    } else {
        oss << " reason=no-matching-transition";
    }
    return oss.str();
}

ControllerConfig UnicycleUgvController::config() const {
    std::lock_guard<std::mutex> lock(config_mutex_);
    return config_;
}

void UnicycleUgvController::setConfig(const ControllerConfig& config) {
    {
        std::lock_guard<std::mutex> lock(config_mutex_);
        config_ = config;
    }
    applyResetSpec();
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
    if (!stateFresh(state_, Time(current_time_sec_), cfg.state_timeout) ||
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

void UnicycleUgvController::setCommand(ControlCommand command) {
    std::lock_guard<std::mutex> lock(command_mutex_);
    command_ = reset_executor_.requiresZeroCommand() ? ControlCommand{} : command;
}

ControlCommand UnicycleUgvController::command() const {
    std::lock_guard<std::mutex> lock(command_mutex_);
    return command_;
}

void UnicycleUgvController::clearCommand() {
    std::lock_guard<std::mutex> lock(command_mutex_);
    command_ = ControlCommand{};
}

bool UnicycleUgvController::resetTargetReady() const {
    std::lock_guard<std::mutex> lock(reset_mutex_);
    return reset_target_.valid && std::isfinite(reset_target_.x) &&
           std::isfinite(reset_target_.y) && std::isfinite(reset_target_.yaw);
}

void UnicycleUgvController::setResetTarget(ResetTarget target) {
    std::lock_guard<std::mutex> lock(reset_mutex_);
    reset_target_ = target;
}

ResetTarget UnicycleUgvController::resetTarget() const {
    std::lock_guard<std::mutex> lock(reset_mutex_);
    return reset_target_;
}

ugv_reset_client::Measurement UnicycleUgvController::resetMeasurement() const {
    ugv_reset_client::Measurement m;
    m.ros_now = current_time_sec_;
    m.wall_now = monotonicSeconds();
    m.pose = {state_.x, state_.y, state_.yaw};
    m.pose_stamp = state_.stamp.toSec();
    m.healthy = healthReady();
    m.pose_stamp_trusted = state_.pose_stamp_trusted;
    const auto outer = command();
    m.external_command_zero = outer.linear_speed == 0.0 && outer.angular_speed == 0.0;
    return m;
}

void UnicycleUgvController::beginResetSession(const ResetTarget& target) {
    // Admission refused exhaustion before accepting the input; never reuse an old S.
    if (reset_session_counter_ == std::numeric_limits<uint32_t>::max()) return;
    reset_ever_started_ = true;
    reset_executor_.begin(++reset_session_counter_, {target.x, target.y, target.yaw},
                          target.valid && std::isfinite(target.x) && std::isfinite(target.y) &&
                              std::isfinite(target.yaw),
                          resetMeasurement());
}

void UnicycleUgvController::onResetGrant(const ugv_reset_client::Grant& grant) {
    reset_executor_.onGrant(grant, resetMeasurement());
    if (reset_executor_.requiresZeroCommand()) clearCommand();
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
void UnicycleUgvController::resetResultPublished() { reset_executor_.commitResult(); }
void UnicycleUgvController::resetSessionPublished() { reset_executor_.markSessionPublished(resetMeasurement()); }
void UnicycleUgvController::notifyResetZeroPublished(double now, uint32_t generation, uint32_t brake) {
    if (resetMeasurement().external_command_zero) reset_executor_.onZeroPublished(now,generation,brake);
}


// What the coordinator says while it holds the vehicle, once per change.
void UnicycleUgvController::logResetHold() {
    if (!reset_executor_.active()) {
        last_hold_logged_.clear();
        return;
    }
    std::string text = ugv_reset_client::name(reset_executor_.holdReason());
    if (text.empty()) {
        last_hold_logged_.clear();
        return;
    }
    text += ": " + reset_executor_.holdDetail();
    for (const std::string& who : reset_executor_.holdBlockers()) {
        text += " [" + who + "]";
    }
    if (text != last_hold_logged_) {
        last_hold_logged_ = text;
        UGV_LOG_INFO("[UnicycleUgvController] Reset held by the coordinator: %s", text.c_str());
    }
}

CmdVel UnicycleUgvController::cmdVel() const {
    if (reset_executor_.requiresZeroCommand()) return CmdVel{};
    const ControllerConfig cfg = config();
    const ControlCommand current = command();
    CmdVel out;
    if (!current.valid || !std::isfinite(current.linear_speed) ||
        !std::isfinite(current.angular_speed)) {
        return out;
    }
    const auto control = machine_->currentState(region_type::CONTROL);
    if (control == state_type::Custom1 && cfg.tracking_strategy == TrackingStrategy::NMPC) {
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

bool UnicycleUgvController::worldPvaReady() const {
    std::lock_guard<std::mutex> lock(pva_mutex_);
    return ::unicycle_ugv_controller::worldPvaReady(world_pva_);
}

void UnicycleUgvController::setWorldPva(WorldPvaReference reference) {
    std::lock_guard<std::mutex> lock(pva_mutex_);
    world_pva_ = reference;
}

WorldPvaReference UnicycleUgvController::worldPva() const {
    std::lock_guard<std::mutex> lock(pva_mutex_);
    return world_pva_;
}

WorldPvaReference UnicycleUgvController::liftedWorldPva() const {
    std::lock_guard<std::mutex> lock(pva_mutex_);
    return liftWorldPva(world_pva_, current_time_sec_);
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

void UnicycleUgvController::setupMachine() {
    auto builder = sm::StateMachine::builder("UnicycleUgvControllerStateMachine");
    builder.region(region_type::HEALTH)
        .name("health")
        .order(0)
        .initial(state_type::HealthMonitor)
        .state(state_type::HealthMonitor)
        .name("HealthMonitor")
        .impl(std::make_unique<HealthMonitorState>(*this))
        .endRegion()
        .region(region_type::CONTROL)
        .name("control")
        .order(10)
        .initial(state_type::SelfCheck)
        .state(state_type::SelfCheck)
        .name("SelfCheck")
        .impl(std::make_unique<SelfCheckState>(*this))
        .state(state_type::Ready)
        .name("Ready")
        .impl(std::make_unique<ReadyState>(*this))
        .state(state_type::Custom1)
        .name("Custom1")
        .impl(std::make_unique<Custom1State>(*this))
        .state(state_type::Reset)
        .name("Reset")
        .impl(std::make_unique<ResetState>(*this))
        .endRegion();

    builder.transition()
        .from(state_type::SelfCheck)
        .to(state_type::Ready)
        .on(event_type::HEALTH_READY)
        .priority(transition_priority::AUTOMATIC);
    builder.transition()
        .from(state_type::Ready)
        .to(state_type::SelfCheck)
        .on(event_type::HEALTH_UNHEALTHY)
        .priority(transition_priority::AUTOMATIC);
    builder.transition()
        .from(state_type::Reset)
        .to(state_type::SelfCheck)
        .on(event_type::HEALTH_UNHEALTHY)
        .priority(transition_priority::AUTOMATIC);
    builder.transition()
        .from(state_type::Custom1)
        .to(state_type::SelfCheck)
        .on(event_type::HEALTH_UNHEALTHY)
        .priority(transition_priority::AUTOMATIC);
    builder.transition()
        .from(state_type::Reset)
        .to(state_type::Ready)
        .on(event_type::STOP_REQUESTED)
        .priority(transition_priority::COMMAND);
    builder.transition()
        .from(state_type::Custom1)
        .to(state_type::Ready)
        .on(event_type::STOP_REQUESTED)
        .priority(transition_priority::COMMAND);
    builder.transition()
        .from(state_type::Ready)
        .to(state_type::Custom1)
        .on(event_type::CUSTOM1_REQUESTED)
        .when([this](const sm::GuardContext&) { return !reset_executor_.permissionPending(); })
        .priority(transition_priority::COMMAND);
    builder.transition()
        .from(state_type::Ready)
        .to(state_type::Reset)
        .on(event_type::RESET_REQUESTED)
        .priority(transition_priority::COMMAND);
    builder.transition()
        .from(state_type::Custom1)
        .to(state_type::Reset)
        .on(event_type::RESET_REQUESTED)
        .priority(transition_priority::COMMAND);
    builder.transition()
        .from(state_type::Reset)
        .to(state_type::Reset)
        .on(event_type::RESET_REQUESTED)
        .type(sm::TransitionType::kExternalSelf)
        .priority(transition_priority::COMMAND);
    builder.transition()
        .from(state_type::Reset)
        .to(state_type::Ready)
        .on(event_type::RESET_ARRIVED)
        .priority(transition_priority::AUTOMATIC);
    builder.transition()
        .from(state_type::Reset)
        .to(state_type::Ready)
        .on(event_type::RESET_TIMEOUT)
        .priority(transition_priority::AUTOMATIC);
    builder.transition()
        .from(state_type::Reset)
        .to(state_type::Ready)
        .on(event_type::RESET_REJECTED)
        .priority(transition_priority::AUTOMATIC);

    auto result = builder.build();
    requireOk(result.status, "build UGV controller state machine");
    machine_ = std::move(result.value);
    requireOk(machine_->start(), "start UGV controller state machine");
}

void UnicycleUgvController::maybeAutoStartCustom1() {
    const auto cfg = config();
    if (reset_ever_started_ || reset_executor_.permissionPending() || !cfg.auto_start_tracking || !healthReady() || !referenceReady() || !machine_) {
        return;
    }
    if (machine_->currentState(region_type::CONTROL) != state_type::Ready) {
        return;
    }
    ::state_machine::Event event(event_type::CUSTOM1_REQUESTED,
                                 ::state_machine::EventTimestamp{current_time_sec_});
    event.source = "auto_start_tracking";
    event.category = ::state_machine::EventCategory::kInput;
    const auto status = machine_->postEvent(std::move(event));
    if (!status.ok()) {
        UGV_LOG_WARN("[UnicycleUgvController] Failed to post auto Custom1 event: %s",
                     status.message.c_str());
    }
}

}  // namespace unicycle_ugv_controller
