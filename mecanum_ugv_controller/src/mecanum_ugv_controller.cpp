#include "mecanum_ugv_controller/mecanum_ugv_controller.h"

#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

#include "mecanum_ugv_controller/state_machine/health_monitor_state.h"
#include "mecanum_ugv_controller/state_machine/hold_state.h"
#include "mecanum_ugv_controller/state_machine/ready_state.h"
#include "mecanum_ugv_controller/state_machine/reset_state.h"
#include "mecanum_ugv_controller/state_machine/self_check_state.h"
#include "mecanum_ugv_controller/state_machine/tracking_state.h"

namespace mecanum_ugv_controller {
namespace {

namespace sm = ::state_machine;

void requireOk(const sm::Status& status, const char* operation) {
    if (!status.ok()) {
        throw std::runtime_error(std::string(operation) + ": " + status.message);
    }
}

}  // namespace

MecanumUgvController::MecanumUgvController(const UgvState& state) : state_(state) {
    setupMachine();
}

void MecanumUgvController::update(double now_sec) {
    current_time_sec_ = now_sec;
    maybeAutoStartTracking();
    if (machine_) {
        (void)machine_->update();
    }
}

::state_machine::Status MecanumUgvController::postEvent(::state_machine::Event event) {
    return machine_->postEvent(std::move(event));
}

ControllerConfig MecanumUgvController::config() const {
    std::lock_guard<std::mutex> lock(config_mutex_);
    return config_;
}

void MecanumUgvController::setConfig(const ControllerConfig& config) {
    std::lock_guard<std::mutex> lock(config_mutex_);
    config_ = config;
}

bool MecanumUgvController::healthReady() const {
    return stateFresh(state_, ros::Time(current_time_sec_), config().state_timeout);
}

bool MecanumUgvController::resetTargetReady() const {
    std::lock_guard<std::mutex> lock(reset_mutex_);
    return reset_target_.valid && std::isfinite(reset_target_.x) &&
           std::isfinite(reset_target_.y) && std::isfinite(reset_target_.yaw);
}

void MecanumUgvController::setResetTarget(ResetTarget target) {
    std::lock_guard<std::mutex> lock(reset_mutex_);
    reset_target_ = target;
}

ResetTarget MecanumUgvController::resetTarget() const {
    std::lock_guard<std::mutex> lock(reset_mutex_);
    return reset_target_;
}

bool MecanumUgvController::worldReferenceReady() const {
    std::lock_guard<std::mutex> lock(reference_mutex_);
    if (!world_reference_.valid || !std::isfinite(world_reference_.vx) ||
        !std::isfinite(world_reference_.vy)) {
        return false;
    }
    constexpr double kFutureStampTolerance = 0.05;
    const double timeout = config().reference_timeout;
    const double age = current_time_sec_ - world_reference_.stamp.toSec();
    return timeout > 0.0 && std::isfinite(age) && age >= -kFutureStampTolerance && age <= timeout;
}

void MecanumUgvController::setWorldReference(WorldVelocityReference reference) {
    std::lock_guard<std::mutex> lock(reference_mutex_);
    world_reference_ = reference;
}

WorldVelocityReference MecanumUgvController::worldReference() const {
    std::lock_guard<std::mutex> lock(reference_mutex_);
    return world_reference_;
}

void MecanumUgvController::setCommand(ControlCommand command) {
    std::lock_guard<std::mutex> lock(command_mutex_);
    command_ = command;
}

ControlCommand MecanumUgvController::command() const {
    std::lock_guard<std::mutex> lock(command_mutex_);
    return command_;
}

void MecanumUgvController::clearCommand() {
    std::lock_guard<std::mutex> lock(command_mutex_);
    command_ = ControlCommand{};
}

void MecanumUgvController::setupMachine() {
    auto builder = sm::StateMachine::builder("MecanumUgvControllerStateMachine");
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
        .state(state_type::Tracking)
        .name("Tracking")
        .impl(std::make_unique<TrackingState>(*this))
        .state(state_type::Hold)
        .name("Hold")
        .impl(std::make_unique<HoldState>(*this))
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
        .from(state_type::Hold)
        .to(state_type::SelfCheck)
        .on(event_type::HEALTH_UNHEALTHY)
        .priority(transition_priority::AUTOMATIC);
    builder.transition()
        .from(state_type::Reset)
        .to(state_type::SelfCheck)
        .on(event_type::HEALTH_UNHEALTHY)
        .priority(transition_priority::AUTOMATIC);
    builder.transition()
        .from(state_type::Tracking)
        .to(state_type::SelfCheck)
        .on(event_type::HEALTH_UNHEALTHY)
        .priority(transition_priority::AUTOMATIC);
    builder.transition()
        .from(state_type::Ready)
        .to(state_type::Hold)
        .on(event_type::HOLD_REQUESTED)
        .priority(transition_priority::COMMAND);
    builder.transition()
        .from(state_type::Reset)
        .to(state_type::Hold)
        .on(event_type::HOLD_REQUESTED)
        .priority(transition_priority::COMMAND);
    builder.transition()
        .from(state_type::Tracking)
        .to(state_type::Hold)
        .on(event_type::HOLD_REQUESTED)
        .priority(transition_priority::COMMAND);
    builder.transition()
        .from(state_type::Tracking)
        .to(state_type::Hold)
        .on(event_type::INPUT_REFERENCE_LOST)
        .priority(transition_priority::AUTOMATIC);
    builder.transition()
        .from(state_type::Ready)
        .to(state_type::Tracking)
        .on(event_type::TRACKING_REQUESTED)
        .priority(transition_priority::COMMAND);
    builder.transition()
        .from(state_type::Hold)
        .to(state_type::Tracking)
        .on(event_type::TRACKING_REQUESTED)
        .priority(transition_priority::COMMAND);
    builder.transition()
        .from(state_type::Reset)
        .to(state_type::Tracking)
        .on(event_type::TRACKING_REQUESTED)
        .priority(transition_priority::COMMAND);
    builder.transition()
        .from(state_type::Ready)
        .to(state_type::Reset)
        .on(event_type::RESET_REQUESTED)
        .priority(transition_priority::COMMAND);
    builder.transition()
        .from(state_type::Hold)
        .to(state_type::Reset)
        .on(event_type::RESET_REQUESTED)
        .priority(transition_priority::COMMAND);
    builder.transition()
        .from(state_type::Tracking)
        .to(state_type::Reset)
        .on(event_type::RESET_REQUESTED)
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

    auto result = builder.build();
    requireOk(result.status, "build Mecanum controller state machine");
    machine_ = std::move(result.value);
    requireOk(machine_->start(), "start Mecanum controller state machine");
}

void MecanumUgvController::maybeAutoStartTracking() {
    const auto cfg = config();
    if (!cfg.auto_start_tracking || !healthReady() || !worldReferenceReady() || !machine_) {
        return;
    }
    const auto control_state = machine_->currentState(region_type::CONTROL);
    if (control_state != state_type::Ready && control_state != state_type::Hold) {
        return;
    }
    ::state_machine::Event event(event_type::TRACKING_REQUESTED,
                                 ::state_machine::EventTimestamp{current_time_sec_});
    event.source = "auto_start_tracking";
    event.category = ::state_machine::EventCategory::kInput;
    (void)machine_->postEvent(std::move(event));
}

}  // namespace mecanum_ugv_controller
