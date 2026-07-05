#include "unicycle_ugv_controller/unicycle_ugv_controller.h"

#include <rigid_state_estimator_msgs/PlanarStateEstimate.h>
#include <ros/console.h>

#include <stdexcept>
#include <utility>

#include "unicycle_ugv_controller/state_machine/health_monitor_state.h"
#include "unicycle_ugv_controller/state_machine/hold_state.h"
#include "unicycle_ugv_controller/state_machine/ready_state.h"
#include "unicycle_ugv_controller/state_machine/self_check_state.h"
#include "unicycle_ugv_controller/state_machine/tracking_state.h"

namespace unicycle_ugv_controller {
namespace {

namespace sm = ::state_machine;

void requireOk(const sm::Status& status, const char* operation) {
    if (!status.ok()) {
        throw std::runtime_error(std::string(operation) + ": " + status.message);
    }
}

}  // namespace

UnicycleUgvController::UnicycleUgvController(const UgvState& state) : state_(state) {
    setupMachine();
}

void UnicycleUgvController::update(double now_sec) {
    current_time_sec_ = now_sec;
    if (machine_) {
        maybeAutoStartTracking();
        (void)machine_->update();
    }
}

::state_machine::Status UnicycleUgvController::postEvent(::state_machine::Event event) {
    return machine_->postEvent(std::move(event));
}

ControllerConfig UnicycleUgvController::config() const {
    std::lock_guard<std::mutex> lock(config_mutex_);
    return config_;
}

void UnicycleUgvController::setConfig(const ControllerConfig& config) {
    std::lock_guard<std::mutex> lock(config_mutex_);
    config_ = config;
}

bool UnicycleUgvController::healthReady() const {
    const auto cfg = config();
    return stateFresh(state_, ros::Time(current_time_sec_), cfg.state_timeout) &&
           state_.estimator_state ==
               rigid_state_estimator_msgs::PlanarStateEstimate::STATE_RUNNING &&
           (state_.estimator_flags & rigid_state_estimator_msgs::PlanarStateEstimate::FLAG_FAULT) ==
               0U;
}

bool UnicycleUgvController::referenceReady() const {
    const auto cfg = config();
    return reference_cache_.valid(ros::Time(current_time_sec_), cfg.reference_timeout);
}

void UnicycleUgvController::setCommand(ControlCommand command) {
    std::lock_guard<std::mutex> lock(command_mutex_);
    command_ = command;
}

ControlCommand UnicycleUgvController::command() const {
    std::lock_guard<std::mutex> lock(command_mutex_);
    return command_;
}

void UnicycleUgvController::clearCommand() {
    std::lock_guard<std::mutex> lock(command_mutex_);
    command_ = ControlCommand{};
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
        .state(state_type::Tracking)
        .name("Tracking")
        .impl(std::make_unique<TrackingState>(*this))
        .state(state_type::Hold)
        .name("Hold")
        .impl(std::make_unique<HoldState>(*this))
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
        .from(state_type::Tracking)
        .to(state_type::SelfCheck)
        .on(event_type::HEALTH_UNHEALTHY)
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
        .from(state_type::Tracking)
        .to(state_type::Hold)
        .on(event_type::HOLD_REQUESTED)
        .priority(transition_priority::COMMAND);
    builder.transition()
        .from(state_type::Ready)
        .to(state_type::Hold)
        .on(event_type::HOLD_REQUESTED)
        .priority(transition_priority::COMMAND);
    builder.transition()
        .from(state_type::Tracking)
        .to(state_type::Hold)
        .on(event_type::INPUT_REFERENCE_LOST)
        .priority(transition_priority::AUTOMATIC);
    builder.transition()
        .from(state_type::Hold)
        .to(state_type::SelfCheck)
        .on(event_type::RESET_REQUESTED)
        .priority(transition_priority::COMMAND);
    auto result = builder.build();
    requireOk(result.status, "build UGV controller state machine");
    machine_ = std::move(result.value);
    requireOk(machine_->start(), "start UGV controller state machine");
}

void UnicycleUgvController::maybeAutoStartTracking() {
    const auto cfg = config();
    if (!cfg.auto_start_tracking || !healthReady() || !referenceReady()) {
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
    const auto status = machine_->postEvent(std::move(event));
    if (!status.ok()) {
        ROS_WARN("[UnicycleUgvController] Failed to post auto tracking event: %s",
                 status.message.c_str());
    }
}

}  // namespace unicycle_ugv_controller
