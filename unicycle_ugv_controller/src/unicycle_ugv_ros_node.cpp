#include "unicycle_ugv_controller/unicycle_ugv_ros_node.h"

#include <ros1_utils/param_utils.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <utility>

#include "unicycle_ugv_controller/output/cmd_vel_output_consumer.h"
#include "unicycle_ugv_controller/output/nmpc_output_consumer.h"

namespace unicycle_ugv_controller {
namespace {

constexpr uint32_t kMinQueueSize = 1U;

double finitePositiveOr(double value, double fallback) {
    return std::isfinite(value) && value > 0.0 ? value : fallback;
}

}  // namespace

UnicycleUgvRosNode::UnicycleUgvRosNode(ros::NodeHandle& nh)
    : nh_(nh), private_nh_("~"), controller_(state_), output_executor_(nh_) {
    loadParams();
    controller_.setConfig(config_);

    auto post_input_event = [this](::state_machine::Event event) {
        return controller_.postEvent(std::move(event));
    };

    output_dispatcher_.addConsumer(std::make_unique<CmdVelOutputConsumer>(
        nh_, output_executor_, controller_, cmd_vel_topic_, queue_size_));
    output_dispatcher_.addConsumer(
        std::make_unique<NmpcOutputConsumer>(controller_, post_input_event));

    command_input_ = std::make_unique<CommandInputProducer>(nh_, post_input_event, queue_size_);
    state_input_ = std::make_unique<StateInputProducer>(nh_, state_, state_topic_, post_input_event,
                                                        queue_size_);
    reference_input_ = std::make_unique<ReferenceInputProducer>(
        nh_, controller_.referenceCache(), active_analytic_topic_, active_polynomial_topic_,
        active_sampled_topic_, post_input_event, queue_size_);

    output_executor_.start();
    ROS_INFO(
        "[UnicycleUgvRosNode] Initialized: state=%s cmd_vel=%s analytic=%s polynomial=%s "
        "sampled=%s",
        state_topic_.c_str(), cmd_vel_topic_.c_str(), active_analytic_topic_.c_str(),
        active_polynomial_topic_.c_str(), active_sampled_topic_.c_str());
}

UnicycleUgvRosNode::~UnicycleUgvRosNode() {
    output_executor_.stop();
}

void UnicycleUgvRosNode::run(double frequency_hz) {
    const double frequency = finitePositiveOr(frequency_hz, config_.control_rate_hz);
    ROS_INFO("[UnicycleUgvRosNode] Starting main loop at %.1f Hz", frequency);
    ros::Rate rate(frequency);
    while (ros::ok()) {
        ros::spinOnce();
        updateOnce();
        rate.sleep();
    }
}

void UnicycleUgvRosNode::loadParams() {
    int queue_size = static_cast<int>(queue_size_);
    private_nh_.param("queue_size", queue_size, queue_size);
    queue_size_ = std::max(kMinQueueSize, static_cast<uint32_t>(std::max(1, queue_size)));
    private_nh_.param("state_estimate_topic", state_topic_, state_topic_);
    private_nh_.param("active_analytic_topic", active_analytic_topic_, active_analytic_topic_);
    private_nh_.param("active_polynomial_topic", active_polynomial_topic_,
                      active_polynomial_topic_);
    private_nh_.param("active_sampled_topic", active_sampled_topic_, active_sampled_topic_);
    private_nh_.param("cmd_vel_topic", cmd_vel_topic_, cmd_vel_topic_);

    private_nh_.param("control_rate_hz", config_.control_rate_hz, config_.control_rate_hz);
    private_nh_.param("nmpc/control_period", config_.control_period, config_.control_period);
    private_nh_.param("nmpc/prediction_horizon", config_.prediction_horizon,
                      config_.prediction_horizon);
    private_nh_.param("state_timeout", config_.state_timeout, config_.state_timeout);
    private_nh_.param("reference_timeout", config_.reference_timeout, config_.reference_timeout);
    private_nh_.param("nmpc/solve_timeout", config_.solve_timeout, config_.solve_timeout);
    private_nh_.param("command_publish_rate_hz", config_.command_publish_rate_hz,
                      config_.command_publish_rate_hz);
    private_nh_.param("nmpc/request_rate_hz", config_.nmpc_request_rate_hz,
                      config_.nmpc_request_rate_hz);
    private_nh_.param("limits/max_linear_speed", config_.max_linear_speed,
                      config_.max_linear_speed);
    private_nh_.param("limits/min_linear_speed", config_.min_linear_speed,
                      config_.min_linear_speed);
    private_nh_.param("limits/max_angular_speed", config_.max_angular_speed,
                      config_.max_angular_speed);
    private_nh_.param("limits/max_linear_acceleration", config_.max_linear_acceleration,
                      config_.max_linear_acceleration);

    config_.control_rate_hz = finitePositiveOr(config_.control_rate_hz, 100.0);
    config_.control_period = finitePositiveOr(config_.control_period, 0.05);
    config_.prediction_horizon = finitePositiveOr(config_.prediction_horizon, 1.0);
    config_.state_timeout = finitePositiveOr(config_.state_timeout, 0.2);
    config_.reference_timeout = finitePositiveOr(config_.reference_timeout, 0.5);
    config_.solve_timeout = finitePositiveOr(config_.solve_timeout, 0.05);
    config_.command_publish_rate_hz = finitePositiveOr(config_.command_publish_rate_hz, 50.0);
    config_.nmpc_request_rate_hz = finitePositiveOr(config_.nmpc_request_rate_hz, 20.0);
    config_.max_linear_speed = finitePositiveOr(config_.max_linear_speed, 3.0);
    if (!std::isfinite(config_.min_linear_speed) ||
        config_.min_linear_speed >= config_.max_linear_speed) {
        config_.min_linear_speed = -0.5;
    }
    config_.max_angular_speed = finitePositiveOr(config_.max_angular_speed, 2.5);
    config_.max_linear_acceleration = finitePositiveOr(config_.max_linear_acceleration, 2.0);
}

void UnicycleUgvRosNode::updateOnce() {
    controller_.update(ros::Time::now().toSec());
    dispatchOutputEvents(controller_.stateMachine().currentOutputEvents());
}

void UnicycleUgvRosNode::dispatchOutputEvents(const std::vector<::state_machine::Event>& events) {
    const auto result = output_dispatcher_.dispatch(events);
    for (const auto& event : result.unhandled_events) {
        ROS_WARN("[UnicycleUgvRosNode] Unhandled output event id: %u",
                 static_cast<unsigned>(event.id));
    }
    for (const auto& failure : result.failures) {
        ROS_WARN("[UnicycleUgvRosNode] Output consumer '%s' failed on event %u: %s",
                 failure.consumer_name.c_str(), static_cast<unsigned>(failure.event.id),
                 failure.message.c_str());
    }
}

}  // namespace unicycle_ugv_controller
