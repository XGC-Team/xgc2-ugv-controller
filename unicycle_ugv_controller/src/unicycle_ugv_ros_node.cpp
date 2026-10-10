#include "unicycle_ugv_controller/unicycle_ugv_ros_node.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <utility>

#include "unicycle_ugv_controller/config_loader.h"
#include "unicycle_ugv_controller/output/cmd_vel_output_consumer.h"
#include "unicycle_ugv_controller/output/nmpc_output_consumer.h"
#include "unicycle_ugv_controller/ros_time_conversion.h"

namespace unicycle_ugv_controller {
namespace {

constexpr uint32_t kMinQueueSize = 1U;

double finitePositiveOr(double value, double fallback) {
    return std::isfinite(value) && value > 0.0 ? value : fallback;
}

// The node's private parameters as a configuration source (config_loader.h).
class RosParamSource {
   public:
    explicit RosParamSource(const ros::NodeHandle& nh) : nh_(nh) {}

    bool get(const std::string& key, double& value) const {
        return nh_.getParam(key, value);
    }
    bool get(const std::string& key, bool& value) const {
        return nh_.getParam(key, value);
    }
    bool get(const std::string& key, std::string& value) const {
        return nh_.getParam(key, value);
    }
    bool has(const std::string& key) const {
        return nh_.hasParam(key);
    }
    bool isMapping(const std::string& key) const {
        XmlRpc::XmlRpcValue value;
        return nh_.getParam(key, value) && value.getType() == XmlRpc::XmlRpcValue::TypeStruct;
    }
    std::vector<std::string> keys(const std::string& key) const {
        std::vector<std::string> names;
        XmlRpc::XmlRpcValue value;
        if (nh_.getParam(key, value) && value.getType() == XmlRpc::XmlRpcValue::TypeStruct) {
            for (auto it = value.begin(); it != value.end(); ++it) {
                names.push_back(it->first);
            }
        }
        return names;
    }

   private:
    const ros::NodeHandle& nh_;
};

// The protocol's status values are the controller's.
static_assert(static_cast<int>(ResetClearance::RUNNING) ==
                  static_cast<int>(ugv_reset_client::ResetLease::RUNNING),
              "");
static_assert(static_cast<int>(ResetClearance::ARRIVED) ==
                  static_cast<int>(ugv_reset_client::ResetLease::ARRIVED),
              "");
static_assert(static_cast<int>(ResetClearance::REJECTED) ==
                  static_cast<int>(ugv_reset_client::ResetLease::REJECTED),
              "");

ResetClearance toResetClearance(const ugv_reset_client::ResetLease::Clearance& clearance) {
    ResetClearance out;
    out.generation = clearance.generation;
    out.stamp_ns = clearance.stamp;
    out.issue_wall = clearance.issue_wall;
    out.status = static_cast<ResetClearance::Status>(clearance.status);
    out.linear_x = clearance.command.x;
    out.linear_y = clearance.command.y;
    out.yaw_rate = clearance.command.yaw;
    out.lease_seconds = clearance.lease_seconds;
    return out;
}

}  // namespace

UnicycleUgvRosNode::UnicycleUgvRosNode(ros::NodeHandle& nh)
    : nh_(nh),
      private_nh_("~"),
      controller_(state_),
      reset_client_(nh_,
                    [this](const ugv_reset_client::ResetLease::Clearance& clearance) {
                        controller_.setResetClearance(toResetClearance(clearance));
                    }),
      output_executor_(nh_) {
    loadParams();
    controller_.setConfig(config_);
    seedResetTarget();

    auto post_input_event = [this](::state_machine::Event event) {
        return controller_.postEvent(std::move(event));
    };

    output_dispatcher_.addConsumer(std::make_unique<CmdVelOutputConsumer>(
        nh_, output_executor_, controller_, reset_client_, cmd_vel_topic_, queue_size_));
    if (config_.tracking_strategy == TrackingStrategy::NMPC) {
        output_dispatcher_.addConsumer(
            std::make_unique<NmpcOutputConsumer>(nh_, controller_, post_input_event, queue_size_));
        reference_input_ = std::make_unique<ReferenceInputProducer>(
            nh_, controller_.referenceCache(), active_analytic_topic_, active_polynomial_topic_,
            active_sampled_topic_, post_input_event, queue_size_);
    } else {
        pva_reference_input_ = std::make_unique<PvaReferenceInputProducer>(
            nh_, controller_, pva_reference_topic_, post_input_event, queue_size_);
    }

    control_state_pub_ = nh_.advertise<std_msgs::String>(control_state_topic_, queue_size_);

    command_input_ = std::make_unique<CommandInputProducer>(nh_, post_input_event, queue_size_);
    state_input_ =
        std::make_unique<StateInputProducer>(nh_, state_, config_.state_source, state_topic_,
                                             platform_pose_topic_, post_input_event, queue_size_);
    reset_target_input_ = std::make_unique<ResetTargetInputProducer>(
        nh_, controller_, reset_pose_topic_, post_input_event, queue_size_);

    output_executor_.start();
    ROS_INFO(
        "[UnicycleUgvRosNode] Initialized: strategy=%s state_source=%s state=%s pose=%s "
        "pva=%s reset_pose=%s cmd_vel=%s",
        config_.tracking_strategy == TrackingStrategy::FLATNESS ? "flatness" : "nmpc",
        config_.state_source == StateSource::PLATFORM_POSE ? "platform_pose" : "state_estimator",
        state_topic_.c_str(), platform_pose_topic_.c_str(), pva_reference_topic_.c_str(),
        reset_pose_topic_.c_str(), cmd_vel_topic_.c_str());
}

UnicycleUgvRosNode::~UnicycleUgvRosNode() {
    output_executor_.stop();
}

void UnicycleUgvRosNode::run(double frequency_hz) {
    const double frequency = finitePositiveOr(frequency_hz, config_.control_rate_hz);
    ROS_INFO("[UnicycleUgvRosNode] Starting main loop at %.1f Hz", frequency);
    ros::WallRate rate(frequency);
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
    private_nh_.param("platform_pose_topic", platform_pose_topic_, platform_pose_topic_);
    private_nh_.param("reset_pose_topic", reset_pose_topic_, reset_pose_topic_);
    private_nh_.param("active_analytic_topic", active_analytic_topic_, active_analytic_topic_);
    private_nh_.param("active_polynomial_topic", active_polynomial_topic_,
                      active_polynomial_topic_);
    private_nh_.param("active_sampled_topic", active_sampled_topic_, active_sampled_topic_);
    private_nh_.param("pva_reference_topic", pva_reference_topic_, pva_reference_topic_);
    private_nh_.param("cmd_vel_topic", cmd_vel_topic_, cmd_vel_topic_);
    private_nh_.param("control_state_topic", control_state_topic_, control_state_topic_);
    private_nh_.param("status_publish_rate_hz", status_publish_rate_hz_, status_publish_rate_hz_);
    loadControllerConfig(RosParamSource(private_nh_), config_);
    status_publish_rate_hz_ = finitePositiveOr(status_publish_rate_hz_, 5.0);
}

void UnicycleUgvRosNode::seedResetTarget() {
    const ResetTarget target = initialResetTarget(config_);
    if (target.valid) {
        controller_.setResetTarget(target);
    }
}

void UnicycleUgvRosNode::updateOnce() {
    controller_.update(ros::Time::now().toSec());
    if (!controller_.lastResetAdmissionMiss().empty() &&
        controller_.lastResetAdmissionMiss() != last_logged_reset_miss_) {
        last_logged_reset_miss_ = controller_.lastResetAdmissionMiss();
        ROS_ERROR("[UnicycleUgvRosNode] %s", last_logged_reset_miss_.c_str());
    }
    dispatchOutputEvents(controller_.stateMachine().currentOutputEvents());
    const ResetSession reset = controller_.resetSession();
    reset_client_.update(reset.active, reset.generation,
                         {reset.target.x, reset.target.y, reset.target.yaw},
                         {state_.x, state_.y, state_.yaw}, toRosTime(state_.stamp),
                         controller_.healthReady());
    const auto control_state = controller_.stateMachine().currentState(region_type::CONTROL);
    const auto health_state = controller_.stateMachine().currentState(region_type::HEALTH);
    logStateChanges(control_state, health_state);
    publishStatusIfDue(ros::Time::now());
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

void UnicycleUgvRosNode::publishStatusIfDue(const ros::Time& now) {
    if (now.isZero() || !control_state_pub_) {
        return;
    }
    const double period = 1.0 / status_publish_rate_hz_;
    if (!last_status_stamp_.isZero() && (now - last_status_stamp_).toSec() < period) {
        return;
    }
    last_status_stamp_ = now;
    std_msgs::String control_state;
    control_state.data = controller_.stateMachine().currentStateName(region_type::CONTROL);
    if (control_state.data.empty()) {
        control_state.data = "Unknown";
    }
    control_state_pub_.publish(control_state);
}

void UnicycleUgvRosNode::logStateChanges(::state_machine::StateId control_state,
                                         ::state_machine::StateId health_state) {
    if (control_state != last_logged_control_state_) {
        ROS_INFO("[UnicycleUgvRosNode] CONTROL state -> %s",
                 controller_.stateMachine().currentStateName(region_type::CONTROL).c_str());
        last_logged_control_state_ = control_state;
    }
    if (health_state != last_logged_health_state_) {
        ROS_INFO("[UnicycleUgvRosNode] HEALTH state -> %u", static_cast<unsigned>(health_state));
        last_logged_health_state_ = health_state;
    }
}

}  // namespace unicycle_ugv_controller
