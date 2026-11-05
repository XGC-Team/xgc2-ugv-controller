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

}  // namespace

UnicycleUgvRosNode::UnicycleUgvRosNode(ros::NodeHandle& nh)
    : nh_(nh), private_nh_("~"), controller_(state_) {
    loop_.attach(nh_);
    reset_edge_ = std::make_unique<ugv_reset_client::ResetRosEdge>(nh_, [this](const auto& grant) {
        controller_.observeInputs(loop_.now(), loop_.wallNow());
        controller_.onResetGrant(grant);
    });
    loadParams();
    controller_.setConfig(config_);
    seedResetTarget();

    auto command = [this](OperatorCommand input, const char* source) {
        command_turn_ = true;
        return controller_.command(input, loop_.now(), ros1_utils::AdaptiveOwnerLoop::wallNow(),
                                   source);
    };
    command_output_ = std::make_unique<CmdVelOutputConsumer>(
        nh_, controller_, cmd_vel_topic_, queue_size_, [this] { return loop_.now(); });
    if (config_.tracking_strategy == TrackingStrategy::NMPC) {
        nmpc_output_ = std::make_unique<NmpcOutputConsumer>(
            nh_, controller_, [this](NmpcCompletion result) { controller_.postCompletion(result); },
            [this] { loop_.notify(); }, queue_size_);
        reference_input_ = std::make_unique<ReferenceInputProducer>(
            nh_, controller_.referenceCache(), active_analytic_topic_, active_polynomial_topic_,
            active_sampled_topic_, queue_size_);
    } else {
        pva_reference_input_ = std::make_unique<PvaReferenceInputProducer>(
            nh_, controller_, pva_reference_topic_, queue_size_, [this] { return loop_.now(); });
    }
    control_state_pub_ = nh_.advertise<std_msgs::String>(control_state_topic_, queue_size_);
    command_input_ = std::make_unique<CommandInputProducer>(nh_, command, queue_size_);
    state_input_ = std::make_unique<StateInputProducer>(
        nh_, state_, config_.state_source, state_topic_, platform_pose_topic_, queue_size_,
        [this] { return loop_.now(); });
    reset_target_input_ = std::make_unique<ResetTargetInputProducer>(
        nh_, controller_, reset_pose_topic_, queue_size_);
    ROS_INFO(
        "[UnicycleUgvRosNode] Initialized: strategy=%s state_source=%s state=%s pose=%s "
        "pva=%s reset_pose=%s cmd_vel=%s",
        config_.tracking_strategy == TrackingStrategy::FLATNESS ? "flatness" : "nmpc",
        config_.state_source == StateSource::PLATFORM_POSE ? "platform_pose" : "state_estimator",
        state_topic_.c_str(), platform_pose_topic_.c_str(), pva_reference_topic_.c_str(),
        reset_pose_topic_.c_str(), cmd_vel_topic_.c_str());
}

UnicycleUgvRosNode::~UnicycleUgvRosNode() = default;

void UnicycleUgvRosNode::run(double frequency_hz) {
    config_.control_rate_hz = finitePositiveOr(frequency_hz, config_.control_rate_hz);
    controller_.setConfig(config_);
    controller_.setWallControlPhase(static_cast<int64_t>(ros::SteadyTime::now().toNSec()),
                                    loop_.externalClock());
    loop_.run([this] { updateOnce(); }, [this] { return controller_.nextWake(); });
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
    loadControllerConfig(RosParamSource(private_nh_), config_);
}

void UnicycleUgvRosNode::seedResetTarget() {
    const ResetTarget target = initialResetTarget(config_);
    if (target.valid) {
        controller_.setResetTarget(target);
    }
}

void UnicycleUgvRosNode::updateOnce() {
    if (!std::exchange(command_turn_, false)) {
        controller_.update(loop_.now(), ros1_utils::AdaptiveOwnerLoop::wallNow());
    }
    if (!controller_.lastResetAdmissionMiss().empty() &&
        controller_.lastResetAdmissionMiss() != last_logged_reset_miss_) {
        last_logged_reset_miss_ = controller_.lastResetAdmissionMiss();
        ROS_ERROR("[UnicycleUgvRosNode] %s", last_logged_reset_miss_.c_str());
    }
    dispatchOutputs(controller_.outputs());
    ugv_reset_client::Session session;
    if (controller_.takeResetSession(&session)) {
        reset_edge_->publish(session);
        controller_.resetSessionPublished();
    }
    ugv_reset_client::Result result;
    if (controller_.peekResetResult(&result)) {
        reset_edge_->publish(result);
        controller_.resetResultPublished();
    }
    logStateChanges();
    publishStatusIfDue();
}

void UnicycleUgvRosNode::dispatchOutputs(const std::vector<ControllerOutput>& outputs) {
    for (const auto& output : outputs) {
        std::visit(
            [this](const auto& value) {
                using T = std::decay_t<decltype(value)>;
                if constexpr (std::is_same_v<T, SolveNmpc>) {
                    if (nmpc_output_) {
                        nmpc_output_->handle(value);
                    }
                } else {
                    command_output_->handle(value);
                }
            },
            output);
    }
    if (controller_.resetExecutor().needsZeroPublication()) {
        command_output_->handle(PublishZero{controller_.currentTime()});
    }
}
void UnicycleUgvRosNode::publishStatusIfDue() {
    if (!control_state_pub_ || !controller_.statusDue()) {
        return;
    }
    std_msgs::String status;
    status.data = controller_.modeName();
    control_state_pub_.publish(status);
    controller_.statusPublished();
}
void UnicycleUgvRosNode::logStateChanges() {
    const auto mode = controller_.mode();
    if (!last_logged_control_state_ || mode != *last_logged_control_state_) {
        ROS_INFO("[UnicycleUgvRosNode] CONTROL state -> %s", controller_.modeName());
        last_logged_control_state_ = mode;
    }
}
}  // namespace unicycle_ugv_controller
