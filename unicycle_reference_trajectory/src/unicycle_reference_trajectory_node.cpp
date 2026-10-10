#include "unicycle_reference_trajectory/unicycle_reference_trajectory_node.h"

#include <algorithm>
#include <cmath>

#include "unicycle_reference_trajectory/config_loader.h"

namespace unicycle_reference_trajectory {
namespace {

// The node's private parameters as a configuration source (config_loader.h).
struct RosParamSource {
    const ros::NodeHandle& nh;

    template <typename T>
    bool get(const std::string& key, T& value) const {
        return nh.getParam(key, value);
    }
};

}  // namespace

ReferenceTrajectoryNode::ReferenceTrajectoryNode(ros::NodeHandle& nh)
    : nh_(nh), private_nh_("~"), output_executor_(nh_) {
    loadParams();
    driver_.configure(config_, default_analytic_);
    output_dispatcher_.addConsumer(std::make_unique<ReferenceOutputConsumer>(
        nh_, output_executor_, driver_.runtime(), status_topic_, active_analytic_topic_,
        active_polynomial_topic_, active_sampled_topic_, reference_path_topic_,
        reference_path_sample_dt_, reference_path_preview_duration_, queue_size_));
    input_producer_ = std::make_unique<ReferenceInputProducer>(
        nh_, driver_, analytic_topic_, waypoint_topic_, sampled_topic_, reset_topic_, queue_size_);
    output_executor_.start();
    ROS_INFO(
        "[ReferenceTrajectoryNode] Initialized: analytic=%s waypoint=%s sampled=%s "
        "status=%s active_analytic=%s active_polynomial=%s active_sampled=%s reference_path=%s",
        analytic_topic_.c_str(), waypoint_topic_.c_str(), sampled_topic_.c_str(),
        status_topic_.c_str(), active_analytic_topic_.c_str(), active_polynomial_topic_.c_str(),
        active_sampled_topic_.c_str(), reference_path_topic_.c_str());
}

ReferenceTrajectoryNode::~ReferenceTrajectoryNode() {
    output_executor_.stop();
}

void ReferenceTrajectoryNode::run(double main_frequency_hz) {
    const double frequency =
        std::isfinite(main_frequency_hz) && main_frequency_hz > 0.0 ? main_frequency_hz : 100.0;
    ROS_INFO("[ReferenceTrajectoryNode] Starting main loop at %.1f Hz", frequency);
    ros::Rate rate(frequency);
    while (ros::ok()) {
        ros::spinOnce();
        const auto update = driver_.update(ros::Time::now().toSec());
        if (update.default_analytic == ReferenceTrajectoryDriver::Request::kRejected) {
            ROS_WARN_THROTTLE(1.0, "[ReferenceTrajectoryNode] Rejected default analytic reference");
        } else if (update.default_analytic == ReferenceTrajectoryDriver::Request::kPostFailed) {
            ROS_WARN_THROTTLE(1.0,
                              "[ReferenceTrajectoryNode] Failed to post event from "
                              "default_analytic_reference: %s",
                              driver_.postError().c_str());
        }
        dispatchOutputEvents(update.events);
        rate.sleep();
    }
}

void ReferenceTrajectoryNode::loadParams() {
    int queue_size = static_cast<int>(queue_size_);
    private_nh_.param("queue_size", queue_size, queue_size);
    queue_size_ = static_cast<uint32_t>(std::max(1, queue_size));
    private_nh_.param("analytic_topic", analytic_topic_, analytic_topic_);
    private_nh_.param("waypoint_topic", waypoint_topic_, waypoint_topic_);
    private_nh_.param("sampled_topic", sampled_topic_, sampled_topic_);
    private_nh_.param("reset_topic", reset_topic_, reset_topic_);
    private_nh_.param("status_topic", status_topic_, status_topic_);
    private_nh_.param("active_analytic_topic", active_analytic_topic_, active_analytic_topic_);
    private_nh_.param("active_polynomial_topic", active_polynomial_topic_,
                      active_polynomial_topic_);
    private_nh_.param("active_sampled_topic", active_sampled_topic_, active_sampled_topic_);
    private_nh_.param("reference_path_topic", reference_path_topic_, reference_path_topic_);
    private_nh_.param("reference_path_sample_dt", reference_path_sample_dt_,
                      reference_path_sample_dt_);
    if (!std::isfinite(reference_path_sample_dt_) || reference_path_sample_dt_ <= 0.0) {
        reference_path_sample_dt_ = 0.5;
    }
    private_nh_.param("reference_path_preview_duration", reference_path_preview_duration_,
                      reference_path_preview_duration_);
    if (!std::isfinite(reference_path_preview_duration_) ||
        reference_path_preview_duration_ <= 0.0) {
        reference_path_preview_duration_ = 60.0;
    }

    loadReferenceConfig(RosParamSource{private_nh_}, config_, default_analytic_);
}

void ReferenceTrajectoryNode::dispatchOutputEvents(
    const std::vector<::state_machine::Event>& events) {
    const auto result = output_dispatcher_.dispatch(events);
    for (const auto& event : result.unhandled_events) {
        ROS_WARN("[ReferenceTrajectoryNode] Unhandled output event id: %u",
                 static_cast<unsigned>(event.id));
    }
    for (const auto& failure : result.failures) {
        ROS_WARN("[ReferenceTrajectoryNode] Output consumer '%s' failed on event %u: %s",
                 failure.consumer_name.c_str(), static_cast<unsigned>(failure.event.id),
                 failure.message.c_str());
    }
}

}  // namespace unicycle_reference_trajectory
