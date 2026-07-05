#include "unicycle_ugv_controller/input/reference_input_producer.h"

#include <utility>

#include "unicycle_ugv_controller/common/types.h"

namespace unicycle_ugv_controller {

ReferenceInputProducer::ReferenceInputProducer(ros::NodeHandle& nh, ReferenceCache& cache,
                                               const std::string& active_analytic_topic,
                                               const std::string& active_polynomial_topic,
                                               const std::string& active_sampled_topic,
                                               EventSink event_sink, uint32_t queue_size)
    : cache_(cache), event_sink_(std::move(event_sink)) {
    active_analytic_sub_ = nh.subscribe(active_analytic_topic, queue_size,
                                        &ReferenceInputProducer::analyticCallback, this);
    active_polynomial_sub_ = nh.subscribe(active_polynomial_topic, queue_size,
                                          &ReferenceInputProducer::polynomialCallback, this);
    active_sampled_sub_ = nh.subscribe(active_sampled_topic, queue_size,
                                       &ReferenceInputProducer::sampledCallback, this);
}

void ReferenceInputProducer::analyticCallback(
    const unicycle_reference_trajectory_msgs::AnalyticReference::ConstPtr& msg) {
    if (!msg || !cache_.updateAnalytic(*msg, ros::Time::now())) {
        ROS_WARN_THROTTLE(1.0, "[UgvReferenceInputProducer] Rejected active analytic reference");
        return;
    }
    post(event_type::INPUT_REFERENCE_UPDATED, "active_analytic");
}

void ReferenceInputProducer::polynomialCallback(
    const unicycle_reference_trajectory_msgs::ActivePolynomialReference::ConstPtr& msg) {
    if (!msg || !cache_.updatePolynomial(*msg, ros::Time::now())) {
        ROS_WARN_THROTTLE(1.0, "[UgvReferenceInputProducer] Rejected active polynomial reference");
        return;
    }
    post(event_type::INPUT_REFERENCE_UPDATED, "active_polynomial");
}

void ReferenceInputProducer::sampledCallback(
    const unicycle_reference_trajectory_msgs::SampledReference::ConstPtr& msg) {
    if (!msg || !cache_.updateSampled(*msg, ros::Time::now())) {
        ROS_WARN_THROTTLE(1.0, "[UgvReferenceInputProducer] Rejected active sampled reference");
        return;
    }
    post(event_type::INPUT_REFERENCE_UPDATED, "active_sampled");
}

void ReferenceInputProducer::post(::state_machine::EventId id, const char* source) {
    if (!event_sink_) {
        ROS_ERROR("[UgvReferenceInputProducer] Event sink is not configured");
        return;
    }
    ::state_machine::Event event(id, ::state_machine::EventTimestamp{ros::Time::now().toSec()});
    event.source = source;
    event.category = ::state_machine::EventCategory::kInput;
    const auto status = event_sink_(std::move(event));
    if (!status.ok()) {
        ROS_WARN("[UgvReferenceInputProducer] Failed to post reference event: %s",
                 status.message.c_str());
    }
}

}  // namespace unicycle_ugv_controller
