#include "unicycle_ugv_controller/input/reference_input_producer.h"

#include <utility>

#include "unicycle_ugv_controller/common/types.h"
#include "unicycle_ugv_controller/ros_reference_conversion.h"

namespace unicycle_ugv_controller {

ReferenceInputProducer::ReferenceInputProducer(ros::NodeHandle& nh, ReferenceCache& cache,
                                               const std::string& active_analytic_topic,
                                               const std::string& active_polynomial_topic,
                                               const std::string& active_sampled_topic,
                                               uint32_t queue_size)
    : cache_(cache) {
    active_analytic_sub_ = nh.subscribe(active_analytic_topic, queue_size,
                                        &ReferenceInputProducer::analyticCallback, this);
    active_polynomial_sub_ = nh.subscribe(active_polynomial_topic, queue_size,
                                          &ReferenceInputProducer::polynomialCallback, this);
    active_sampled_sub_ = nh.subscribe(active_sampled_topic, queue_size,
                                       &ReferenceInputProducer::sampledCallback, this);
}

void ReferenceInputProducer::analyticCallback(
    const unicycle_reference_trajectory_msgs::AnalyticReference::ConstPtr& msg) {
    if (!msg || !cache_.updateAnalytic(toCoreReference(*msg))) {
        ROS_WARN_THROTTLE(1.0, "[UgvReferenceInputProducer] Rejected active analytic reference");
        return;
    }
}

void ReferenceInputProducer::polynomialCallback(
    const unicycle_reference_trajectory_msgs::ActivePolynomialReference::ConstPtr& msg) {
    if (!msg || !cache_.updatePolynomial(toCoreReference(*msg))) {
        ROS_WARN_THROTTLE(1.0, "[UgvReferenceInputProducer] Rejected active polynomial reference");
        return;
    }
}

void ReferenceInputProducer::sampledCallback(
    const unicycle_reference_trajectory_msgs::SampledReference::ConstPtr& msg) {
    if (!msg || !cache_.updateSampled(toCoreReference(*msg))) {
        ROS_WARN_THROTTLE(1.0, "[UgvReferenceInputProducer] Rejected active sampled reference");
        return;
    }
}

}  // namespace unicycle_ugv_controller
