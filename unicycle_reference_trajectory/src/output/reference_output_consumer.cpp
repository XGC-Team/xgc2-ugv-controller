#include "unicycle_reference_trajectory/output/reference_output_consumer.h"

#include <memory>
#include <utility>

namespace unicycle_reference_trajectory {
namespace {

template <typename Message>
std::unique_ptr<::state_machine::runtime::Task<ros::NodeHandle>> makePublishTask(
    std::string name, const ros::Publisher& pub, Message msg) {
    return std::make_unique<::state_machine::runtime::LambdaTask<ros::NodeHandle>>(
        std::move(name),
        [pub, msg = std::move(msg)](ros::NodeHandle&) mutable { pub.publish(msg); });
}

}  // namespace

ReferenceOutputConsumer::ReferenceOutputConsumer(
    ros::NodeHandle& nh, ::state_machine::runtime::AsyncTaskExecutor<ros::NodeHandle>& executor,
    ReferenceTrajectoryRuntime& runtime, const std::string& status_topic,
    const std::string& active_analytic_topic, const std::string& active_polynomial_topic,
    const std::string& active_sampled_topic, uint32_t queue_size)
    : executor_(executor), runtime_(runtime) {
    status_pub_ = nh.advertise<ReferenceStatus>(status_topic, queue_size, true);
    active_analytic_pub_ = nh.advertise<AnalyticReference>(active_analytic_topic, queue_size, true);
    active_polynomial_pub_ =
        nh.advertise<ActivePolynomialReference>(active_polynomial_topic, queue_size, true);
    active_sampled_pub_ = nh.advertise<SampledReference>(active_sampled_topic, queue_size, true);
}

bool ReferenceOutputConsumer::handle(const ::state_machine::Event& event) {
    if (event.id == output_event_type::PUBLISH_STATUS) {
        executor_.pushTask(
            makePublishTask("PublishReferenceStatus", status_pub_,
                            runtime_.makeStatus(event.timestamp > 0.0 ? event.timestamp
                                                                      : ros::Time::now().toSec())));
        return true;
    }
    if (event.id == output_event_type::PUBLISH_ACTIVE_ANALYTIC) {
        executor_.pushTask(makePublishTask("PublishActiveAnalytic", active_analytic_pub_,
                                           runtime_.activeAnalyticMessage()));
        return true;
    }
    if (event.id == output_event_type::PUBLISH_ACTIVE_POLYNOMIAL) {
        executor_.pushTask(makePublishTask("PublishActivePolynomial", active_polynomial_pub_,
                                           runtime_.activePolynomialMessage()));
        return true;
    }
    if (event.id == output_event_type::PUBLISH_ACTIVE_SAMPLED) {
        executor_.pushTask(makePublishTask("PublishActiveSampled", active_sampled_pub_,
                                           runtime_.activeSampledMessage()));
        return true;
    }
    return false;
}

}  // namespace unicycle_reference_trajectory
