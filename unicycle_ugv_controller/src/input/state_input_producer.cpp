#include "unicycle_ugv_controller/input/state_input_producer.h"

#include <utility>

namespace unicycle_ugv_controller {

StateInputProducer::StateInputProducer(ros::NodeHandle& nh, UgvState& state,
                                       const std::string& state_topic, EventSink event_sink,
                                       uint32_t queue_size)
    : state_(state), event_sink_(std::move(event_sink)) {
    state_sub_ = nh.subscribe(state_topic, queue_size, &StateInputProducer::stateCallback, this);
}

void StateInputProducer::stateCallback(
    const estimator_vrpn_ugv_state::PlanarStateEstimate::ConstPtr& msg) {
    if (!msg) {
        ROS_ERROR("[UgvStateInputProducer] Received null state estimate");
        return;
    }
    const double yaw = yawFromQuaternion(msg->orientation.x, msg->orientation.y, msg->orientation.z,
                                         msg->orientation.w);
    const double forward_speed = std::cos(yaw) * msg->velocity.x + std::sin(yaw) * msg->velocity.y;
    state_.stamp = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    state_.x = msg->position.x;
    state_.y = msg->position.y;
    state_.yaw = yaw;
    state_.speed = forward_speed;
    state_.yaw_rate = msg->angular_velocity.z;
    state_.estimator_state = msg->estimator_state;
    state_.estimator_flags = msg->flags;
    state_.received = true;
    post(event_type::INPUT_STATE_UPDATED, "state_estimate", state_.stamp);
}

void StateInputProducer::post(::state_machine::EventId id, const char* source,
                              const ros::Time& stamp) {
    if (!event_sink_) {
        ROS_ERROR("[UgvStateInputProducer] Event sink is not configured");
        return;
    }
    ::state_machine::Event event(id, ::state_machine::EventTimestamp{stamp.toSec()});
    event.source = source;
    event.category = ::state_machine::EventCategory::kInput;
    const auto status = event_sink_(std::move(event));
    if (!status.ok()) {
        ROS_WARN("[UgvStateInputProducer] Failed to post state event: %s", status.message.c_str());
    }
}

}  // namespace unicycle_ugv_controller
