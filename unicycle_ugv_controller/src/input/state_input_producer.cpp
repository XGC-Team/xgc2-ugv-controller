#include "unicycle_ugv_controller/input/state_input_producer.h"

#include <cmath>
#include <utility>

#include "unicycle_ugv_controller/common/rigid_to_unicycle.h"

namespace unicycle_ugv_controller {

StateInputProducer::StateInputProducer(ros::NodeHandle& nh, UgvState& state,
                                       StateSource state_source, const std::string& state_topic,
                                       const std::string& platform_pose_topic, EventSink event_sink,
                                       uint32_t queue_size)
    : state_(state), state_source_(state_source), event_sink_(std::move(event_sink)) {
    if (state_source_ == StateSource::STATE_ESTIMATOR) {
        state_sub_ =
            nh.subscribe(state_topic, queue_size, &StateInputProducer::stateCallback, this);
    } else {
        pose_sub_ =
            nh.subscribe(platform_pose_topic, queue_size, &StateInputProducer::poseCallback, this);
    }
}

void StateInputProducer::stateCallback(
    const rigid_state_estimator_msgs::RigidStateEstimate::ConstPtr& msg) {
    if (!msg) {
        ROS_ERROR("[UgvStateInputProducer] Received null state estimate");
        return;
    }
    const UnicycleProjection planar = projectRigidToUnicycle(*msg);
    state_.stamp = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    state_.x = planar.x;
    state_.y = planar.y;
    state_.yaw = planar.yaw;
    state_.vx = msg->velocity.x;
    state_.vy = msg->velocity.y;
    state_.speed = planar.speed;
    state_.yaw_rate = planar.yaw_rate;
    state_.estimator_state = msg->estimator_state;
    state_.estimator_flags = msg->flags;
    state_.received = true;
    state_.velocity_valid = std::isfinite(state_.vx) && std::isfinite(state_.vy);
    post(event_type::INPUT_STATE_UPDATED, "state_estimate", state_.stamp);
}

void StateInputProducer::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    if (!msg) {
        ROS_ERROR("[UgvStateInputProducer] Received null canonical pose");
        return;
    }
    double yaw = 0.0;
    if (!tryYawFromQuaternion(msg->pose.orientation.x, msg->pose.orientation.y,
                              msg->pose.orientation.z, msg->pose.orientation.w, yaw)) {
        ROS_WARN_THROTTLE(1.0, "[UgvStateInputProducer] Rejecting pose with invalid quaternion");
        return;
    }
    if (!std::isfinite(msg->pose.position.x) || !std::isfinite(msg->pose.position.y)) {
        return;
    }
    state_.x = msg->pose.position.x;
    state_.y = msg->pose.position.y;
    state_.yaw = yaw;
    state_.stamp = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    state_.estimator_state = 0U;
    state_.estimator_flags = 0U;
    state_.received = true;
    post(event_type::INPUT_STATE_UPDATED, "platform_pose", state_.stamp);
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
