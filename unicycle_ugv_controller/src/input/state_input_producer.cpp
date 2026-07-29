#include "unicycle_ugv_controller/input/state_input_producer.h"

#include <cmath>
#include <utility>

namespace unicycle_ugv_controller {

StateInputProducer::StateInputProducer(ros::NodeHandle& nh, UgvState& state,
                                       StateSource state_source, const std::string& state_topic,
                                       const std::string& vrpn_pose_topic,
                                       const std::string& vrpn_twist_topic, EventSink event_sink,
                                       uint32_t queue_size)
    : state_(state), state_source_(state_source), event_sink_(std::move(event_sink)) {
    if (state_source_ == StateSource::VRPN_DIRECT) {
        vrpn_pose_sub_ =
            nh.subscribe(vrpn_pose_topic, queue_size, &StateInputProducer::vrpnPoseCallback, this);
        vrpn_twist_sub_ = nh.subscribe(vrpn_twist_topic, queue_size,
                                       &StateInputProducer::vrpnTwistCallback, this);
    } else {
        state_sub_ =
            nh.subscribe(state_topic, queue_size, &StateInputProducer::stateCallback, this);
    }
}

void StateInputProducer::stateCallback(
    const rigid_state_estimator_msgs::PlanarStateEstimate::ConstPtr& msg) {
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

void StateInputProducer::vrpnPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    if (!msg) {
        ROS_ERROR("[UgvStateInputProducer] Received null VRPN pose");
        return;
    }
    double yaw = 0.0;
    if (!tryYawFromQuaternion(msg->pose.orientation.x, msg->pose.orientation.y,
                              msg->pose.orientation.z, msg->pose.orientation.w, yaw)) {
        ROS_WARN_THROTTLE(1.0,
                          "[UgvStateInputProducer] Rejecting VRPN pose with invalid quaternion");
        return;
    }
    state_.x = msg->pose.position.x;
    state_.y = msg->pose.position.y;
    state_.yaw = yaw;
    vrpn_pose_stamp_ = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    vrpn_pose_received_ = true;
    updateVrpnState(vrpn_pose_stamp_);
}

void StateInputProducer::vrpnTwistCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
    if (!msg) {
        ROS_ERROR("[UgvStateInputProducer] Received null VRPN twist");
        return;
    }
    vrpn_velocity_x_ = msg->twist.linear.x;
    vrpn_velocity_y_ = msg->twist.linear.y;
    state_.yaw_rate = msg->twist.angular.z;
    vrpn_twist_stamp_ = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    vrpn_twist_received_ = true;
    updateVrpnState(vrpn_twist_stamp_);
}

void StateInputProducer::updateVrpnState(const ros::Time& update_stamp) {
    if (!vrpn_pose_received_ || !vrpn_twist_received_) {
        return;
    }
    state_.speed =
        std::cos(state_.yaw) * vrpn_velocity_x_ + std::sin(state_.yaw) * vrpn_velocity_y_;
    state_.stamp = vrpn_pose_stamp_ < vrpn_twist_stamp_ ? vrpn_pose_stamp_ : vrpn_twist_stamp_;
    if (state_.stamp.isZero()) {
        state_.stamp = update_stamp.isZero() ? ros::Time::now() : update_stamp;
    }
    state_.estimator_state = 0U;
    state_.estimator_flags = 0U;
    state_.received = true;
    post(event_type::INPUT_STATE_UPDATED, "vrpn_direct", state_.stamp);
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
