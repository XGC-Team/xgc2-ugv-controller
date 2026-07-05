#include "unicycle_reference_trajectory/input/reference_input_producer.h"

#include <algorithm>
#include <cmath>
#include <utility>

namespace unicycle_reference_trajectory {
namespace {

geometry_msgs::Quaternion yawQuaternion(double yaw) {
    geometry_msgs::Quaternion q;
    if (!std::isfinite(yaw)) {
        yaw = 0.0;
    }
    q.w = std::cos(0.5 * yaw);
    q.z = std::sin(0.5 * yaw);
    return q;
}

uint16_t normalizedAnalyticType(uint16_t type) {
    if (type == unicycle_reference_trajectory_msgs::AnalyticReference::ANALYTIC_HOLD ||
        type == unicycle_reference_trajectory_msgs::AnalyticReference::ANALYTIC_CIRCLE ||
        type == unicycle_reference_trajectory_msgs::AnalyticReference::ANALYTIC_CIRCLE_ENTRY ||
        type == unicycle_reference_trajectory_msgs::AnalyticReference::ANALYTIC_FIGURE_EIGHT) {
        return type;
    }
    return unicycle_reference_trajectory_msgs::AnalyticReference::ANALYTIC_CIRCLE;
}

}  // namespace

ReferenceInputProducer::ReferenceInputProducer(ros::NodeHandle& nh,
                                               ReferenceTrajectoryRuntime& runtime,
                                               const std::string& analytic_topic,
                                               const std::string& waypoint_topic,
                                               const std::string& sampled_topic,
                                               const std::string& reset_topic, uint32_t queue_size,
                                               DefaultAnalyticReferenceConfig default_analytic)
    : runtime_(runtime), default_analytic_(default_analytic) {
    analytic_sub_ =
        nh.subscribe(analytic_topic, queue_size, &ReferenceInputProducer::analyticCallback, this);
    waypoint_sub_ =
        nh.subscribe(waypoint_topic, queue_size, &ReferenceInputProducer::waypointCallback, this);
    sampled_sub_ =
        nh.subscribe(sampled_topic, queue_size, &ReferenceInputProducer::sampledCallback, this);
    reset_sub_ =
        nh.subscribe(reset_topic, queue_size, &ReferenceInputProducer::resetCallback, this);
}

void ReferenceInputProducer::update(double now_sec) {
    if (!default_analytic_.enabled || default_analytic_sent_ || !std::isfinite(now_sec) ||
        runtime_.currentState() !=
            unicycle_reference_trajectory_msgs::ReferenceStatus::STATE_READY) {
        return;
    }
    publishDefaultAnalytic(now_sec);
}

void ReferenceInputProducer::analyticCallback(
    const unicycle_reference_trajectory_msgs::AnalyticReference::ConstPtr& msg) {
    if (!msg) {
        ROS_ERROR("[ReferenceInputProducer] Null analytic reference");
        return;
    }
    if (!runtime_.acceptAnalytic(*msg)) {
        ROS_WARN_THROTTLE(1.0, "[ReferenceInputProducer] Rejected analytic reference");
        return;
    }
    post(event_type::ANALYTIC_RECEIVED, "analytic_reference");
}

void ReferenceInputProducer::waypointCallback(
    const unicycle_reference_trajectory_msgs::WaypointReferenceRequest::ConstPtr& msg) {
    if (!msg) {
        ROS_ERROR("[ReferenceInputProducer] Null waypoint reference");
        return;
    }
    if (!runtime_.acceptWaypoint(*msg)) {
        ROS_WARN_THROTTLE(1.0, "[ReferenceInputProducer] Rejected waypoint request");
        return;
    }
    post(event_type::WAYPOINT_RECEIVED, "waypoint_reference");
}

void ReferenceInputProducer::sampledCallback(
    const unicycle_reference_trajectory_msgs::SampledReference::ConstPtr& msg) {
    if (!msg) {
        ROS_ERROR("[ReferenceInputProducer] Null sampled reference");
        return;
    }
    if (!runtime_.acceptSampled(*msg)) {
        ROS_WARN_THROTTLE(1.0, "[ReferenceInputProducer] Rejected sampled reference");
        return;
    }
    post(event_type::SAMPLED_RECEIVED, "sampled_reference");
}

void ReferenceInputProducer::resetCallback(const std_msgs::Empty::ConstPtr& msg) {
    (void)msg;
    runtime_.reset();
    default_analytic_sent_ = false;
    post(event_type::RESET_REQUESTED, "reset");
}

void ReferenceInputProducer::publishDefaultAnalytic(double now_sec) {
    unicycle_reference_trajectory_msgs::AnalyticReference msg;
    msg.header.stamp = ros::Time(now_sec);
    msg.request_id = default_analytic_.request_id;
    msg.trajectory_id = default_analytic_.trajectory_id;
    msg.revision = default_analytic_.revision;
    msg.analytic_type = normalizedAnalyticType(default_analytic_.analytic_type);
    msg.start_time = ros::Time(now_sec + std::max(0.0, default_analytic_.start_delay));
    msg.duration = std::isfinite(default_analytic_.duration) && default_analytic_.duration > 0.0
                       ? default_analytic_.duration
                       : 120.0;
    msg.origin.position.x = default_analytic_.origin_x;
    msg.origin.position.y = default_analytic_.origin_y;
    msg.origin.orientation = yawQuaternion(default_analytic_.origin_yaw);
    msg.params = {default_analytic_.radius, default_analytic_.line_speed,
                  default_analytic_.entry_duration, default_analytic_.center_x,
                  default_analytic_.center_y};
    if (!runtime_.acceptAnalytic(msg)) {
        ROS_WARN_THROTTLE(1.0, "[ReferenceInputProducer] Rejected default analytic reference");
        return;
    }
    default_analytic_sent_ = true;
    post(event_type::ANALYTIC_RECEIVED, "default_analytic_reference");
}

void ReferenceInputProducer::post(uint32_t event_id, const char* source) {
    ::state_machine::Event event(event_id,
                                 ::state_machine::EventTimestamp{ros::Time::now().toSec()});
    event.source = source;
    const auto status = runtime_.postEvent(std::move(event));
    if (!status.ok()) {
        ROS_WARN_THROTTLE(1.0, "[ReferenceInputProducer] Failed to post event %u from %s: %s",
                          event_id, source, status.message.c_str());
    }
}

}  // namespace unicycle_reference_trajectory
