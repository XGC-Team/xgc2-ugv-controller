#include "unicycle_reference_trajectory/input/reference_input_producer.h"

#include "unicycle_reference_trajectory/ros_reference_conversion.h"

namespace unicycle_reference_trajectory {

ReferenceInputProducer::ReferenceInputProducer(ros::NodeHandle& nh,
                                               ReferenceTrajectoryDriver& driver,
                                               const std::string& analytic_topic,
                                               const std::string& waypoint_topic,
                                               const std::string& sampled_topic,
                                               const std::string& reset_topic, uint32_t queue_size)
    : driver_(driver) {
    analytic_sub_ =
        nh.subscribe(analytic_topic, queue_size, &ReferenceInputProducer::analyticCallback, this);
    waypoint_sub_ =
        nh.subscribe(waypoint_topic, queue_size, &ReferenceInputProducer::waypointCallback, this);
    sampled_sub_ =
        nh.subscribe(sampled_topic, queue_size, &ReferenceInputProducer::sampledCallback, this);
    reset_sub_ =
        nh.subscribe(reset_topic, queue_size, &ReferenceInputProducer::resetCallback, this);
}

void ReferenceInputProducer::analyticCallback(
    const unicycle_reference_trajectory_msgs::AnalyticReference::ConstPtr& msg) {
    if (!msg) {
        ROS_ERROR("[ReferenceInputProducer] Null analytic reference");
        return;
    }
    report(driver_.acceptAnalytic(toCore(*msg), ros::Time::now().toSec()), "analytic reference",
           "analytic_reference");
}

void ReferenceInputProducer::waypointCallback(
    const unicycle_reference_trajectory_msgs::WaypointReferenceRequest::ConstPtr& msg) {
    if (!msg) {
        ROS_ERROR("[ReferenceInputProducer] Null waypoint reference");
        return;
    }
    report(driver_.acceptWaypoint(toCore(*msg), ros::Time::now().toSec()), "waypoint request",
           "waypoint_reference");
}

void ReferenceInputProducer::sampledCallback(
    const unicycle_reference_trajectory_msgs::SampledReference::ConstPtr& msg) {
    if (!msg) {
        ROS_ERROR("[ReferenceInputProducer] Null sampled reference");
        return;
    }
    report(driver_.acceptSampled(toCore(*msg), ros::Time::now().toSec()), "sampled reference",
           "sampled_reference");
}

void ReferenceInputProducer::resetCallback(const std_msgs::Empty::ConstPtr& msg) {
    (void)msg;
    const auto status = driver_.reset(ros::Time::now().toSec());
    if (!status.ok()) {
        ROS_WARN_THROTTLE(1.0, "[ReferenceInputProducer] Failed to post event from reset: %s",
                          status.message.c_str());
    }
}

void ReferenceInputProducer::report(ReferenceTrajectoryDriver::Request result, const char* what,
                                    const char* source) {
    if (result == ReferenceTrajectoryDriver::Request::kRejected) {
        ROS_WARN_THROTTLE(1.0, "[ReferenceInputProducer] Rejected %s", what);
    } else if (result == ReferenceTrajectoryDriver::Request::kPostFailed) {
        ROS_WARN_THROTTLE(1.0, "[ReferenceInputProducer] Failed to post event from %s: %s", source,
                          driver_.postError().c_str());
    }
}

}  // namespace unicycle_reference_trajectory
