#include "unicycle_reference_trajectory/input/reference_input_producer.h"

#include "unicycle_reference_trajectory/ros_reference_conversion.h"

namespace unicycle_reference_trajectory {

ReferenceInputProducer::ReferenceInputProducer(
    ros::NodeHandle& nh, ReferenceTrajectoryDriver& driver, const std::string& analytic_topic,
    const std::string& waypoint_topic, const std::string& sampled_topic,
    const std::string& reset_topic, uint32_t queue_size, std::function<double()> clock)
    : driver_(driver), clock_(std::move(clock)) {
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
    report(driver_.acceptAnalytic(toCore(*msg), clock_()), "analytic reference",
           "analytic_reference");
}

void ReferenceInputProducer::waypointCallback(
    const unicycle_reference_trajectory_msgs::WaypointReferenceRequest::ConstPtr& msg) {
    if (!msg) {
        ROS_ERROR("[ReferenceInputProducer] Null waypoint reference");
        return;
    }
    report(driver_.acceptWaypoint(toCore(*msg), clock_()), "waypoint request",
           "waypoint_reference");
}

void ReferenceInputProducer::sampledCallback(
    const unicycle_reference_trajectory_msgs::SampledReference::ConstPtr& msg) {
    if (!msg) {
        ROS_ERROR("[ReferenceInputProducer] Null sampled reference");
        return;
    }
    report(driver_.acceptSampled(toCore(*msg), clock_()), "sampled reference", "sampled_reference");
}

void ReferenceInputProducer::resetCallback(const std_msgs::Empty::ConstPtr& msg) {
    (void)msg;
    driver_.reset(clock_());
}

void ReferenceInputProducer::report(ReferenceTrajectoryDriver::Request result, const char* what,
                                    const char*) {
    if (result == ReferenceTrajectoryDriver::Request::kRejected) {
        ROS_WARN_THROTTLE(1.0, "[ReferenceInputProducer] Rejected %s", what);
    }
}

}  // namespace unicycle_reference_trajectory
