#pragma once

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <unicycle_reference_trajectory_msgs/AnalyticReference.h>
#include <unicycle_reference_trajectory_msgs/SampledReference.h>
#include <unicycle_reference_trajectory_msgs/WaypointReferenceRequest.h>

#include <string>

#include "unicycle_reference_trajectory/reference_driver.h"

namespace unicycle_reference_trajectory {

// The ROS subscribers of the request topics. Each callback converts the
// message and hands it to the driver at the current ROS time.
class ReferenceInputProducer {
   public:
    ReferenceInputProducer(ros::NodeHandle& nh, ReferenceTrajectoryDriver& driver,
                           const std::string& analytic_topic, const std::string& waypoint_topic,
                           const std::string& sampled_topic, const std::string& reset_topic,
                           uint32_t queue_size);

   private:
    void analyticCallback(
        const unicycle_reference_trajectory_msgs::AnalyticReference::ConstPtr& msg);
    void waypointCallback(
        const unicycle_reference_trajectory_msgs::WaypointReferenceRequest::ConstPtr& msg);
    void sampledCallback(const unicycle_reference_trajectory_msgs::SampledReference::ConstPtr& msg);
    void resetCallback(const std_msgs::Empty::ConstPtr& msg);
    void report(ReferenceTrajectoryDriver::Request result, const char* what, const char* source);

    ReferenceTrajectoryDriver& driver_;
    ros::Subscriber analytic_sub_;
    ros::Subscriber waypoint_sub_;
    ros::Subscriber sampled_sub_;
    ros::Subscriber reset_sub_;
};

}  // namespace unicycle_reference_trajectory
