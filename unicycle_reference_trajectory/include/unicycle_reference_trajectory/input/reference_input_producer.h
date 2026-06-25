#pragma once

#include <ros/ros.h>

#include <string>

#include "std_msgs/Empty.h"
#include "unicycle_reference_trajectory/AnalyticReference.h"
#include "unicycle_reference_trajectory/SampledReference.h"
#include "unicycle_reference_trajectory/WaypointReferenceRequest.h"
#include "unicycle_reference_trajectory/unicycle_reference_trajectory_runtime.h"

namespace unicycle_reference_trajectory {

struct DefaultAnalyticReferenceConfig {
    bool enabled{false};
    uint32_t request_id{1U};
    uint32_t trajectory_id{1U};
    uint32_t revision{1U};
    uint16_t analytic_type{AnalyticReference::ANALYTIC_CIRCLE};
    double start_delay{0.5};
    double duration{120.0};
    double origin_x{0.0};
    double origin_y{0.0};
    double origin_yaw{0.0};
    double radius{3.0};
    double line_speed{1.0};
    double entry_duration{2.0};
    double center_x{0.0};
    double center_y{0.0};
};

class ReferenceInputProducer {
   public:
    ReferenceInputProducer(ros::NodeHandle& nh, ReferenceTrajectoryRuntime& runtime,
                           const std::string& analytic_topic, const std::string& waypoint_topic,
                           const std::string& sampled_topic, const std::string& reset_topic,
                           uint32_t queue_size,
                           DefaultAnalyticReferenceConfig default_analytic = {});

    void update(double now_sec);

   private:
    void analyticCallback(const AnalyticReference::ConstPtr& msg);
    void waypointCallback(const WaypointReferenceRequest::ConstPtr& msg);
    void sampledCallback(const SampledReference::ConstPtr& msg);
    void resetCallback(const std_msgs::Empty::ConstPtr& msg);
    void publishDefaultAnalytic(double now_sec);
    void post(uint32_t event_id, const char* source);

    ReferenceTrajectoryRuntime& runtime_;
    ros::Subscriber analytic_sub_;
    ros::Subscriber waypoint_sub_;
    ros::Subscriber sampled_sub_;
    ros::Subscriber reset_sub_;
    DefaultAnalyticReferenceConfig default_analytic_;
    bool default_analytic_sent_{false};
};

}  // namespace unicycle_reference_trajectory
