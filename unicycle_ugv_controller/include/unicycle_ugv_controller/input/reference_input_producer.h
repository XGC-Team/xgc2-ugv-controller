#pragma once

#include <ros/ros.h>
#include <unicycle_reference_trajectory_msgs/ActivePolynomialReference.h>
#include <unicycle_reference_trajectory_msgs/AnalyticReference.h>
#include <unicycle_reference_trajectory_msgs/SampledReference.h>

#include "unicycle_ugv_controller/common/reference_cache.h"

namespace unicycle_ugv_controller {

class ReferenceInputProducer {
   public:
    ReferenceInputProducer(ros::NodeHandle& nh, ReferenceCache& cache,
                           const std::string& active_analytic_topic,
                           const std::string& active_polynomial_topic,
                           const std::string& active_sampled_topic, uint32_t queue_size);

   private:
    void analyticCallback(
        const unicycle_reference_trajectory_msgs::AnalyticReference::ConstPtr& msg);
    void polynomialCallback(
        const unicycle_reference_trajectory_msgs::ActivePolynomialReference::ConstPtr& msg);
    void sampledCallback(const unicycle_reference_trajectory_msgs::SampledReference::ConstPtr& msg);

    ReferenceCache& cache_;
    ros::Subscriber active_analytic_sub_;
    ros::Subscriber active_polynomial_sub_;
    ros::Subscriber active_sampled_sub_;
};

}  // namespace unicycle_ugv_controller
