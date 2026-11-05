#pragma once

#include <ros/ros.h>
#include <functional>
#include <unicycle_reference_trajectory_msgs/PlanarPvaReference.h>

#include <string>

#include "unicycle_ugv_controller/unicycle_ugv_controller.h"

namespace unicycle_ugv_controller {

class PvaReferenceInputProducer {
   public:

    PvaReferenceInputProducer(ros::NodeHandle& nh, UnicycleUgvController& controller,
                              const std::string& topic, uint32_t queue_size, std::function<double()> clock);

   private:
    void callback(const unicycle_reference_trajectory_msgs::PlanarPvaReference::ConstPtr& msg);

    UnicycleUgvController& controller_;
    std::function<double()> clock_;
    ros::Subscriber sub_;
};

}  // namespace unicycle_ugv_controller
