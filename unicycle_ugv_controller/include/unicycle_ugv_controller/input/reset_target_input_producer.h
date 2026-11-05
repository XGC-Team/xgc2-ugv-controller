#pragma once

#include <geometry_msgs/Pose2D.h>
#include <ros/ros.h>

#include <string>

#include "unicycle_ugv_controller/unicycle_ugv_controller.h"

namespace unicycle_ugv_controller {

class ResetTargetInputProducer {
   public:
    ResetTargetInputProducer(ros::NodeHandle& nh, UnicycleUgvController& controller,
                             const std::string& topic, uint32_t queue_size);

   private:
    void callback(const geometry_msgs::Pose2D::ConstPtr& msg);

    UnicycleUgvController& controller_;
    ros::Subscriber sub_;
};

}  // namespace unicycle_ugv_controller
