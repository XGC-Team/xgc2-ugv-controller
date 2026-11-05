#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <rigid_state_estimator_msgs/RigidStateEstimate.h>
#include <ros/ros.h>
#include <functional>


#include "unicycle_ugv_controller/common/types.h"

namespace unicycle_ugv_controller {

class StateInputProducer {
   public:

    StateInputProducer(ros::NodeHandle& nh, UgvState& state, StateSource state_source,
                       const std::string& state_topic, const std::string& platform_pose_topic,
                       uint32_t queue_size, std::function<double()> clock);

   private:
    void stateCallback(const rigid_state_estimator_msgs::RigidStateEstimate::ConstPtr& msg);
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

    UgvState& state_;
    StateSource state_source_{StateSource::STATE_ESTIMATOR};
    std::function<double()> clock_;
    ros::Subscriber state_sub_;
    ros::Subscriber pose_sub_;
};

}  // namespace unicycle_ugv_controller
