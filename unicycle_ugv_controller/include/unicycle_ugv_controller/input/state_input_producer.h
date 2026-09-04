#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <rigid_state_estimator_msgs/RigidStateEstimate.h>
#include <ros/ros.h>

#include <functional>
#include <state_machine/state_machine.hpp>

#include "unicycle_ugv_controller/common/types.h"

namespace unicycle_ugv_controller {

class StateInputProducer {
   public:
    using EventSink = std::function<::state_machine::Status(::state_machine::Event)>;

    StateInputProducer(ros::NodeHandle& nh, UgvState& state, StateSource state_source,
                       const std::string& state_topic, const std::string& platform_pose_topic,
                       EventSink event_sink, uint32_t queue_size);

   private:
    void stateCallback(const rigid_state_estimator_msgs::RigidStateEstimate::ConstPtr& msg);
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void post(::state_machine::EventId id, const char* source, const ros::Time& stamp);

    UgvState& state_;
    StateSource state_source_{StateSource::STATE_ESTIMATOR};
    EventSink event_sink_;
    ros::Subscriber state_sub_;
    ros::Subscriber pose_sub_;
};

}  // namespace unicycle_ugv_controller
