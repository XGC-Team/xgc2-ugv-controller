#pragma once

#include <rigid_state_estimator_msgs/PlanarStateEstimate.h>
#include <ros/ros.h>

#include <functional>
#include <state_machine/state_machine.hpp>

#include "unicycle_ugv_controller/common/types.h"

namespace unicycle_ugv_controller {

class StateInputProducer {
   public:
    using EventSink = std::function<::state_machine::Status(::state_machine::Event)>;

    StateInputProducer(ros::NodeHandle& nh, UgvState& state, const std::string& state_topic,
                       EventSink event_sink, uint32_t queue_size);

   private:
    void stateCallback(const rigid_state_estimator_msgs::PlanarStateEstimate::ConstPtr& msg);
    void post(::state_machine::EventId id, const char* source, const ros::Time& stamp);

    UgvState& state_;
    EventSink event_sink_;
    ros::Subscriber state_sub_;
};

}  // namespace unicycle_ugv_controller
