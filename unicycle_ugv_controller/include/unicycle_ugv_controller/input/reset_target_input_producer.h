#pragma once

#include <geometry_msgs/Pose2D.h>
#include <ros/ros.h>

#include <functional>
#include <state_machine/state_machine.hpp>
#include <string>

#include "unicycle_ugv_controller/unicycle_ugv_controller.h"

namespace unicycle_ugv_controller {

class ResetTargetInputProducer {
   public:
    using EventSink = std::function<::state_machine::Status(::state_machine::Event)>;

    ResetTargetInputProducer(ros::NodeHandle& nh, UnicycleUgvController& controller,
                             const std::string& topic, EventSink event_sink, uint32_t queue_size);

   private:
    void callback(const geometry_msgs::Pose2D::ConstPtr& msg);
    void post(::state_machine::EventId id, const char* source);

    UnicycleUgvController& controller_;
    EventSink event_sink_;
    ros::Subscriber sub_;
};

}  // namespace unicycle_ugv_controller
