#pragma once

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <unicycle_reference_trajectory_msgs/PlanarPvaReference.h>

#include <cstdint>
#include <functional>
#include <state_machine/state_machine.hpp>
#include <string>

#include "unicycle_ugv_controller/unicycle_ugv_controller.h"

namespace unicycle_ugv_controller {

class PvaReferenceInputProducer {
   public:
    using EventSink = std::function<::state_machine::Status(::state_machine::Event)>;

    PvaReferenceInputProducer(ros::NodeHandle& nh, UnicycleUgvController& controller,
                              const std::string& topic, EventSink event_sink, uint32_t queue_size);

   private:
    void callback(const unicycle_reference_trajectory_msgs::PlanarPvaReference::ConstPtr& msg);
    void post(::state_machine::EventId id, const char* source, const ros::Time& stamp);

    UnicycleUgvController& controller_;
    EventSink event_sink_;
    ros::Subscriber sub_;
    ros::Publisher receipt_pub_;
    std::uint64_t receipt_sequence_ = 0;
};

}  // namespace unicycle_ugv_controller
