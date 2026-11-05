#pragma once

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <functional>

#include "unicycle_ugv_controller/common/operator_command.h"
#include "unicycle_ugv_controller/common/types.h"

namespace unicycle_ugv_controller {

class CommandInputProducer {
   public:
    using CommandSink = std::function<CommandOutcome(OperatorCommand, const char*)>;

    CommandInputProducer(ros::NodeHandle& nh, CommandSink command_sink, uint32_t queue_size);

   private:
    void namespacedCommandCallback(const std_msgs::String::ConstPtr& msg);
    void publicCommandCallback(const std_msgs::String::ConstPtr& msg);
    void handleCommand(const std_msgs::String::ConstPtr& msg, const char* source);

    CommandSink command_sink_;
    ros::Subscriber command_sub_;
    ros::Subscriber namespaced_command_sub_;
};

}  // namespace unicycle_ugv_controller
