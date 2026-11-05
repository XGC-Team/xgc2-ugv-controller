#include "unicycle_ugv_controller/input/command_input_producer.h"

#include <string>
#include <utility>

#include "unicycle_ugv_controller/common/operator_command.h"
#include "unicycle_ugv_controller/common/types.h"

namespace unicycle_ugv_controller {
CommandInputProducer::CommandInputProducer(ros::NodeHandle& nh, CommandSink command_sink,
                                           uint32_t queue_size)
    : command_sink_(std::move(command_sink)) {
    namespaced_command_sub_ =
        nh.subscribe("command", queue_size, &CommandInputProducer::namespacedCommandCallback, this);
    command_sub_ =
        nh.subscribe("/command", queue_size, &CommandInputProducer::publicCommandCallback, this);
}

void CommandInputProducer::namespacedCommandCallback(const std_msgs::String::ConstPtr& msg) {
    handleCommand(msg, "command");
}

void CommandInputProducer::publicCommandCallback(const std_msgs::String::ConstPtr& msg) {
    handleCommand(msg, "/command");
}

void CommandInputProducer::handleCommand(const std_msgs::String::ConstPtr& msg,
                                         const char* source) {
    if (!msg || msg->data.empty()) {
        ROS_WARN("[UgvCommandInputProducer] Ignoring empty command on %s", source);
        return;
    }
    const auto command = parseOperatorCommand(msg->data);
    if (command == OperatorCommand::kUnknown) {
        ROS_WARN("[UgvCommandInputProducer] Unknown command: %s on %s", msg->data.c_str(), source);
        return;
    }
    const auto outcome = command_sink_(command, source);
    if (outcome.ok()) {
        ROS_INFO("[UgvCommandInputProducer] Command applied: %u on %s", static_cast<unsigned>(command), source);
    } else {
        ROS_WARN("[UgvCommandInputProducer] Command unmatched/refused on %s: %s", source, outcome.message.c_str());
    }
}

}  // namespace unicycle_ugv_controller
