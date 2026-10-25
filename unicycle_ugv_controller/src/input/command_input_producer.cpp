#include "unicycle_ugv_controller/input/command_input_producer.h"

#include <string>
#include <utility>

#include "unicycle_ugv_controller/common/operator_command.h"
#include "unicycle_ugv_controller/common/types.h"

namespace unicycle_ugv_controller {
CommandInputProducer::CommandInputProducer(ros::NodeHandle& nh, EventSink event_sink,
                                           uint32_t queue_size)
    : event_sink_(std::move(event_sink)) {
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
    switch (parseOperatorCommand(msg->data)) {
        case OperatorCommand::kCustom1:

            post(event_type::CUSTOM1_REQUESTED, source);
            break;
        case OperatorCommand::kStop:

            post(event_type::STOP_REQUESTED, source);
            break;
        case OperatorCommand::kReset:

            post(event_type::RESET_REQUESTED, source);
            break;
        case OperatorCommand::kUnknown:
            ROS_WARN("[UgvCommandInputProducer] Unknown command: %s on %s", msg->data.c_str(),
                     source);
            break;
    }
}

void CommandInputProducer::post(::state_machine::EventId id, const char* source) {
    if (!event_sink_) {
        ROS_ERROR("[UgvCommandInputProducer] Event sink is not configured");
        return;
    }
    ::state_machine::Event event(id, ::state_machine::EventTimestamp{ros::Time::now().toSec()});
    event.source = source;
    event.category = ::state_machine::EventCategory::kInput;
    const auto status = event_sink_(std::move(event));
    if (status.ok()) {
        ROS_INFO("[UgvCommandInputProducer] Command event queued: %u on %s",
                 static_cast<unsigned>(id), source);
    }
    if (!status.ok()) {
        ROS_WARN("[UgvCommandInputProducer] Failed to post command event: %s",
                 status.message.c_str());
    }
}

}  // namespace unicycle_ugv_controller
