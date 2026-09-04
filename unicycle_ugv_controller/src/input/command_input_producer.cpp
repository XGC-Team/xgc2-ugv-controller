#include "unicycle_ugv_controller/input/command_input_producer.h"

#include <algorithm>
#include <cctype>
#include <string>
#include <utility>

#include "unicycle_ugv_controller/common/types.h"

namespace unicycle_ugv_controller {
namespace {

std::string normalize(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return value;
}

}  // namespace

CommandInputProducer::CommandInputProducer(ros::NodeHandle& nh, EventSink event_sink,
                                           uint32_t queue_size)
    : event_sink_(std::move(event_sink)) {
    namespaced_command_sub_ =
        nh.subscribe("command", queue_size, &CommandInputProducer::commandCallback, this);
    command_sub_ =
        nh.subscribe("/command", queue_size, &CommandInputProducer::commandCallback, this);
}

void CommandInputProducer::commandCallback(const std_msgs::String::ConstPtr& msg) {
    if (!msg || msg->data.empty()) {
        ROS_WARN("[UgvCommandInputProducer] Ignoring empty command");
        return;
    }
    const std::string command = normalize(msg->data);
    if (command == "track" || command == "tracking" || command == "custom" ||
        command == "custom1" || command == "start") {
        ROS_INFO("[UgvCommandInputProducer] Accepted Custom1 command: %s", msg->data.c_str());
        post(event_type::CUSTOM1_REQUESTED, "command");
    } else if (command == "hold" || command == "stop") {
        ROS_INFO("[UgvCommandInputProducer] Accepted stop command: %s", msg->data.c_str());
        post(event_type::STOP_REQUESTED, "command");
    } else if (command == "reset") {
        ROS_INFO("[UgvCommandInputProducer] Accepted reset command");
        post(event_type::RESET_REQUESTED, "command");
    } else {
        ROS_WARN("[UgvCommandInputProducer] Unknown command: %s", msg->data.c_str());
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
    if (!status.ok()) {
        ROS_WARN("[UgvCommandInputProducer] Failed to post command event: %s",
                 status.message.c_str());
    }
}

}  // namespace unicycle_ugv_controller
