#include "unicycle_ugv_controller/ros_log_sink.h"

#include <ros/console.h>

#include "unicycle_ugv_controller/common/core_log.h"

namespace unicycle_ugv_controller {
namespace {

void rosSink(LogLevel level, const char* message) {
    switch (level) {
        case LogLevel::kError:
            ROS_ERROR("%s", message);
            break;
        case LogLevel::kWarn:
            ROS_WARN("%s", message);
            break;
        default:
            ROS_INFO("%s", message);
            break;
    }
}

}  // namespace

void installRosLogSink() {
    setLogSink(&rosSink);
}

}  // namespace unicycle_ugv_controller
