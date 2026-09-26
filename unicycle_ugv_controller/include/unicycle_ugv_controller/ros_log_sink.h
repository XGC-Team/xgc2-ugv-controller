#pragma once

namespace unicycle_ugv_controller {

// Route the controller core's log (common/core_log.h) to rosconsole. The ROS
// node calls this once, before it constructs the controller.
void installRosLogSink();

}  // namespace unicycle_ugv_controller
