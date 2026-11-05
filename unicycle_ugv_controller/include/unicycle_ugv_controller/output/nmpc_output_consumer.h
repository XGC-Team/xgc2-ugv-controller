#pragma once

#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

#include <functional>
#include <string>

#include "unicycle_ugv_controller/nmpc/nmpc_execution.h"
#include "unicycle_ugv_controller/unicycle_ugv_controller.h"

namespace unicycle_ugv_controller {

// Runs the NMPC solve requests of Custom1 (NmpcExecution) and publishes the predicted path.
class NmpcOutputConsumer {
   public:
    using CompletionSink = NmpcExecution::CompletionSink;

    NmpcOutputConsumer(ros::NodeHandle& nh, UnicycleUgvController& controller, CompletionSink completion_sink,
                       NmpcExecution::Notify notify, uint32_t queue_size);

    void handle(const SolveNmpc& request) { execution_.handle(request); }

   private:
    void publishPrediction(const Time& time, const NmpcTrackingBackend& backend);

    ros::Publisher predicted_path_pub_;
    ros::Publisher predicted_poses_pub_;
    NmpcExecution execution_;
};

}  // namespace unicycle_ugv_controller
