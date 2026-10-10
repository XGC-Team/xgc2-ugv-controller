#pragma once

#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

#include <functional>
#include <state_machine/runtime/event_dispatcher.hpp>
#include <string>

#include "unicycle_ugv_controller/nmpc/nmpc_execution.h"
#include "unicycle_ugv_controller/unicycle_ugv_controller.h"

namespace unicycle_ugv_controller {

// Runs the NMPC solve requests of Custom1 (NmpcExecution) and publishes the predicted path.
class NmpcOutputConsumer final : public ::state_machine::runtime::EventConsumer {
   public:
    using EventSink = std::function<::state_machine::Status(::state_machine::Event)>;

    NmpcOutputConsumer(ros::NodeHandle& nh, UnicycleUgvController& controller, EventSink event_sink,
                       uint32_t queue_size);

    std::string name() const override {
        return "NmpcOutputConsumer";
    }
    bool handle(const ::state_machine::Event& event) override {
        return execution_.handle(event);
    }

   private:
    void publishPrediction(const Time& stamp, const NmpcTrackingBackend& backend);

    ros::Publisher predicted_path_pub_;
    ros::Publisher predicted_poses_pub_;
    NmpcExecution execution_;
};

}  // namespace unicycle_ugv_controller
