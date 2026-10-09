#pragma once
#include <ugv_reset_safety/reset_session.h>
#include <ugv_reset_safety/scene_model.h>

#include <memory>
#include <string>
namespace ugv_reset_safety {
// Original ROS data inputs and sole cmd_vel publishers are owned by one fleet
// host. No reset topic, socket, thread or algorithm is introduced at this edge.
class FleetEdge {
   public:
    virtual ~FleetEdge() = default;
    virtual void updateOnce() = 0;
    virtual double controlRate() const = 0;
    virtual ResetSession& resetSession() = 0;
    virtual scene_model::PoseSample poseSample() const = 0;
    virtual std::string controlState() const = 0;
    virtual bool healthReady() const = 0;
    virtual bool reset() = 0;
    virtual bool stop() = 0;
    virtual bool admissionRejected() const = 0;
};
}  // namespace ugv_reset_safety
