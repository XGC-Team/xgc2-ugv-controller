#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <rigid_state_estimator_msgs/PlanarStateEstimate.h>
#include <ros/ros.h>

#include <functional>
#include <state_machine/state_machine.hpp>

#include "unicycle_ugv_controller/common/types.h"

namespace unicycle_ugv_controller {

class StateInputProducer {
   public:
    using EventSink = std::function<::state_machine::Status(::state_machine::Event)>;

    StateInputProducer(ros::NodeHandle& nh, UgvState& state, StateSource state_source,
                       const std::string& state_topic, const std::string& vrpn_pose_topic,
                       const std::string& vrpn_twist_topic, EventSink event_sink,
                       uint32_t queue_size);

   private:
    void stateCallback(const rigid_state_estimator_msgs::PlanarStateEstimate::ConstPtr& msg);
    void vrpnPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void vrpnTwistCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);
    void updateVrpnState(const ros::Time& update_stamp);
    void post(::state_machine::EventId id, const char* source, const ros::Time& stamp);

    UgvState& state_;
    StateSource state_source_{StateSource::STATE_ESTIMATOR};
    EventSink event_sink_;
    ros::Subscriber state_sub_;
    ros::Subscriber vrpn_pose_sub_;
    ros::Subscriber vrpn_twist_sub_;
    ros::Time vrpn_pose_stamp_;
    ros::Time vrpn_twist_stamp_;
    double vrpn_velocity_x_{0.0};
    double vrpn_velocity_y_{0.0};
    bool vrpn_pose_received_{false};
    bool vrpn_twist_received_{false};
};

}  // namespace unicycle_ugv_controller
