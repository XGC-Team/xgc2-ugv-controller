#include "unicycle_ugv_controller/input/reset_target_input_producer.h"

#include <cmath>
#include <utility>

#include "unicycle_ugv_controller/common/types.h"

namespace unicycle_ugv_controller {

ResetTargetInputProducer::ResetTargetInputProducer(ros::NodeHandle& nh,
                                                   UnicycleUgvController& controller,
                                                   const std::string& topic, EventSink event_sink,
                                                   uint32_t queue_size)
    : controller_(controller), event_sink_(std::move(event_sink)) {
    sub_ = nh.subscribe(topic, queue_size, &ResetTargetInputProducer::callback, this);
}

void ResetTargetInputProducer::callback(const geometry_msgs::Pose2D::ConstPtr& msg) {
    if (!msg || !std::isfinite(msg->x) || !std::isfinite(msg->y) || !std::isfinite(msg->theta)) {
        ROS_WARN("[UgvResetTargetInputProducer] Ignoring non-finite reset pose");
        return;
    }
    ResetTarget target;
    target.x = msg->x;
    target.y = msg->y;
    target.yaw = wrapAngle(msg->theta);
    target.valid = true;
    controller_.setResetTarget(target);
    ROS_INFO("[UgvResetTargetInputProducer] Reset target x=%.3f y=%.3f yaw=%.3f", target.x,
             target.y, target.yaw);
    post(event_type::INPUT_RESET_TARGET_UPDATED, "reset_pose");
}

void ResetTargetInputProducer::post(::state_machine::EventId id, const char* source) {
    if (!event_sink_) {
        return;
    }
    ::state_machine::Event event(id, ::state_machine::EventTimestamp{ros::Time::now().toSec()});
    event.source = source;
    event.category = ::state_machine::EventCategory::kInput;
    const auto status = event_sink_(std::move(event));
    if (!status.ok()) {
        ROS_WARN("[UgvResetTargetInputProducer] Failed to post event: %s", status.message.c_str());
    }
}

}  // namespace unicycle_ugv_controller
