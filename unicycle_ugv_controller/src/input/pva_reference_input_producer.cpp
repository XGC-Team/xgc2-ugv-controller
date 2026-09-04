#include "unicycle_ugv_controller/input/pva_reference_input_producer.h"

#include <cmath>
#include <utility>

#include "unicycle_ugv_controller/common/types.h"

namespace unicycle_ugv_controller {

PvaReferenceInputProducer::PvaReferenceInputProducer(ros::NodeHandle& nh,
                                                     UnicycleUgvController& controller,
                                                     const std::string& topic, EventSink event_sink,
                                                     uint32_t queue_size)
    : controller_(controller), event_sink_(std::move(event_sink)) {
    sub_ = nh.subscribe(topic, queue_size, &PvaReferenceInputProducer::callback, this);
}

void PvaReferenceInputProducer::callback(
    const unicycle_reference_trajectory_msgs::PlanarPvaReference::ConstPtr& msg) {
    if (!msg || !std::isfinite(msg->x) || !std::isfinite(msg->y) || !std::isfinite(msg->vx) ||
        !std::isfinite(msg->vy) || !std::isfinite(msg->ax) || !std::isfinite(msg->ay)) {
        ROS_WARN_THROTTLE(1.0, "[UgvPvaReferenceInputProducer] Rejecting non-finite PVA");
        return;
    }
    WorldPvaReference reference;
    reference.stamp = ros::Time::now();
    reference.x = msg->x;
    reference.y = msg->y;
    reference.yaw = std::isfinite(msg->yaw) ? wrapAngle(msg->yaw) : 0.0;
    reference.vx = msg->vx;
    reference.vy = msg->vy;
    reference.ax = msg->ax;
    reference.ay = msg->ay;
    reference.valid = true;
    controller_.setWorldPva(reference);
    post(event_type::INPUT_REFERENCE_UPDATED, "reference_pva", reference.stamp);
}

void PvaReferenceInputProducer::post(::state_machine::EventId id, const char* source,
                                     const ros::Time& stamp) {
    if (!event_sink_) {
        return;
    }
    ::state_machine::Event event(id, ::state_machine::EventTimestamp{stamp.toSec()});
    event.source = source;
    event.category = ::state_machine::EventCategory::kInput;
    const auto status = event_sink_(std::move(event));
    if (!status.ok()) {
        ROS_WARN("[UgvPvaReferenceInputProducer] Failed to post event: %s", status.message.c_str());
    }
}

}  // namespace unicycle_ugv_controller
