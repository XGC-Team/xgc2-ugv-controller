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
    accepted_pub_ = nh.advertise<unicycle_reference_trajectory_msgs::PlanarPvaReference>(
        topic + "/accepted", queue_size, false);
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
    // Preserve the existing receipt-epoch integration contract and tracking
    // law. An observer's bag time is not this receiver's time, and the input
    // header can name a future planning knot. Record the exact accepted epoch
    // so q(t)=q0+v0*tau+0.5*a0*tau^2 can be reconstructed without guessing it.
    // Acceptance means stored input, not proof of tracking-state admission,
    // command publication, or physical execution. Record those independently.
    auto accepted = *msg;
    accepted.header.stamp = reference.stamp;
    accepted.yaw = reference.yaw;
    accepted_pub_.publish(accepted);
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
