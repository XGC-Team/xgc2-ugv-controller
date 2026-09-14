#include "unicycle_ugv_controller/input/pva_reference_input_producer.h"

#include <cmath>
#include <utility>

#include "unicycle_ugv_controller/common/pva_receipt.h"
#include "unicycle_ugv_controller/common/types.h"

namespace unicycle_ugv_controller {

PvaReferenceInputProducer::PvaReferenceInputProducer(ros::NodeHandle& nh,
                                                  UnicycleUgvController& controller,
                                                  const std::string& topic, EventSink event_sink,
                                                  uint32_t queue_size)
    : controller_(controller), event_sink_(std::move(event_sink)) {
    receipt_pub_ = nh.advertise<std_msgs::String>(nh.resolveName(topic) + "/receipt", 10, false);
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

    PvaReceipt receipt;
    receipt.source_sec = msg->header.stamp.sec;
    receipt.source_nsec = msg->header.stamp.nsec;
    receipt.received_sec = reference.stamp.sec;
    receipt.received_nsec = reference.stamp.nsec;
    receipt.source_sequence = msg->header.seq;
    receipt.received_sequence = ++receipt_sequence_;
    receipt.pva = {reference.x, reference.y, reference.yaw, reference.vx,
                   reference.vy, reference.ax, reference.ay};
    std_msgs::String trace;
    trace.data = serializePvaReceipt(receipt);
    receipt_pub_.publish(trace);
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
