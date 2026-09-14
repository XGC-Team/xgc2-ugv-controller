#include "unicycle_ugv_controller/input/pva_reference_input_producer.h"

#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>

#include <cmath>
#include <iomanip>
#include <sstream>
#include <utility>

#include "unicycle_ugv_controller/common/pva_receipt_trace.h"
#include "unicycle_ugv_controller/common/types.h"

namespace unicycle_ugv_controller {

PvaReferenceInputProducer::PvaReferenceInputProducer(ros::NodeHandle& nh,
                                                     UnicycleUgvController& controller,
                                                     const std::string& topic, EventSink event_sink,
                                                     uint32_t queue_size)
    : controller_(controller), event_sink_(std::move(event_sink)) {
    ros::NodeHandle private_nh("~");
    bool publish_receipts = false;
    private_nh.param("publish_pva_receipts", publish_receipts, publish_receipts);
    if (publish_receipts) {
        receipt_pub_ = nh.advertise<std_msgs::Float64MultiArray>(nh.resolveName(topic) + "/receipt",
                                                                 queue_size);
        contract_pub_ =
            nh.advertise<std_msgs::String>(nh.resolveName(topic) + "/contract", 1, true);
        publishContract();
    }
    sub_ = nh.subscribe(topic, queue_size, &PvaReferenceInputProducer::callback, this);
}

void PvaReferenceInputProducer::publishContract() {
    const auto cfg = controller_.config();
    std::ostringstream out;
    out << std::setprecision(17) << "{\"schema\":\"unicycle.pva-receipt-contract/v1\","
        << "\"time_basis\":\"local_ros_receipt\","
        << "\"state_source\":\""
        << (cfg.state_source == StateSource::PLATFORM_POSE ? "platform_pose" : "state_estimator")
        << "\",\"tracking_strategy\":\""
        << (cfg.tracking_strategy == TrackingStrategy::FLATNESS ? "flatness" : "nmpc")
        << "\",\"command_publish_rate_hz\":" << cfg.command_publish_rate_hz
        << ",\"control_rate_hz\":" << cfg.control_rate_hz
        << ",\"chassis_max_linear_speed_mps\":" << cfg.chassis_max_linear_speed
        << ",\"chassis_max_yaw_rate_radps\":" << cfg.chassis_max_yaw_rate
        << ",\"flatness_kp_per_s2\":" << cfg.flatness_kp
        << ",\"flatness_kv_per_s\":" << cfg.flatness_kv
        << ",\"lateral_response_length_m\":" << cfg.flatness_lateral_response_length
        << ",\"lateral_damping\":" << cfg.flatness_lateral_damping
        << ",\"v_eps_mps\":" << cfg.flatness_v_eps << ",\"filter_wn_radps\":" << cfg.filter_wn
        << ",\"filter_zeta\":" << cfg.filter_zeta
        << ",\"velocity_dt_min_s\":" << cfg.velocity_dt_min
        << ",\"velocity_dt_max_s\":" << cfg.velocity_dt_max << "}";
    std_msgs::String message;
    message.data = out.str();
    contract_pub_.publish(message);
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
    if (receipt_pub_) {
        const auto fields = pvaReceiptTrace(
            reference.stamp.toSec(), msg->header.stamp.toSec(), msg->header.seq,
            ++receipt_sequence_,
            {{reference.x, reference.y, reference.vx, reference.vy, reference.ax, reference.ay}},
            reference.yaw);
        std_msgs::Float64MultiArray receipt;
        receipt.layout.dim.resize(1);
        receipt.layout.dim[0].label = pvaReceiptTraceLayout();
        receipt.layout.dim[0].size = fields.size();
        receipt.layout.dim[0].stride = fields.size();
        receipt.data.assign(fields.begin(), fields.end());
        receipt_pub_.publish(receipt);
    }
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
