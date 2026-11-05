#include "unicycle_ugv_controller/input/pva_reference_input_producer.h"

#include <cmath>
#include <utility>

#include "unicycle_ugv_controller/common/types.h"
#include "unicycle_ugv_controller/ros_time_conversion.h"

namespace unicycle_ugv_controller {

PvaReferenceInputProducer::PvaReferenceInputProducer(ros::NodeHandle& nh,
                                                     UnicycleUgvController& controller,
                                                     const std::string& topic, uint32_t queue_size,
                                                     std::function<double()> clock)
    : controller_(controller), clock_(std::move(clock)) {
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
    reference.stamp = toCoreTime(ros::Time(clock_()));
    reference.x = msg->x;
    reference.y = msg->y;
    reference.yaw = std::isfinite(msg->yaw) ? wrapAngle(msg->yaw) : 0.0;
    reference.vx = msg->vx;
    reference.vy = msg->vy;
    reference.ax = msg->ax;
    reference.ay = msg->ay;
    reference.valid = true;
    controller_.setWorldPva(reference);
}

}  // namespace unicycle_ugv_controller
