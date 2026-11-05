#include "unicycle_ugv_controller/input/reset_target_input_producer.h"

#include <cmath>
#include <utility>

#include "unicycle_ugv_controller/common/types.h"

namespace unicycle_ugv_controller {

ResetTargetInputProducer::ResetTargetInputProducer(ros::NodeHandle& nh,
                                                   UnicycleUgvController& controller,
                                                   const std::string& topic, uint32_t queue_size)
    : controller_(controller) {
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
}

}  // namespace unicycle_ugv_controller
