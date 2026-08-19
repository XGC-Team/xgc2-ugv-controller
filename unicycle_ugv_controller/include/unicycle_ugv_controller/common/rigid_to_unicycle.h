#pragma once

#include <rigid_state_estimator_msgs/RigidStateEstimate.h>

#include <cmath>
#include <cstdint>

namespace unicycle_ugv_controller {
namespace rigid_to_unicycle_detail {

inline double wrapAngle(double value) {
    return std::atan2(std::sin(value), std::cos(value));
}

inline double yawFromQuaternion(double x, double y, double z, double w) {
    const double siny_cosp = 2.0 * (w * z + x * y);
    const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    return wrapAngle(std::atan2(siny_cosp, cosy_cosp));
}

}  // namespace rigid_to_unicycle_detail

struct UnicycleProjection {
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
    double speed{0.0};
    double yaw_rate{0.0};
};

inline UnicycleProjection projectRigidToUnicycle(
    const rigid_state_estimator_msgs::RigidStateEstimate& msg) {
    UnicycleProjection out;
    out.x = msg.position.x;
    out.y = msg.position.y;
    out.yaw = rigid_to_unicycle_detail::yawFromQuaternion(msg.orientation.x, msg.orientation.y,
                                                          msg.orientation.z, msg.orientation.w);
    out.speed = std::cos(out.yaw) * msg.velocity.x + std::sin(out.yaw) * msg.velocity.y;
    out.yaw_rate = msg.angular_velocity.z;
    return out;
}

inline bool rigidEstimateHealthy(uint8_t estimator_state, uint32_t flags) {
    return estimator_state == rigid_state_estimator_msgs::RigidStateEstimate::STATE_RUNNING &&
           (flags & rigid_state_estimator_msgs::RigidStateEstimate::FLAG_FAULT) == 0U;
}

}  // namespace unicycle_ugv_controller
