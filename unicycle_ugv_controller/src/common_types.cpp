#include <algorithm>
#include <cmath>

#include "unicycle_ugv_controller/common/types.h"

namespace unicycle_ugv_controller {

double wrapAngle(double value) {
    return std::atan2(std::sin(value), std::cos(value));
}

double yawFromQuaternion(double x, double y, double z, double w) {
    const double siny_cosp = 2.0 * (w * z + x * y);
    const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    return wrapAngle(std::atan2(siny_cosp, cosy_cosp));
}

bool finiteState(const UgvState& state) {
    return std::isfinite(state.x) && std::isfinite(state.y) && std::isfinite(state.yaw) &&
           std::isfinite(state.speed) && std::isfinite(state.yaw_rate);
}

bool stateFresh(const UgvState& state, const ros::Time& now, double timeout) {
    return state.received && finiteState(state) && timeout > 0.0 &&
           (now - state.stamp).toSec() <= timeout;
}

double clamp(double value, double min_value, double max_value) {
    return std::max(min_value, std::min(max_value, value));
}

}  // namespace unicycle_ugv_controller
