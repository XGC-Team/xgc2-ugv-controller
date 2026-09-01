#include <algorithm>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

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

bool tryYawFromQuaternion(double x, double y, double z, double w, double& yaw) {
    const double norm_squared = x * x + y * y + z * z + w * w;
    if (!std::isfinite(norm_squared) || norm_squared <= 1.0e-12) {
        return false;
    }
    const double inverse_norm = 1.0 / std::sqrt(norm_squared);
    yaw = yawFromQuaternion(x * inverse_norm, y * inverse_norm, z * inverse_norm, w * inverse_norm);
    return std::isfinite(yaw);
}

bool finiteState(const UgvState& state) {
    return std::isfinite(state.x) && std::isfinite(state.y) && std::isfinite(state.yaw) &&
           std::isfinite(state.speed) && std::isfinite(state.yaw_rate);
}

bool stateFresh(const UgvState& state, const ros::Time& now, double timeout) {
    constexpr double kFutureStampTolerance = 0.05;
    const double age = (now - state.stamp).toSec();
    return state.received && finiteState(state) && timeout > 0.0 && std::isfinite(age) &&
           age >= -kFutureStampTolerance && age <= timeout;
}

double clamp(double value, double min_value, double max_value) {
    return std::max(min_value, std::min(max_value, value));
}

UnicycleResetOutput computeUnicycleResetCommand(const UgvState& state, const ResetTarget& goal,
                                                const ControllerConfig& config) {
    UnicycleResetOutput output;
    if (!goal.valid || !finiteState(state)) {
        return output;
    }
    const double ex = goal.x - state.x;
    const double ey = goal.y - state.y;
    const double dist = std::hypot(ex, ey);
    const double path_yaw = std::atan2(ey, ex);
    const double forward_err = wrapAngle(path_yaw - state.yaw);
    const double reverse_err = wrapAngle(path_yaw + M_PI - state.yaw);
    const bool reverse =
        dist > config.reset_arrive_position && std::fabs(reverse_err) < std::fabs(forward_err);
    const double heading_err = reverse ? reverse_err : forward_err;
    const double yaw_err = wrapAngle(goal.yaw - state.yaw);
    output.position_ok = dist <= config.reset_arrive_position;
    output.yaw_ok = std::fabs(yaw_err) <= config.reset_arrive_yaw;
    output.settled = output.position_ok && output.yaw_ok &&
                     std::fabs(state.speed) <= config.reset_settle_speed &&
                     std::fabs(state.yaw_rate) <= config.reset_settle_yaw_rate;
    if (!output.position_ok) {
        const double align = clamp(std::cos(heading_err), 0.0, 1.0);
        double speed = config.reset_kp_along * dist * align;
        if (reverse) {
            speed = -speed;
        }
        output.linear_speed = clamp(speed, -config.reset_max_speed, config.reset_max_speed);
        output.angular_speed =
            clamp(config.reset_kp_heading * heading_err, -config.reset_max_yaw_rate,
                  config.reset_max_yaw_rate);
        return output;
    }
    output.linear_speed = 0.0;
    output.angular_speed =
        clamp(config.reset_kp_heading * yaw_err, -config.reset_max_yaw_rate, config.reset_max_yaw_rate);
    return output;
}

}  // namespace unicycle_ugv_controller
