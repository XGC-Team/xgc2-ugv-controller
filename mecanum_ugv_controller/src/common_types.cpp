#include <algorithm>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "mecanum_ugv_controller/common/types.h"

namespace mecanum_ugv_controller {

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
           std::isfinite(state.vx) && std::isfinite(state.vy) && std::isfinite(state.yaw_rate);
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

void worldVelocityToBody(double yaw, double v_wx, double v_wy, double& v_bx, double& v_by) {
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    v_bx = c * v_wx + s * v_wy;
    v_by = -s * v_wx + c * v_wy;
}

double headingRateToTarget(double yaw, double target_yaw, double kp_yaw, double max_yaw_rate) {
    const double error = wrapAngle(target_yaw - yaw);
    return clamp(kp_yaw * error, -max_yaw_rate, max_yaw_rate);
}

HolonomicResetOutput computeHolonomicResetCommand(const UgvState& state, const ResetTarget& goal,
                                                  const ControllerConfig& config) {
    HolonomicResetOutput output;
    if (!goal.valid || !finiteState(state)) {
        return output;
    }
    const double ex = goal.x - state.x;
    const double ey = goal.y - state.y;
    const double dist = std::hypot(ex, ey);
    const double yaw_err = wrapAngle(goal.yaw - state.yaw);
    const double v_wx =
        clamp(config.reset_kp_xy * ex, -config.reset_max_speed, config.reset_max_speed);
    const double v_wy =
        clamp(config.reset_kp_xy * ey, -config.reset_max_speed, config.reset_max_speed);
    worldVelocityToBody(state.yaw, v_wx, v_wy, output.linear_x, output.linear_y);
    output.angular_z =
        clamp(config.reset_kp_yaw * yaw_err, -config.reset_max_yaw_rate, config.reset_max_yaw_rate);
    output.position_ok = dist <= config.reset_arrive_position;
    output.yaw_ok = std::fabs(yaw_err) <= config.reset_arrive_yaw;
    const double speed = std::hypot(state.vx, state.vy);
    output.settled = output.position_ok && output.yaw_ok && speed <= config.reset_settle_speed &&
                     std::fabs(state.yaw_rate) <= config.reset_settle_yaw_rate;
    return output;
}

HolonomicTrackOutput computeHolonomicTrackCommand(const UgvState& state,
                                                  const WorldVelocityReference& reference,
                                                  const ControllerConfig& config) {
    HolonomicTrackOutput output;
    if (!reference.valid || !finiteState(state) || !std::isfinite(reference.vx) ||
        !std::isfinite(reference.vy)) {
        return output;
    }
    worldVelocityToBody(state.yaw, reference.vx, reference.vy, output.linear_x, output.linear_y);
    const double speed = std::hypot(output.linear_x, output.linear_y);
    if (speed > config.track_max_speed && speed > 0.0) {
        const double scale = config.track_max_speed / speed;
        output.linear_x *= scale;
        output.linear_y *= scale;
    }
    output.angular_z = headingRateToTarget(state.yaw, config.heading_target_yaw,
                                           config.track_kp_yaw, config.track_max_yaw_rate);
    return output;
}

}  // namespace mecanum_ugv_controller
